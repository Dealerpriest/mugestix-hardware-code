// Radio pipe addresses for the 6 nodes to communicate.
const uint8_t pipes[][6] = {"1ad",
                            "2ad",
                            "3ad",
                            "4ad",
                            "5ad",
                            "6ad",
                            "7ad"
                          };
const uint8_t lvl1hopPipes[][6] = { "8ad",
                                    "9ad",
                                    "aad",
                                    "bad",
                                    "cad",
                                    "dad",
                                    "ead"
                              };

const uint8_t lvl2hopPipes[][6] = { "fad",
                                    "gad",
                                    "had",
                                    "iad",
                                    "jad",
                                    "kad",
                                    "lad"
                              };

const uint8_t baseAddress[6] = "bas";
const uint8_t dummyAddress[6] = "baj";

const uint8_t channelSpread = 8;
const uint8_t startChannel = 100;


int packCounter[nrOfNodes] = {0,0,0,0,0,0};
int packsPerSec[nrOfNodes] = {0,0,0,0,0,0};
int totalPacksPerSec = 0;

unsigned long radioPrintStamp = 0;
unsigned long secStamp = 0;

volatile uint8_t sendStamp = 1;//incrementing packet id in order to make packets more unique

void openFirstPipe(uint8_t nodeNr){
  radio.openReadingPipe(1, pipes[nodeNr]);
}

void openSecondPipe(uint8_t nodeNr){
  radio.openReadingPipe(2, lvl1hopPipes[nodeNr]);
}

void resetPerformanceData(){
  for (int i = 0; i < nrOfNodes; ++i)
  {
    receivedPollPacketsOnPipe[i] = 0;
    receivedRelayPacketsOnPipe[i] = 0;
    pollFailsOnPipe[i] = 0;
    relayFailsOnPipe[i] = 0;
    maxTimeBetweenSuccesses[i] = 0;
  }
}

void printPerformance(unsigned long freq){
  if(millis() - radioPrintStamp > freq){
    radioPrintStamp = millis();

    printf("received/failed poll packets:\t");
    for (int i = 0; i < nrOfNodes; ++i)
    {
      printf("%5i/%5i\t", receivedPollPacketsOnPipe[i], pollFailsOnPipe[i]);
    }
    Serial.println();

    printf("received/failed relay packets:\t");
    for (int i = 0; i < nrOfNodes; ++i)
    {
      printf("%5i/%5i\t", receivedRelayPacketsOnPipe[i], relayFailsOnPipe[i]);
    }
    Serial.println();

    printf("retrieveDuration for last packet:\t");
    for (int i = 0; i < nrOfNodes; ++i)
    {
      printf("%5i micros\t\t", retrieveDuration[i]);
    }
    Serial.println();

    printf("longest wait between successes:\t");
    for (int i = 0; i < nrOfNodes; ++i)
    {
      printf("%5i micros\t\t", maxTimeBetweenSuccesses[i]);
    }
    Serial.println();

    Serial.print("packs/s:\t\t\t ");
    for (int i = 0; i < nrOfNodes; ++i)
    {
      printf("%5ip/s\t\t", packsPerSec[i]);
    }
    Serial.println();

    printf("total packs/s: %5i \n", totalPacksPerSec);

  }

  unsigned long duration = millis() - secStamp;
  if(duration > 50){
    secStamp = millis();
    totalPacksPerSec = 0;
    for (int i = 0; i < nrOfNodes; ++i)
    {
      packsPerSec[i] = packCounter[i]*(1000/duration);
      packCounter[i] = 0;
      totalPacksPerSec += packsPerSec[i];
    }
  }

}

void commonSetup(){
  radio.begin();
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.setChannel(100);
  // radio.setPayloadSize(32);
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.setAddressWidth(3);

}

void setupPoller(){
  radio.setRetries(2,2);
  radio.openReadingPipe(1, baseAddress);
}

void setupAcker(int nodeNr){
  radio.setRetries(2,2);
  // radio.setChannel(startChannel + nodeNr * channelSpread);

  printf("opening these readingpipes: %s, %s, %s \n", pipes[nodeNr], lvl1hopPipes[nodeNr], lvl2hopPipes[nodeNr]);
  radio.maskIRQ(1, 1, 0);//Only trigger irq-pin on rx interrupt.
  pinMode(radioIRQPin, INPUT_PULLUP);
  attachInterrupt(radioIRQPin, radioInterrupt, FALLING);
	radio.openReadingPipe(1, pipes[nodeNr]);
  radio.openReadingPipe(2, lvl1hopPipes[nodeNr]);
  radio.openReadingPipe(3, lvl2hopPipes[nodeNr]);
	radio.startListening();
}

bool pollNode(uint8_t node, uint8_t* commands, binaryInt16* receive){
  unsigned long startStamp = micros();

  // radio.setChannel(startChannel + node * channelSpread);
  // 
  uint8_t data[32] = {0};
  int i = 0;
  data[i++] = 'n';
  data[i++] = millis(); //unique dummy data. Edge goes here in relay requests
  memcpy(&data[i], commands, commandArraySize);
  i += commandArraySize;
  // data[i++] = command[0]; //Chosen sensor
  // data[i++] = command[1]; //Color
  // data[i++] = command[2]; //Brightness
  // data[i++] = command[3]; //Vibration motor counter
  data[i++] = incSendStamp();
  // data[4] = (millis() >> 8);
  // data[5] = (millis() >> 16);

  // printf("opening normal pipe for writing request: %s \n", pipes[node]);
  radio.openWritingPipe(pipes[node]);

  bool ok = radio.write( data, commandArraySize+3);
  uint8_t size;
  if(ok && handleReceivedAckPayload(receive, &size)){
    updateStats(node, true, false, startStamp);
    commandReceived[node] = true;

    // if(size > 16){
    //   receivedPollPacketsOnPipe[receive[8].c[0]]++;
    //   packCounter[receive[8].c[0]]++;
    // }else{
    // }
  }else{
    updateStats(node, false, false, startStamp);
    // Serial.println("pollNode failed");
    ok = false;
  }

  return ok;
}

//This returns an int represnting the result from relaying. 0 = didn't reach bridge. 1 = reached bridge but bridge didn't reach edge. 2 = total success
uint8_t blockingPollBridge(unsigned long timeout, uint8_t bridge, uint8_t* bridgeCommands, uint8_t edge, uint8_t* edgeCommands, binaryInt16* receive, binaryInt16* relayReceive){
  unsigned long startStamp = micros();

  // radio.setChannel(startChannel + bridge * channelSpread);

  uint8_t data[32] = {0};
  int i = 0;
  data[i++] = 'r';
  data[i++] = edge;
  memcpy(&data[i], bridgeCommands, commandArraySize);
  i += commandArraySize;
  memcpy(&data[i], edgeCommands, commandArraySize);
  i += commandArraySize;
  // data[i++] = command[0]; //Chosen sensor
  // data[i++] = command[1]; //Color
  // data[i++] = command[2]; //Brightness
  // data[i++] = command[3]; //Vibration motor counter
  data[i++] = incSendStamp();

  // printf("requesting from bridge %i and edge %i \n", bridge, edge);
  radio.openWritingPipe(lvl1hopPipes[bridge]);
  uint8_t result = radio.write( data, commandArraySize*2+3);
  uint8_t size;
  if(result && handleReceivedAckPayload(receive, &size)){
    updateStats(bridge, true, false, startStamp);
    commandReceived[bridge] = true;
    // uint8_t command1 = edge;
    uint8_t link = 0; //Link is assumed to fail until proven otherwise
    // printf("ack received. starting to wait for bridge \n");
    bool bridgeReply = waitForBridge(timeout, edge, &link, relayReceive);
    if(bridgeReply){
      // printf("bridge replied\n");
      if(link){
        // printf("hop sucess\n");
        updateStats(edge, true, true, startStamp);
        commandReceived[edge] = true;
        result = 3;
      }else{
        // printf("bridge %i didn't reach edge %i\n", bridge, edge);
        result = 2;
      }
    }else{
      result = 1;
      // printf("no (active) response from bridge %i\n", bridge);
    }
  }else{
    result = 0;
    // printf("bridge %i not reached at all\n", bridge);
    updateStats(edge, false, true, startStamp);
    updateStats(bridge, false, false, startStamp);
    // relayFailsOnPipe[edge]++;
    // pollFailsOnPipe[bridge]++;
  }
  return result;
}

bool pollEdgeAndSendToBase(uint8_t edge, uint8_t* commands){
  uint8_t data[5] = {0};
  int i = 0;
  data[i++] = 'e';
  data[i++] = millis();
  memcpy(&data[i], commands, commandArraySize+1);//adding 1 to include the original stamp from base
  // i += commandArraySize;
  // data[i++] = incSendStamp(); // we want to keep the stamp from the basestation instead of sending a new one.

  // radio.setChannel(startChannel + edge * channelSpread);

  // printf("opening hop lvl2 pipe for writing: %s \n", lvl2hopPipes[edge]);

  // printf("It took %lu microsecs from irq to before stopListening\n", micros() - irqStamp);
  radio.stopListening();
  radio.openWritingPipe(lvl2hopPipes[edge]);
  bool ok = radio.write( data, commandArraySize+3);
  // unsigned long dur = micros() - irqStamp;
  // printf("It took %lu microsecs from irq to finished writing to edge\n", dur);
  // printf("Trying to send rerouting result to base: ");
  
  // radio.setChannel(startChannel + role * channelSpread);

  binaryInt16 receive[8];

  uint8_t size;
  if(ok && handleReceivedAckPayload(receive, &size)){
    if(size < 1 || size > 32){
      ok = false;
      printf("Wrong size on received pack!! Size is: %i \n", size);
    }else
    if(sendToBase(edge, true, receive, size)){
      // printf("ok \n");
    }else{
      ok = false;
      // printf("failed \n");
    }
  }else{
    if(sendToBase(edge, false, receive, size)){
      // printf("ok \n");
    }else{
      // printf("failed \n");
    }
    // Serial.println("no response / no ackpack from edge");
  }
  radio.startListening();
  // data[0] = role;
  // data[1] = incSendStamp();
  // memcpy(&data[2], transmit, 14);
  // printf("Writing ackpack with stamp: %i.\n", sendStamp);
  // writeAckPacks(transmit, 16);
  return ok;
}

bool sendToBase(uint8_t originNode, bool link, binaryInt16* transmit, int size){
  uint8_t data[32] = {0};
  uint8_t pLength = size;
  data[0] = originNode;
  data[1] = (link ? incSendStamp() : 0);
  if(link){
    memcpy(&data[2], &transmit[1].c[0], size);
  }else{
    pLength = 3;
    data[2] = millis(); // Just to make packs more "unique" since we don't include stamp in this packet.
  }
  // printf("Trying ");
  // printf("to ");
  // printf("write to base\n");
  // radio.flush_tx();
  radio.openWritingPipe(baseAddress);
  bool ok = radio.write( data, pLength);
  if(ok){
    // Serial.println("write to base ok. ack received.");
    // printf("It took %lu microsecs from irq to delivered bridge response\n", (micros() - irqStamp));
  }else{
    Serial.println("sendToBasse failed");
  }

  //handleAckPayload(&receive);
  return ok;
}

//only rx interrupts will trigger this ISR. The others are masked in the radio chip setup.
//Be aware. We don't want this ISR to trigger when node is polling edge and receives ackpack.
//The way this is avoided now is that we're already in the ISR when polling edge.
//That could be considered superbad practice :-)
//But hey. It works...
//I think that the ISR will get queued and triggered directly afterwards though.
//But at that point the ackpack is already handled and the rx-fifo should be empty, effectively rendering the ISR "useless", exiting rather early.

FASTRUN void radioInterrupt(void){

  // printf("IRQ starts here ------------- \n");
  // irqStamp = micros();
  // unsigned long IrqStartStamp = micros();

  int bytesToSend = 12+nrOfSendSensors*3*2;
  uint8_t txData[32] = {0};
  txData[0] = role;
  txData[1] = incSendStamp();
  memcpy(&txData[2], (void*) transmitData, bytesToSend-2);

  radio.flush_tx();
  radio.writeAckPayload(1, txData, bytesToSend);
  radio.writeAckPayload(2, txData, bytesToSend);
  radio.writeAckPayload(3, txData, bytesToSend);

  bool tx,fail,rx;
  radio.whatHappened(tx,fail,rx);                     // What happened?
  // uint8_t pipe;
  if (rx){                      // Did we receive a message?
    
    uint8_t rxData[32];
    
    if(!radio.available()){//Exit if there is no message. This was probably a queued ISR.
      return;
    }
    while(radio.available()){
      uint8_t size = radio.getDynamicPayloadSize();
      if(size != 0){
        radio.read(rxData, size);
      }else{
        printf("payload has size 0. Means corrupt packet was received!\n");
      }
    }
    switch (rxData[0]){//First byte always describes type of message
      case 'r':{//This was a relaying request.
        // printf("This was a relay request to %i\n", rxData[1]);
        // relayPendingTo = rxData[1]+1;
        pollEdgeAndSendToBase(rxData[1], &rxData[2]+commandArraySize);
        break;
      }
      case 'e':
        //This was an edge request
        // printf("This was an edge request\n");
        break;
      case 'n':
        //This was a normal request
        // printf("This was a normal request\n");
        break;
      default:
        // printf("corrupt request type was %c (char), %i (int) \n", rxData[0], rxData[0]);
        break;
    }

    // uint8_t cmd = rxData[2];
    memcpy((void*) receivedCommands, &rxData[2], commandArraySize);
    // printf("command is: %c \n", cmd);
    // if(cmd != 0){
    //   sendSensors = cmd;
    //   // switch (cmd){
    //   //   case 'a':
    //   //     sendSensors = SEND_ACCEL;
    //   //     break;
    //   //   case 's':
    //   //     sendSensors = SEND_ACCEL_RAW;
    //   //     break;
    //   //   case 'g':
    //   //     sendSensors = SEND_GYRO;
    //   //     break;
    //   //   case 'h':
    //   //     sendSensors = SEND_GYRO_RAW;
    //   //     break;
    //   //   case 'm':  
    //   //     sendSensors = SEND_MAGNETOM;
    //   //     break;
    //   //   case ',':  
    //   //     sendSensors = SEND_MAGNETOM_RAW;
    //   //     break;
    //   // }
    //   // printf("Changing sendSensors to %i \n", sendSensors);
    // }

    // unsigned long IrqDuration = micros() - IrqStartStamp;
    // printf("rx-ISR of type %c took %lu microsecs\n", rxData[0], IrqDuration);
  }
  if(fail){
    // printf("fail irq was triggered \n");
  }
  if(tx){
    // printf("tx irq triggered \n");
  }

  // printf("triggered irq flags are fail: %i, rx: %i, tx: %i\n", fail, rx, tx);
  
}


// FASTRUN void baseStationInterrupt( void ){
//   irqStamp = micros();  
//   bool tx,fail,rx;
//   radio.whatHappened(tx,fail,rx);                     // What happened?
//   if(tx){//Packet is sent 

//   }else if(fail){ //Packet failed

//   }
// }

//This returns true if it reaches the bridge. The result of the bridge-edge connection is available in the link parameter.
bool waitForBridge(unsigned long timeout, uint8_t edge, uint8_t* link, binaryInt16* receive){
  // uint8_t pipe;
  *link = 0;
  bool result = false;
  radio.startListening();
  // delayMicroseconds(100);
  unsigned long startStamp = micros();
  // printf("timeout is %lu", timeout);
  while((micros() - startStamp) < timeout){
    // printf("entered timeout loop");
    if(radio.available()){
      // unsigned long waitTime = micros() - startStamp;
      // printf("waited o for approx. %lu microsecs for bridge response\n", waitTime);

      // uint8_t data[16] = {0};
      uint8_t size = radio.getDynamicPayloadSize();

      // printf("psize from bridge is %i \n", size);
      radio.read(&receive[0].c[0], size);
      // *edge = data[0];
      *link = receive[0].c[1];
      // memcpy(receive, data, 16);
      result = true;
      
      break;
    }
  }
  radio.stopListening();

  if(result && *link && receive[0].c[0] == edge){
  }else{
    *link = false;
  }
  return result;
}


//This function should be called directly after radio.write()
//Only call this function if you're prepared to discard packets laying in front of the expected ackPack in the rxfifo
bool handleReceivedAckPayload(binaryInt16* receive, uint8_t* size){
  uint8_t pipe;
  bool result = false;
  while(radio.available(&pipe) ){
    // printf("pack in pipe %i \n", pipe);
    *size = radio.getDynamicPayloadSize();
    radio.read(receive, *size);
    if(pipe == 0){ // ack packs must come on pipe 0!
      // printf("ack payload received from node %i with size %i and stamp: %i\n", receive[0].c[0], *size, receive[0].c[1]);
      result = true;
      break;
    }
  }
  return result;
}

// void writeAckPacks(binaryInt16* transmit, uint8_t len){
//   // printf("writing ack payloads of size %i\n", len);

//   uint8_t data[16];
//   data[0] = role;
//   data[1] = incSendStamp();
//   memcpy(&data[2], transmit, len-2);

  
//   radio.flush_tx();
//   radio.writeAckPayload(1, data, len);
//   radio.writeAckPayload(2, data, len);
//   radio.writeAckPayload(3, data, len);
// }

uint8_t incSendStamp(){
  sendStamp++;
  if(sendStamp == 0)//Don't allow zero. If zero, jump to 1.
    sendStamp++;

  return sendStamp;
}

void updateStats(uint8_t node, bool success, bool isEdge, unsigned long startStamp){
  if(success)
  {
    isResponding[node] = true;
    if(isEdge){
      receivedRelayPacketsOnPipe[node]++;
    }else{
      receivedPollPacketsOnPipe[node]++;
    }
    packCounter[node]++;

    unsigned long now = micros();

    if(successStamp[node] == 0){
      lastSuccessStamp[node] = now;
    }else{
      lastSuccessStamp[node] = successStamp[node];
    }
    successStamp[node] = now;
    maxTimeBetweenSuccesses[node] = max(maxTimeBetweenSuccesses[node], successStamp[node] - lastSuccessStamp[node]);

    retrieveDuration[node] = now - startStamp;
  }
  else
  {
    unsigned long noResponseDuration = micros() - successStamp[node];
    if(noResponseDuration > 5000000){
      tryReestablishStamp[node] = millis();
      isResponding[node] = false;
    }
    if(isEdge){
      relayFailsOnPipe[node]++;
    }else{
      pollFailsOnPipe[node]++;
    }

    retrieveDuration[node] = 0;
  }
}