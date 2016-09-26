#include "globals.h"
#include <Adafruit_NeoPixel.h>
#include <i2c_t3.h>
#include "MPU9250.h"
#include "sensorFusion.h"

#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

#include "helpers.h"

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"

#include <EEPROM.h>

#define VERSION 2.0f

#define GYRO_BIAS_ADDRESS 10


// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 
RF24 radio(14,10);
const uint8_t radioIRQPin = 15;

Adafruit_NeoPixel rgbLed = Adafruit_NeoPixel(1, 8, NEO_GRB + NEO_KHZ800);
Adafruit_BNO055 bno = Adafruit_BNO055();

const int motorPin = 22;

// RF24 radio2(17,19);
// const uint8_t radio2IRQPin = 21;

//
// Role management
//
// Set up role.  This sketch uses the same software for all the nodes
// in this system.  Doing so greatly simplifies testing.  
//

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "Node 0", "Node 1", "Node 2", "Node 3", "Node 4", "Node 5", "Node 6", "BaseStation"};

const int baseStation = 7;
const int8_t autoRole = -1; 
int currentNode = 0;

//Bitmasks
#define SEND_ACCEL        0x01
#define SEND_GYRO         0x02
#define SEND_MAGNETOM     0x04
#define SEND_ACCEL_RAW    0x08
#define SEND_GYRO_RAW     0x10
#define SEND_MAGNETOM_RAW 0x20

// #define SET_ORIGIN_QUAT   0x40
#define USE_BNO_FOR_QUAT  0x80
//*******************************************************
//CONFIG
//Configurable parameters here. Set role to autoRole to automatically read hardware pins to configure role
//Set manually to 0-5 or baseStation if you want to override auto assignment.
volatile int8_t role = autoRole;
//configure how manyy nodes we have
const int nrOfNodes = 7;
//if radio is not used the imu data will be spewed directly to serial port, rather than sent over radio.
const bool useRadio = true;
//Is nrf chip on pin 14? This configuration only applies if role is set manually (not autoRole)
// const bool NRFOnPin14 = false;
//Do we have a / should we use the, IMU
bool useMPU = true;
bool useBNO = true;

volatile int sendSensors = SEND_ACCEL;
volatile int nrOfSendSensors;

bool outputToProcessing = false;
bool printRadioPerformance = false;
bool printSensorDataOn = false;
bool outputMagCalibration = false;
bool printMagAccelAngleOn = false;
bool printBatteryLevel = false;

//******************************************************

///OCCUPIED PINS
// LASER RANGER: 5,6,7,18,19,20


MPU9250 mpu = MPU9250();


bool lastFakePinState = true;

//radio data hopping stuff
  //For nodes
  // volatile uint8_t relayPendingTo = 0;
  volatile unsigned long irqStamp = 0;
  // volatile bool waitingForTriggerAck = false;
  // volatile bool triggerAckReceived = false;

  //For base
  int16_t relayTarget[nrOfNodes] = {0};
  int16_t bridgeFor[nrOfNodes] = {0};
  bool bridgeActive[nrOfNodes] = {false};
  bool edgeActive[nrOfNodes] = {false};
  const unsigned long relayDuration = 250;

//radio message stuffs
//base
  binaryInt16 receivedData[16] = {0};
  binaryInt16 receivedRelayData[16] = {0};

  const int commandArraySize = 4;
  uint8_t currentCommands[nrOfNodes][commandArraySize];
  bool commandReceived[nrOfNodes];
//node
  volatile binaryInt16 transmitData[23] = {0}; //Should never be filled with more than 30 bytes, but make it bigger in case.
  volatile uint8_t receivedCommands[commandArraySize];

//Radio status information
int receivedPollPacketsOnPipe[nrOfNodes] = {0,0,0,0,0,0};
int receivedRelayPacketsOnPipe[nrOfNodes] = {0,0,0,0,0,0};
int pollFailsOnPipe[nrOfNodes] = {0,0,0,0,0,0};
int relayFailsOnPipe[nrOfNodes] = {0,0,0,0,0,0};
unsigned long normalPollFailedStamp[nrOfNodes] = {0};
unsigned long tryReestablishStamp[nrOfNodes] = {0,0,0,0,0,0,0};
unsigned long successStamp[nrOfNodes] = {0,0,0,0,0,0};
unsigned long lastSuccessStamp[nrOfNodes] = {0,0,0,0,0,0};
unsigned long maxTimeBetweenSuccesses[nrOfNodes] = {0,0,0,0,0,0};
unsigned long retrieveDuration[nrOfNodes] = {0,0,0,0,0,0};
bool sendTriggerAck[nrOfNodes] = {false};
bool triggerReceived[nrOfNodes] = {false};
binaryInt16 latestNodeValues[nrOfNodes][16] = {0};
bool isResponding[nrOfNodes] = {true, true, true, true, true, true, true};

//timing stuff
unsigned long mainLoopStamp = 0;
unsigned long mainLoopDuration = 0;
unsigned long mainLoopFrequency = 0;

unsigned long serialFrequency = 0;
unsigned long serialStamp = 0;

bool printDuringThisLap = false;
unsigned long printFrequency = 5;
unsigned long printStamp = 0;

unsigned long sensorFusionDuration = 0;

unsigned long midiStamp = 0;

//IMU stuff
bool bnoIsConnected = false;
imu::Quaternion quat;
uint8_t accelTriggerId = 0;
uint8_t triggerVelocity;
// bool gyroBiasUpdated = false;
// bool magBiasUpdated = false;
// float accelAverage[3];
// int accelNrOfSamples = 0;
// float accelMin[3];
// float accelMax[3];
float gravityMagnitude = 1.0;
float gravity[3] = {0, 0, gravityMagnitude};
float originQuat[4] = {1.0, 0.0, 0.0, 0.0};
bool  originQuatSet = false;
float fromOriginQuat[4];
bool gyroBiasSaved = false;

//Hardware stuff
float batteryLevel = 500;
float batteryLevelFilterC = 0.01;

uint8_t motorLevel = 0;
unsigned long motorOnStamp = 0;
const unsigned long motorDuration = 40;
uint8_t motorCounter = 0;

//Serial stuff
bool  sendQueue[nrOfNodes] = {false};
int   pendingNodesInSendQueue = 0;
  // uint8_t triggerIds[nrOfNodes];

void setup() {
  noInterrupts();

  //Led
  delay(1);
  rgbLed.begin();
  rgbLed.show();
  rgbLed.setPixelColor(0, rgbLed.Color(0,50,0));
  rgbLed.show();

  

  rgbLed.setPixelColor(0, rgbLed.Color(0,0,50));
  rgbLed.show();

  delay(1000);
  Serial.begin(115200);
  delay(10);

  printf("\n\rMugestic Firmware started!\n\r");

  //Handle EEPROM Version stuff
  float versionStampInMem;
  EEPROM.get(0, versionStampInMem);
  if(versionStampInMem != VERSION){
    initialiseEEPROM();
    EEPROM.put(0, VERSION);
    printf("New version number. Initializing EEPROM\n");
  }

  //YELLOW
  rgbLed.setPixelColor(0, rgbLed.Color(25,25,0));
  rgbLed.show();
  

  ///Faking radiosilence
  pinMode(23, INPUT_PULLUP);

  bejta = 3.0f;

  // randomSeed(analogRead(A13));

  for(int i = 0; i < nrOfNodes; ++i){
    commandReceived[i] = true;
    currentCommands[i][0] = sendSensors;
    currentCommands[i][1] = 170;
    currentCommands[i][2] = 1;
  }

  //RED
  rgbLed.setPixelColor(0, rgbLed.Color(50,0,0));
  rgbLed.show();

  //Hardware
  //Motor
  pinMode(motorPin, OUTPUT);



  //ROLE. Do this first so everything that relies on it works properly
  if(role == autoRole){
    role = 0;
    //Reads the pins 2-4 as a binary bcd number.
    for (int i = 2; i < 2+3; ++i)
    {
      pinMode(i, INPUT_PULLUP);
      delay(2);
      if(!digitalRead(i)){
        role = role | 1 << (i-2);
      }
    }
    //The pin config starts with node 0 having bcd 1. So, decrease with one to align index.
    role --;
    //If we are negative after reading pins and aligning index, that means we had bcd 0 = baseStation.
    if(role < 0)
      role = baseStation;
  }

  printf("ROLE: %s\n\r",role_friendly_name[role]);

  if(role == baseStation){
    // rgbLed.updateLength(8);
    // rgbLed.setPixelColor(0, getColor(255, 255));
    // rgbLed.show();
  }else{
    //Inititate Bno
    if(useBNO){
      if(bno.begin())
      {
        bno.setExtCrystalUse(true); 
        bnoIsConnected = true;
      }else{
        useBNO = false;
        /* There was a problem detecting the BNO055 ... check your connections */
        printf("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
      }
    }

    ///Initiate mpu
    if(useMPU){
      mpu.init(role);
      delay(50);
      if(mpu.isConnected()){
        Serial.println("MPU connected!");
      }else{
        Serial.println("MPU not found!");
        useMPU = false;
      }
      Serial.println("accgyro self test...");
      float selftestvalues[6];
      mpu.accGyroSelfTest(selftestvalues);
      for (int i = 0; i < 6; ++i)
      {
        Serial.print(selftestvalues[i]);
        Serial.print('\t');
      }
      Serial.println();
      mpu.initAccGyro();

      //Fetch saved gyrobiases
      if(fetchGyroBiases()){
        printf("fetched gyroBiases from EEPROM. gb[0]: %f, gb[1]: %f, gb[2]: %f \n", mpu.gyroBias[0], mpu.gyroBias[1], mpu.gyroBias[2]);
      }else{
        printf("Invalid gyrobias data in EEPROM. Not fetching biases. For best performance, lay the device still for one minute, until the led lights up\n");
      }
      
      mpu.initMag();
      // delay(20);
    }
  }

  rgbLed.setPixelColor(0, rgbLed.Color(0,25,25));
  rgbLed.show();

  //Radio Stuff!
  if(useRadio){
    printf("Setting up radio\n");
    commonSetup();
    if(role == baseStation){
      setupPoller();    
      // setupMultiReceiver();
    }else{
      setupAcker(role);
      // setupTransmitter(role);
    }
    radio.printDetails();
  }

  //OFF
  rgbLed.setPixelColor(0, rgbLed.Color(0,0,0));
  rgbLed.show();
  interrupts();
}


void loop()
{
  //Read serial commands
  uint8_t c = 0;
  if(Serial.available()){
      c = Serial.read();
  }
  
  setPrintFlag();//Call this every lap to set printflag

  mainLoopDuration = micros() - mainLoopStamp;
  mainLoopFrequency = 1000000/mainLoopDuration;
  mainLoopStamp = micros();

  // printLoopTimingInfo();

  nrOfSendSensors = numberOfSetBits(sendSensors);

  ///SHOULD WE READ THE IMU?
  if(useMPU && role != baseStation){
    //IMU STUFF
    ////////////////////////////////////////////////////
    // if(mpu.newSensorDataAvailable())
    {
      unsigned long sensorFusionStartStamp = micros();

      mpu.readAll();

      Now = micros();
      deltat = ((Now - lastUpdate)/1000000.0f); // set integration time by time elapsed since last filter update
      //Changed the fusion algotithms to return bool for if it finishes. This is due to having an interrupt possibly abort the fusion.
      //If the fusion indeed was aborted, we should not update lastupdate (since we never finished the update)
      // printf("Sensor Data: mx = %f, my = %f, mz = %f, ax = %f, ay = %f, az = %f, gx = %f, gy = %f, gz = %f \n", mpu.magnetom[0], mpu.magnetom[1], mpu.magnetom[2], mpu.accel[0], mpu.accel[1], mpu.accel[2], mpu.gyro[0], mpu.gyro[1], mpu.gyro[2]);
      

      MadgwickAHRSupdate(mpu.accel[0], mpu.accel[1], mpu.accel[2],
                              (mpu.gyro[0]), (mpu.gyro[1]), (mpu.gyro[2]),
                              mpu.magnetom[0], mpu.magnetom[1], mpu.magnetom[2]
                              );
      lastUpdate = Now;

      updateMotor();
      updateGravityVector();
      updateAccel(); //sendAccelTrigger is set in here

      // if(originQuatSet){
      //   quat_mult(q, originQuat, fromOriginQuat);
      //   quat_norm(fromOriginQuat);
      //   float angleFromOrigin = abs(quat_angle(fromOriginQuat));
      //   bejta = (PI/2 - (min(angleFromOrigin, PI/2)))*4;
      //   // if(TO_DEG(angleFromOrigin) > 10){
      //   //   printf("using IMU version for fusion\n");
      //   //   mpu.magnetom[0] = 0.0f;
      //   //   mpu.magnetom[1] = 0.0f;
      //   //   mpu.magnetom[2] = 0.0f;
      //   // }

      //   // VERSION 2
      //   // float euler[3];
      //   // quat_eulerAngles(q, euler);
      //   // float pitch = abs(TO_DEG(euler[1]));
      //   // bejta = (max(pitch, 70)-70)/14;

      //   // printf("bejta: %f\n", bejta);
      // }

      if(useBNO){
        quat = bno.getQuat();
      }
      

      //Handling setting  of sensor commands
      uint8_t receivedSensorCommand = receivedCommands[0];
      if(receivedSensorCommand & ~USE_BNO_FOR_QUAT){
        sendSensors = receivedSensorCommand;
      }

      // if(receivedSensorCommand & USE_BNO_FOR_QUAT){
      //   useBNO = true;
      // }

      // if(receivedCommands[0] & SET_ORIGIN_QUAT){
      //   quat_copy(q, originQuat);
      //   quat_norm(originQuat);
      //   quat_conj(originQuat);
      //   triggerMotor();
      // }

      if(receivedCommands[3] != motorCounter){
        motorCounter = receivedCommands[3];
        triggerMotor();
      }


      if(millis() > 6000 && millis() < 6010){
        bejta = betaDef; // We did set bejta very high in setup to quickly converge to earth frame. But now we should be rather aligned and can set bejta to default.
        // for(int i = 0; i<3; ++i){
        //   mpu.gyroBias[i] = mpu.filteredGyroRaw[i];
        // }
        // printf("noMotion active\n");
        // quat_copy(q, originQuat);
        // quat_norm(originQuat);
        // quat_conj(originQuat);
        // originQuatSet = true;
        // rgbLed.setPixelColor(0, getHue(140));
        // rgbLed.setBrightness(255);
      }

      if(!mpu.isCurrentlyMoving() && !gyroBiasSaved){
        for(int i = 0; i<3; ++i){
          mpu.gyroBias[i] = mpu.filteredGyroRaw[i];
        }
        writeGyroBiasToEEPROM();
        gyroBiasSaved = true;

        rgbLed.setPixelColor(0, getColor(140, 255));
        // rgbLed.setBrightness(255);
        printf("no move\n");
      }else if(gyroBiasSaved){
        rgbLed.setPixelColor(0, getColor(190, 255));
        // rgbLed.setBrightness(255);
      }else{
        rgbLed.setPixelColor(0, getColor(receivedCommands[1], receivedCommands[2]));
        // rgbLed.setBrightness(receivedCommands[2]);
      }
      rgbLed.show();
      
      

      //transmitData is read and written to by ISR, so protect it when we update it
      noInterrupts();
      int i = 0;
      if(!useBNO){
        transmitData[i++].i = floatToQ14(q[0]); //0 startindex for quaternion
        transmitData[i++].i = floatToQ14(q[1]);
        transmitData[i++].i = floatToQ14(q[2]);
        transmitData[i++].i = floatToQ14(-q[3]);
      }else{
        transmitData[i++].i = floatToQ14(quat.w()); //0 startindex for quaternion
        transmitData[i++].i = floatToQ14(quat.y());
        transmitData[i++].i = floatToQ14(quat.x());
        transmitData[i++].i = floatToQ14(quat.z());
      }

      transmitData[i].c[0] = accelTriggerId; // This id will get incremented for each acceltrigger. Thus the basestation will be able to identify when there is a new trigger
      transmitData[i++].c[1] = triggerVelocity;

      if(sendSensors & SEND_ACCEL){
        transmitData[i++].i = (int16_t)(mpu.accel[0]*1000); //4 startindex for accel
        transmitData[i++].i = (int16_t)(mpu.accel[1]*1000);
        transmitData[i++].i = (int16_t)(mpu.accel[2]*1000);
      }
      if(sendSensors & SEND_GYRO){
        transmitData[i++].i = (int16_t)(mpu.gyro[0]*1000); //4 startindex for gyro
        transmitData[i++].i = (int16_t)(mpu.gyro[1]*1000);
        transmitData[i++].i = (int16_t)(mpu.gyro[2]*1000);
      }
      if(sendSensors & SEND_MAGNETOM){
        transmitData[i++].i = (int16_t)(mpu.magnetom[0]*10.0); //4 startindex for magnetom
        transmitData[i++].i = (int16_t)(mpu.magnetom[1]*10.0);
        transmitData[i++].i = (int16_t)(mpu.magnetom[2]*10.0);
      }
      if(sendSensors & SEND_ACCEL_RAW){
        transmitData[i++].i = (int16_t)(mpu.accelRaw[0]*1000); //4 startindex for accel
        transmitData[i++].i = (int16_t)(mpu.accelRaw[1]*1000);
        transmitData[i++].i = (int16_t)(mpu.accelRaw[2]*1000);
      }
      if(sendSensors & SEND_GYRO_RAW){
        transmitData[i++].i = (int16_t)(mpu.gyroRaw[0]*100); //4 startindex for gyro
        transmitData[i++].i = (int16_t)(mpu.gyroRaw[1]*100);
        transmitData[i++].i = (int16_t)(mpu.gyroRaw[2]*100);
      }
      if(sendSensors & SEND_MAGNETOM_RAW){
        transmitData[i++].i = (int16_t)(mpu.magnetomRaw[0]*10.0); //4 startindex for magnetom
        transmitData[i++].i = (int16_t)(mpu.magnetomRaw[1]*10.0);
        transmitData[i++].i = (int16_t)(mpu.magnetomRaw[2]*10.0);
      }

      // if(!useRadio){
      //   transmitData[i++].i = (int16_t)(mpu.magnetom[0]*100); //7 startindex for magnetom
      //   transmitData[i++].i = (int16_t)(mpu.magnetom[1]*100);
      //   transmitData[i++].i = (int16_t)(mpu.magnetom[2]*100);
      // }
      interrupts();

      sensorFusionDuration = micros() - sensorFusionStartStamp;
    }

    if(printSensorDataOn){
      printf("Sensor Data: q1 = %f, q2 = %f, q3 = %f, q4 = %f, mx = %f, my = %f, mz = %f, ax = %f, ay = %f, az = %f, gx = %f, gy = %f, gz = %f \n", q[0], q[1], q[2], q[3], mpu.magnetom[0], mpu.magnetom[1], mpu.magnetom[2], mpu.accel[0], mpu.accel[1], mpu.accel[2], mpu.gyro[0], mpu.gyro[1], mpu.gyro[2]);
    }
    if(printMagAccelAngleOn){
      mpu.printMagAccelAngle();
    }
    if(printBatteryLevel){
      int currentBatLev = analogRead(A11);
      batteryLevel = batteryLevelFilterC * currentBatLev + (1.0 - batteryLevelFilterC) * batteryLevel;
      printf("Battery reading: %f \n", batteryLevel);
    }

    
    if(c == 'd')
    {
      printSensorDataOn = !printSensorDataOn;
    }

    else if(c == 'a'){
      printMagAccelAngleOn = !printMagAccelAngleOn;
    }

    else if(c == 'r')
    {
      // printf("bajsbajs!!\n");
      sendOwnDataToPc();
    }
    else if(c == 'b'){
      printBatteryLevel = !printBatteryLevel;
    }



    // if(c == 'c'){
    //   calibrateAcc = !calibrateAcc;
    // }

    // if(c == '0'){
    //   accelNrOfSamples = 0;
    //   for(int i = 0; i<3; ++i){
    //     accelAverage[i] = 0;
    //     accelMax[i] = 0;
    //     accelMin[i] = 0;
    //   }
    // }

    // if(calibrateAcc){
    //   sendAccCalToPc();
    // }

  }else if(role != baseStation){
    //Prepare some dummy data
    // noInterrupts();
    // int i = 0;
    // transmitData[i++].i = floatToQ14(1);
    // transmitData[i++].i = floatToQ14(2);
    // transmitData[i++].i = floatToQ14(3);
    // transmitData[i++].i = floatToQ14(4);
    // transmitData[i++].i = floatToQ14(5);
    // transmitData[i++].i = floatToQ14(6);
    // transmitData[i++].i = floatToQ14(7);
    // interrupts();
  }



  ///Three cases for handling radio. Either we don't use radio, or we are a node, or we are a base station
  if(!useRadio)
  {
    //Do nothing

    // printLoopTimingInfo();
  }
  else if(role != baseStation)
  {
    if(digitalRead(23) != lastFakePinState){
      lastFakePinState = digitalRead(23);
      if(!lastFakePinState){
        printf("closing pipe 1 and 2! \n");
        radio.closeReadingPipe(1);
        radio.closeReadingPipe(2);
      }else{
        printf("opening pipe 1 and 2! \n");
        openFirstPipe(role);
        openSecondPipe(role);
      }

      delay(20);
    }


    if(radio.failureDetected){
      printf("radio has failed somehow\n");
    }

    noInterrupts();
    if(radio.rxFifoFull()){
      Serial.print("RX FIFO FULL! Dummyreading. ");
      uint8_t data[32] = {0};
      while(radio.available()){
        printf("read. ");
        radio.read(data, radio.getDynamicPayloadSize());
      }
      printf("\n");
      // radio.printDetails();
    }
    interrupts();

    // uint8_t command1 = 0;
    // uint8_t command2 = 0;

    // // bool requestReceived = receivedPoll(&command1, &command2);
    // if(relayPendingTo){

    //   if(true || command1 == 'r'){
    //     // printf("lvl1 hop requested to edge node: %i \n", command2);
        
    //     // noInterrupts();
    //     // // printf("waiting time until relay got handled: %lu microsec \n", (micros() - irqStamp));
    //     // // printf("now trying to relay to %i \n", relayPendingTo-1);
    //     // pollEdgeAndSendToBase(relayPendingTo-1, receivedData);
    //     // relayPendingTo = 0;
    //     // interrupts();
    //     // pollEdgeAndSendToBase(command2, receivedData);
    //     // delayMicroseconds(250);


    //   }else if(command1 == 'e'){
    //     // printf("lvl2 Request received from node: %i \n", command2);
    //   }else if( command1 == 'n'){
    //     // respondToBlockingPoll(role, transmitData);
    //     // printf("Normal request received at %i", millis());
    //     // printf(". From node %i \n", command2);
    //   }

    //   // Serial.println("------------------------");
    // }   

    // printLoopTimingInfo();


  }else if (role == baseStation )
  {
    //init to false. These gets set to true if new data available
    // receivedDataUpdated = false;
    // receivedRelayDataUpdated = false;
    // updatedNodes[currentNode] = false;

    // -------------------------------
    // RELAYING PROCEDURES -----------____________________________/

    //Four cases. Only one of them should be entered. edge, normal, nonresponding or bridge. 
    //EDGE
    if(edgeActive[currentNode])
    {
      //This node is currently an edge. Should we go back to normal polling?
      if(millis() - normalPollFailedStamp[currentNode] > relayDuration){
        ///Poll the edge here directly. If fail, keep current bridge configuration. Only if it works we'll release the relay
        //This is because we don't want to go through the process of finding a working bridge again.
        // Serial.print("timeout poll for; "); Serial.println(currentNode);
        bool ackPackReceived = pollNode(currentNode, currentCommands[currentNode], receivedData);

        if(!ackPackReceived){// It failed. Keep current bridging. Do nothing, that is.
          normalPollFailedStamp[currentNode] = millis();//Besides updating the fail timer.
        }else{
          // printf("pollnode from node %i gave trigger id %i \n",currentNode, receivedData[8].c[0]);
          // receivedDataUpdated = true;
          saveReceivedData(receivedData);
          addToSendQueue(currentNode);
          edgeActive[currentNode] = false;
          bridgeActive[bridgeFor[currentNode]] = false;
        }

        
      }
    }
    //TRY TO REACTIVATE NONRESPONDING NODE
    else if(!isResponding[currentNode]){
      if(millis() - tryReestablishStamp[currentNode] > 5000){
        // tryReestablishStamp[currentNode] = millis();
        // printf("trying to reestablsih contact with node %i\n", currentNode);
        bool ackPackReceived = pollNode(currentNode, currentCommands[currentNode],  receivedData);

        if(ackPackReceived){
          saveReceivedData(receivedData);
          addToSendQueue(currentNode);
        }
      }
    }

    //NORMAL
    else if(!edgeActive[currentNode] && !bridgeActive[currentNode])
    {
      bool ackPackReceived = pollNode(currentNode, currentCommands[currentNode],  receivedData);

      if(!ackPackReceived){//FAILED. AAAAARRRGH! Let's try to relay
        normalPollFailedStamp[currentNode] = millis();//set relay timeout
        // printf("No response from %i. Will try to relay \n", currentNode);
        //Procedure for choosing a relay bridge. Intense shit!!
        setupRelay(currentNode, (currentNode+1)%nrOfNodes);
      }else{
        // printf("pollnode from node %i gave trigger id %i \n",currentNode, receivedData[8].c[0]);
        // receivedDataUpdated = true;
        saveReceivedData(receivedData);
        addToSendQueue(currentNode);
      }
    }

    //BRIDGE
    else if(bridgeActive[currentNode]){
      // printf("polling bridge\n");
      uint8_t relayResult = blockingPollBridge(5000, currentNode, currentCommands[currentNode], relayTarget[currentNode], currentCommands[relayTarget[currentNode]], receivedData, receivedRelayData);
      // printf("moving on \n");
      //Error codes:
      //3 = success, two sets of data. One from bridge, one from edge
      //2 = bridge told us he didn't reach edge
      //1 = bridge acked with his data. But no active response after that.
      //0 = no answer whatsoever.
      if(relayResult < 3){//The relay failed in some way, either base-bridge or bridge-edge. Let's try to find a new bridge.
        //Pretty counterintuituve stuff here. We don't deactivate the bridge before we search for a new one. This is so that it doesn't choose itself again.
        //But the edge is deactivated before, in case no bridge is found. If a new bridge is found, the edge will be reactivated inside setupRelay
        edgeActive[relayTarget[currentNode]] = false;
        setupRelay(relayTarget[currentNode], (currentNode+1)%nrOfNodes);
        //No matter if we found a new one or not, the old bridgeNode was bad and is now deactivated
        bridgeActive[currentNode] = false;
      }
      //Ok. Now more specific actions depending on error code
      if(relayResult == 3){//Both succeded!
        // receivedDataUpdated = true;
        // receivedRelayDataUpdated = true;
        // printf("success pollbridge from node %i gave bridge trigger id %i. ",currentNode, receivedData[8].c[0]);
        // printf("Edge %i gave trigger id %i \n",relayTarget[currentNode], receivedRelayData[8].c[0]);
        saveReceivedData(receivedData);
        saveReceivedData(receivedRelayData);
        addToSendQueue(currentNode);
        addToSendQueue(relayTarget[currentNode]);
      }else if(relayResult > 0){//Only base-bridge data succeded. This is true as long as ackpack was received. I.E. errorcodes above 0
        // receivedDataUpdated = true;

        // printf("Failed pollbridge from node %i gave bridge trigger id %i \n",currentNode, receivedData[8].c[0]);
        saveReceivedData(receivedData);
        addToSendQueue(currentNode);
      }

    }


    if(printRadioPerformance){
      printPerformance(50);
    }

    if(printSensorDataOn){
      printSensorData();
    }

    if(outputToProcessing){
      sendToProcessing();
    }

    if(outputMagCalibration){
      printMagCalibration();
    }

    //Handle serial commands
    if (c == '#')
    {
      Serial.println();
      Serial.println("entering razor command parsing");
      while(Serial.available() < 1){}
      uint8_t rCommand = Serial.read();
      if(rCommand == 's'){
        printRadioPerformance = false;
        printSensorDataOn = false;
        //Set output mode compatible with processing calibration sketch
        outputToProcessing = true;
        //Request raw magnetom sensor data from all active nodes.
        for(int i = 0; i < nrOfNodes; ++i){
          currentCommands[i][0] = SEND_MAGNETOM_RAW;
          commandReceived[i] = false;
        }

        while(Serial.available() < 2){}
        // Read ID
        uint8_t id[2];
        id[0] = Serial.read();
        id[1] = Serial.read();
        
        // Reply with synch message
        Serial.print("#SYNCH");
        Serial.write(id, 2);
        Serial.println();
      }
    }
    else if( c == 'c'){
      for(int i = 0; i < nrOfNodes; ++i){
        currentCommands[i][0] = SEND_MAGNETOM_RAW;
        commandReceived[i] = false;
      }
      outputMagCalibration = true;
    }
    else if(c == 'r')
    {
      handleSerialRequest();
      sendLatestDataToPc();
    }
    else if(c == 'p')
    {
      printRadioPerformance = true;
    }
    else if(c == 'x')
    {
      outputToProcessing = false;
      printRadioPerformance = false;
      printSensorDataOn = false;
    }
    else if(c == '0')
    {
      resetPerformanceData();
    }
    else if(c == 'd')
    {
      printSensorDataOn = !printSensorDataOn;
    }

// #ifdef USB_MIDI
//     for(int i = 0; i < nrOfNodes; ++i){
//       if(sendQueue[i]){
//         int accSum =  latestNodeValues[i][5].i
//                     + latestNodeValues[i][6].i
//                     + latestNodeValues[i][7].i;
//         if(accSum > 2500 && millis() - midiStamp > 100){
//           midiStamp = millis();
//           usbMIDI.sendNoteOn(60+i, 120, 1);
//           usbMIDI.sendNoteOn(60+i, 0, 1);
//         }
//       }
//     }
// #endif

//     const int triggIndex = 5;
//     if(triggerIds[currentNode] != latestNodeValues[currentNode][triggIndex].c[0]){
// #ifdef USB_MIDI
//       uint8_t vel = map(latestNodeValues[currentNode][triggIndex].c[1], 0,255,80,127);
//       usbMIDI.sendNoteOn(60+currentNode, vel, 1);
//       usbMIDI.sendNoteOn(60+currentNode, 0, 1);
// #endif
//       // printf("trigger from node %i, id %i, latestvalueId %i, at %i \n",currentNode, triggerIds[currentNode], latestNodeValues[currentNode][triggIndex].c[0], millis());
//     }
//     triggerIds[currentNode] = latestNodeValues[currentNode][triggIndex].c[0];


    // if(commandReceived[currentNode]){
    //   currentCommands[currentNode][0] = 0;
    // }
    currentNode++;
    currentNode %= nrOfNodes;

  }//BASESTATION
}//LOOP

bool setupRelay(int targetNode, int firstNodeToTry){
  for (int i = 0; i < nrOfNodes; ++i)
  {
    //Be aware. i is 0 first time
    int bridgeNode = (firstNodeToTry+i)%nrOfNodes;
    if(!bridgeActive[bridgeNode] && !edgeActive[bridgeNode] && bridgeNode != targetNode && isResponding[bridgeNode] && isResponding[targetNode]){
      relayTarget[bridgeNode] = targetNode;
      bridgeFor[targetNode] = bridgeNode;
      bridgeActive[bridgeNode] = true;
      edgeActive[targetNode] = true;
      // printf("bridging via %i to %i \n", bridgeNode, targetNode);
      return true;
    }
  }
  // Serial.println("Couldn't find a suitable bridge!");
  return false;
}

void saveReceivedData(binaryInt16* dataToSave){
  uint8_t node = dataToSave[0].c[0];
  memcpy(&latestNodeValues[node][0].c[0], &dataToSave[0].c[0], 32);
}
                                 // two byte width     header        quaternion     acctrigger      we write the nodeid separately
const int byteAmountExceptSensors = 2    *            (1    +        4     +        1)        -     1;
void sendLatestDataToPc(){
  // if(millis() - serialStamp > serialFrequency)
  // {
    // anyType sendData[16] = {0};//size is just chosen to be big enough. 16 elements is really not needed.
    // for (int i = 0; i < (nrOfBytes-2)/2; ++i)
    // {
    //   sendData[i].f = Q14ToFloat(data[i+1].i);
    // }
    // serialStamp = millis();
    // Serial.print(nrOfBytes);
    Serial.print("####");
    Serial.write(pendingNodesInSendQueue);
    for(int i = 0; i < nrOfNodes; ++i){  
      if(sendQueue[i]){

        Serial.write(i);//If the nodes have never made contact, the first byte in the latestNodeValues array have zero as nodeNr ( which is invalid). So let's write it manually.
        Serial.write( &latestNodeValues[i][0].c[1], byteAmountExceptSensors+(2*nrOfSendSensors*3));

        //Write some radio stats
        Serial.write((uint8_t*) &receivedPollPacketsOnPipe[i], 2);
        Serial.write((uint8_t*) &receivedRelayPacketsOnPipe[i], 2);
        Serial.write((uint8_t*) &pollFailsOnPipe[i], 2);
        Serial.write((uint8_t*) &relayFailsOnPipe[i], 2);
        Serial.write((uint8_t*) &retrieveDuration, 2);
        uint8_t c = 'n';
        if(bridgeActive[i]){
          c = 'r';
        }else if(edgeActive[i]){
          c = 'e';
        }
        Serial.write(c);
        Serial.write(']');

        removeFromSendQueue(i);
      }
    }
    Serial.write('>');
  // }
}

void handleSerialRequest(){
  rgbLed.setPixelColor(0, getColor(70, 255));
  rgbLed.show();
  uint8_t endCharacter = readChar();
  while(endCharacter != '>'){
    while(Serial.available() <= commandArraySize){
      //block
    }
    int node = endCharacter;
    Serial.readBytes((char*) currentCommands[node], commandArraySize);
    endCharacter = readChar();
  }
}

uint8_t readChar(){
  while(!Serial.available()){
    //Block until we get a character
  }
  return Serial.read();
}

void sendOwnDataToPc(){
  Serial.print("####");
  Serial.write(14);//Nr of bytes. 
  Serial.write((uint8_t*) transmitData, 14);
  Serial.write('>');
}

void sendToProcessing(){
  // Serial.println();
  // Serial.println("entering sendToProcessing");
  float data[10];
  for(int i = 0; i < nrOfNodes; ++i){  
    if(sendQueue[i]){
      // printf("Sending data for node: %i \n", i);
      data[3] = ((float)latestNodeValues[i][6].i/10.0);
      data[4] = ((float)latestNodeValues[i][7].i/10.0);
      data[5] = ((float)latestNodeValues[i][8].i/10.0);

      Serial.write((uint8_t*) data, 36);
      // printf("%f \t",data[3]);
      // printf("%f \t",data[4]);
      // printf("%f \t \n",data[5]);

      removeFromSendQueue(i);
    }
  }
}

float magMin[3] = {1000, 1000, 100}, magMax[3] = {-1000, -1000, -1000};
void printMagCalibration(){
  // Serial.println();
  // Serial.println("entering sendToProcessing");
  float data[3];
  for(int i = 0; i < nrOfNodes; ++i){  
    if(sendQueue[i]){
      // printf("Sending data for node: %i \n", i);
      data[0] = ((float)latestNodeValues[i][6].i/10.0);
      data[1] = ((float)latestNodeValues[i][7].i/10.0);
      data[2] = ((float)latestNodeValues[i][8].i/10.0);

      printf("min/max, bias: \t");
      for(int i = 0; i < 3; ++i){
        magMax[i] = max(magMax[i], data[i]);
        magMin[i] = min(magMin[i], data[i]);
        printf("%f/%f, %f \t",magMin[i], magMax[i], (magMax[i] + magMin[i])/2);
      }
      Serial.println();

      removeFromSendQueue(i);
    }
  }
}

void addToSendQueue(int nodeIndex){
  if(!sendQueue[nodeIndex]){
    pendingNodesInSendQueue++;
  }
  sendQueue[nodeIndex] = true;
}

void removeFromSendQueue(int nodeIndex){
  if(sendQueue[nodeIndex]){
    pendingNodesInSendQueue--;
  }
  sendQueue[nodeIndex] = false;
}

void setPrintFlag(){
  if(millis() - printStamp >= printFrequency){
    printStamp = millis();
    printDuringThisLap = true;
  }else{
    printDuringThisLap = false;
  }
}

void printLoopTimingInfo(){
  if(printDuringThisLap){
    printf("main loop duration is: %lu microsecs. Frequency is: %lu Hz. Time spent for fusion algorithm: %lu \n", mainLoopDuration, mainLoopFrequency, sensorFusionDuration);
  }
}

void printSensorData(){
  if(printDuringThisLap){
    for(int i = 0; i < nrOfNodes; ++i){
      if(sendQueue[i]){
        // int i = 1;
        printf("Sensor data from node %i: q1 = %f, q2 = %f, q3 = %f, q4 = %f, tId = %i a1 = %i, a2 = %i, a3 = %i b1 = %i, b2 = %i, b3 = %i \n",
            i, 
            Q14ToFloat(latestNodeValues[i][1].i), 
            Q14ToFloat(latestNodeValues[i][2].i), 
            Q14ToFloat(latestNodeValues[i][3].i), 
            Q14ToFloat(latestNodeValues[i][4].i),
            latestNodeValues[i][5].c[0],
            latestNodeValues[i][6].i, 
            latestNodeValues[i][7].i, 
            latestNodeValues[i][8].i,
            latestNodeValues[i][9].i, 
            latestNodeValues[i][10].i, 
            latestNodeValues[i][11].i
            );
      }
    }
  }
}

// void sendAccCalToPc(){

//   // Average gyro values
//   for (int i = 0; i < 3; i++)
//     accelAverage[i] += mpu.accelRaw[i];
//   accelNrOfSamples++;

//   Serial.print("accel x,y,z (min/max), average = ");
//   for (int i = 0; i < 3; i++) {
//     if (mpu.accelRaw[i] < accelMin[i]) accelMin[i] = mpu.accelRaw[i];
//     if (mpu.accelRaw[i] > accelMax[i]) accelMax[i] = mpu.accelRaw[i];
//     Serial.print(accelMin[i], 8);
//     Serial.print("/");
//     Serial.print(accelMax[i], 8);
//     Serial.print(", ");
//     Serial.print(accelAverage[i]/accelNrOfSamples, 8);
//     if (i < 2) Serial.print("  ");
//     else Serial.println();
//   }
// }

const float gravityFilterC = 0.000000001;
void updateGravityVector(){
  // unsigned long stamp = micros();
  gravityMagnitude = (1.0-gravityFilterC) * gravityMagnitude + gravityFilterC * vec_length(mpu.accel);
  float beforeRot[3] = {0,0,gravityMagnitude};
  quat_conj(q);
  quat_rotate(q, beforeRot, gravity);
  quat_conj(q);

  // printf("run time to calculate gravity is %i \n", micros()-stamp);

  // printf("gravity is %f, %f, %f \t linearAccel is %f, %f, %f \n", gravity[0], gravity[1], gravity[2], linearAccel[0], linearAccel[1], linearAccel[2]);
}


float linearAccel[3];
const int nrOfAccelSamples = 4; //Tested to give trigger results with 4. Be aware of trigger algorithm if changing this value
float linearAccelSamples[nrOfAccelSamples][3];
float linearAccelDistances[nrOfAccelSamples];
int linearAccelDistanceSum = 0;
float linearAccelLengths[nrOfAccelSamples];
int linearAccelSamplesIndex = 0;
bool accelTriggerActive = false;
unsigned long accelTriggerStamp;
const int shortestAllowedTriggerTime = 80;
float filteredLinearAccel[3];
float linearAccelFilterC = 0.8;
float filteredLinearAccelLength;
void updateAccel(){
  // unsigned long stamp = micros();

  for(int i = 0; i < 3; ++i){
    linearAccel[i] = mpu.accel[i] + gravity[i];
  }

  //do things that requires previous accelSample
  // float dist = 1000 * vec_distance(linearAccel, linearAccelSamples[linearAccelSamplesIndex]);
  // float angle = vec_angle(linearAccel, linearAccelSamples[(linearAccelSamplesIndex+1)%nrOfAccelSamples]);
  // if(TO_DEG(angle) > 25){
  //   printf("angle trigger\n");
  // }


  //As of now we rely on the linearAccelSamples array having nrOfAccelSamples elements. If we'd have more we'd need to change the loop below!
  float maxDistance = 0;
  float maxAngle = 0;
  for(int i = 0; i < nrOfAccelSamples; ++i){
    maxDistance = max(maxDistance, 1000 * vec_distance(linearAccel, linearAccelSamples[i]));
    maxAngle = max(maxAngle, vec_angle(linearAccel, linearAccelSamples[i]));
  }
  maxAngle = TO_DEG(maxAngle);
  // if(maxAngle > 25){
  //   printf("angle trigger\n");
  // }

  linearAccelSamplesIndex++;
  linearAccelSamplesIndex %= nrOfAccelSamples;

  //Do things that requires index to be incremented
  // linearAccelDistanceSum -= (int) linearAccelDistances[linearAccelSamplesIndex];
  // linearAccelDistanceSum += (int) dist;
  // linearAccelDistances[linearAccelSamplesIndex]  = dist;
  // linearAccelLengths[linearAccelSamplesIndex] = vec_length(linearAccel);
  float v_m = 100*vec_length(linearAccelSamples[linearAccelSamplesIndex]);

  // filteredLinearAccelLength = vec_length(linearAccelSamples[linearAccelSamplesIndex]);
  // printf("filteredLinearAccel %f \n", filteredLinearAccelLength);

  for(int i = 0; i < 3; ++i){
    linearAccelSamples[linearAccelSamplesIndex][i] = linearAccel[i];
    // filteredLinearAccel[i] = (1.0- linearAccelFilterC) *filteredLinearAccel[i] + linearAccelFilterC * linearAccel[i];
  }

  // printf("current linearAccelLength is: %f, ", linearAccelLengths[linearAccelSamplesIndex]);
  // printf("current linearAccelDistance is: %f", linearAccelDistances[linearAccelSamplesIndex]);
  // printf(", sum is: %i \n", linearAccelDistanceSum);
  // printf(" linearAccelVector: %f, %f, %f \n", linearAccel[0], linearAccel[1], linearAccel[2]);
  
  if(maxAngle > 25 && maxDistance > 800){// && linearAccelLengths[linearAccelSamplesIndex] > 0.9){
    if(!accelTriggerActive){
      accelTriggerActive = true;
      accelTriggerStamp = millis();
      accelTriggerId++;
      triggerVelocity = map(v_m, 140, 350, 0, 255);
      triggerVelocity = constrain(triggerVelocity, 0, 255);

      printf("trigger with maxDistance %f, magnitude %f, triggerVelocity %i and time interval %i \n",maxDistance, v_m, triggerVelocity, millis()-accelTriggerStamp);
#ifdef USB_MIDI
      usbMIDI.sendNoteOn(60+role, 120, 1);
      usbMIDI.sendNoteOn(60+role, 0, 1);
#endif

      // printf("trigger with distance %f, maxDistance %f and length %f\n", linearAccelDistances[linearAccelSamplesIndex], maxDistance, linearAccelLengths[linearAccelSamplesIndex]);
    }
  }else{
    if(maxDistance < 150 && accelTriggerActive && millis()-accelTriggerStamp > shortestAllowedTriggerTime){
      // printf("deactivating trigger %i \n", linearAccelDistanceSum);
      accelTriggerActive = false;
    }
  }

  // printf("run time to calculate linearaccel is %i \n", micros()-stamp);
  
}

void triggerMotor(){
  motorLevel = 255;
  motorOnStamp = millis();
  analogWrite(motorPin, motorLevel);
}

void updateMotor(){
  // if(motorLevel != 0){
  //   rgbLed.setPixelColor(0, getColor(motorLevel, 255));
  //   rgbLed.show();
  // }
  if(motorLevel != 0 && millis() - motorOnStamp > motorDuration){
    motorLevel = 0;
    analogWrite(motorPin, motorLevel);
  }
}

void initialiseEEPROM(){
  float zero = 0.0f;
  for(int i = 0; i<3; ++i){
    EEPROM.put(GYRO_BIAS_ADDRESS+i*sizeof(zero), zero);
  }
}

void writeGyroBiasToEEPROM(){
  int address = GYRO_BIAS_ADDRESS;
  uint8_t writeCounter = EEPROM.read(address);
  writeCounter++;
  printf("Writing gyroBias to memory. WriteCounter: %i\n", writeCounter);

  EEPROM.write(address++, writeCounter);
  for(int i = 0; i<3; ++i){
    EEPROM.put(address, mpu.gyroBias[i]);
    address += i*sizeof(mpu.gyroBias[i]);
  }
  EEPROM.write(address++, writeCounter);
}

bool fetchGyroBiases(){
  float gb[3];
  int address = GYRO_BIAS_ADDRESS;
  uint8_t writeCounter = EEPROM.read(address++);
  for(int i = 0; i<3; ++i){
    EEPROM.get(address, gb[i]);
    address += i*sizeof(mpu.gyroBias[i]);
  }
  if(writeCounter == EEPROM.read(address++)){
    vec_copy(gb, mpu.gyroBias);
    return true;
  }
  return false;
}