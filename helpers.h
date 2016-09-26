#ifndef HELPERS_H
#define HELPERS_H

union binaryFloat {
	float f;
	unsigned char c[4];
};

union binaryInt16 {
  int16_t i;
  uint8_t c[2];
};

union anyType {
  float f;
  uint32_t ul;
  int32_t l;
  uint16_t ui[2];
  int16_t i[2];
  unsigned char c[4];
};

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

inline int16_t floatToQ14( float value)
{
	return value * (0x01 << 14);
}

inline float Q14ToFloat( int16_t value)
{
	return ((float) value) / (0x01 << 14);
}

inline int numberOfSetBits(uint32_t i)
{
     // Java: use >>> instead of >>
     // C or C++: use uint32_t
     i = i - ((i >> 1) & 0x55555555);
     i = (i & 0x33333333) + ((i >> 2) & 0x33333333);
     return (((i + (i >> 4)) & 0x0F0F0F0F) * 0x01010101) >> 24;
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
#include <Adafruit_NeoPixel.h>
inline uint32_t getHue(uint8_t WheelPos) {
  if(WheelPos < 85) {
   return Adafruit_NeoPixel::Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return Adafruit_NeoPixel::Color(0, WheelPos * 3, 255 - WheelPos * 3);
  } else {
   WheelPos -= 170;
   return Adafruit_NeoPixel::Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  }
}

inline uint32_t getColor(uint8_t WheelPos, uint8_t brightness) {
  if(WheelPos < 85) {
   return Adafruit_NeoPixel::Color((255 - WheelPos * 3)*brightness/255, 0, (WheelPos * 3)*brightness/255);
  } else if(WheelPos < 170) {
   WheelPos -= 85;
   return Adafruit_NeoPixel::Color(0, (WheelPos * 3)*brightness/255, (255 - WheelPos * 3)*brightness/255);
  } else {
   WheelPos -= 170;
   return Adafruit_NeoPixel::Color((WheelPos * 3)*brightness/255, (255 - WheelPos * 3)*brightness/255, 0);
  }
}


//3d math
//quaternion is stored w, x, y, z

//creates a quaternion that first rotates q1 and then q2
inline void quat_mult(const float *q1, const float *q2, float *qProd)
{
    qProd[0] =
        (q1[0] * q2[0] - q1[1] * q2[1] - q1[2] * q2[2] - q1[3] * q2[3]);
    qProd[1] =
        (q1[0] * q2[1] + q1[1] * q2[0] + q1[2] * q2[3] - q1[3] * q2[2]);
    qProd[2] =
        (q1[0] * q2[2] - q1[1] * q2[3] + q1[2] * q2[0] + q1[3] * q2[1]);
    qProd[3] =
        (q1[0] * q2[3] + q1[1] * q2[2] - q1[2] * q2[1] + q1[3] * q2[0]);
}

inline void quat_norm(float* q)
{
    float normSF = 0;
    int i;
    for (i = 0; i < 4; i++) {
        normSF +=
            q[i] * q[i];
    }
    if (normSF > 0) {
        normSF = 1 / sqrt(normSF);
        for (i = 0; i < 4; i++) {
            q[i] = q[i] * normSF;
        }
    } else {
        q[0] = 1.0f;
        for (i = 1; i < 3; i++) {
            q[i] = 0;
        }
    }
}

inline void quat_conj(float* q){
  q[0] = -q[0];
}

inline void quat_rotate(const float* q, const float* vecIn, float* vecOut)
{
    float q_temp1[4], q_in[4];
    float in4[4], out4[4];


    in4[0] = 0;
    memcpy(&in4[1], vecIn, 3 * sizeof(float));
    memcpy(&q_in, q, 4 * sizeof(float));
    quat_mult(q_in, in4, q_temp1);
    quat_conj(q_in);
    quat_mult(q_temp1, q_in, out4);
    memcpy(vecOut, &out4[1], 3 * sizeof(float));
}

inline void quat_copy(const float* q, float* qOut){
  memcpy(qOut, &q[0], 4 * sizeof(float));
}

inline void quat_build(const float* axis, float angle, float* qOut){
  qOut[0] = cos(angle/2.0f);
  qOut[1] = axis[0] * sin(angle/2.0f);
  qOut[2] = axis[1] * sin(angle/2.0f);
  qOut[3] = axis[2] * sin(angle/2.0f);
}

inline float quat_angle(const float* q){

  //  Here's another formula taken from openFrameworks
  //  float sinhalfangle = sqrt( q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );
  //  result = 2.0 * atan2( sinhalfangle, q[0] );
  //==================================================//

  //Clamp value of w within -1 to 1
  float w = q[0];
  if (w > 1.0f) {
      w = 1.0;
  } else if (w < -1.0f) {
      w = -1.0;
  }
  
  
  float result = 2.0f * acos(w);
  if(result > PI){
    result -= 2.0f*PI;
  }
  return result;
}

inline void quat_axis(const float* q, float* vecOut){
  float sinhalfangle = sqrt( q[1] * q[1] + q[2] * q[2] + q[3] * q[3] );

  if (sinhalfangle) {
    vecOut[0] = q[1] / sinhalfangle;
    vecOut[1] = q[2] / sinhalfangle;
    vecOut[2] = q[3] / sinhalfangle;
  } else {
    vecOut[0] = 0.0;
    vecOut[1] = 0.0;
    vecOut[2] = 1.0;
  }
}

//yaw pitch roll. i.e. In the order they are applied to get the final rotation
inline void quat_eulerAngles(const float* q, float* e){
  e[0]   = atan2(2.0f * (q[1] * q[2] + q[0] * q[3]), q[0] * q[0] + q[1] * q[1] - q[2] * q[2] - q[3] * q[3]);
  e[1] = -asin(2.0f * (q[1] * q[3] - q[0] * q[2]));
  e[2]  = atan2(2.0f * (q[0] * q[1] + q[2] * q[3]), q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
}

inline void vec_scale(float out[3], const float v[3], float scale)
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v[c] * scale; 
  }
}

inline void vec_add(float out[3], const float v1[3], const float v2[3])
{
  for(int c = 0; c < 3; c++)
  {
    out[c] = v1[c] + v2[c];
  }
}

inline void vec_copy(const float* v, float* vOut){
  memcpy(vOut, &v[0], 3 * sizeof(float));
}

inline float vec_dot(const float* v1, const float* v2){
  return v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2];
}

inline void vec_norm(float* v){
  float length = (float) sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if( length > 0 ) {
    v[0] /= length;
    v[1] /= length;
    v[2] /= length;
    // return ofVec3f( x/length, y/length, z/length );
  } else {
    v[0] = 0;
    v[1] = 0;
    v[2] = 0;
  }
}

inline float vec_angle(const float* v1, const float* v2){
  float _vec1[3];
  float _vec2[3];
  vec_copy(v1, _vec1);
  vec_copy(v2, _vec2);
  vec_norm(_vec1);
  vec_norm(_vec2);
  return acos(vec_dot(_vec1, _vec2));

  // vec_norm(v1);
  // vec_norm(v2);
  // return acos(vec_dot(v1, v2));
}

inline float vec_length(const float* v){
  return (float)sqrt( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}

inline float vec_lengthSquared(const float* v){
  return (float)( v[0]*v[0] + v[1]*v[1] + v[2]*v[2] );
}

inline float vec_distance_squared(const float* v1, const float* v2){
  float vx = v1[0] - v2[0];
  float vy = v1[1] - v2[1];
  float vz = v1[2] - v2[2];
  return vx*vx + vy*vy + vz*vz;
}

inline float vec_distance(const float* v1, const float* v2){
  return (float) sqrt(vec_distance_squared(v1, v2));
}

#endif