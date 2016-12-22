#ifndef __KALMAN_H__
#define __KALMAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdint.h>
#include <string.h>

typedef struct Kalman
{
  /* Critical Section */
  float q;    // process noise
  float r;    // measurement noise
  /* Critical Section */
  
  float e;    // estimation
  float d;    // difference
  float k;    // kalman gain
  float p;    // fused variance
}Kalman;

struct Kalman* KalmanCreate();
void KalmanReset(struct Kalman* kalman);
void KalmanSetE(struct Kalman* kalman, float e);
void KalmanSetD(struct Kalman* kalman, float d);
void KalmanSetQ(struct Kalman* kalman, float q);
void KalmanSetR(struct Kalman* kalman, float r);
float KalmanFilter(struct Kalman* kalman, float x);
void KalmanDestroy(struct Kalman* kalman);

#ifdef __cplusplus
}
#endif

#endif

