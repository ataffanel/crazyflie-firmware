
#include <math.h>

#include "stabilizer.h"
#include "stabilizer_types.h"

#include "sensfusion6.h"
#include "position_estimator.h"

#define ATTITUDE_UPDATE_RATE RATE_250_HZ
#define ATTITUDE_UPDATE_DT 1.0/ATTITUDE_UPDATE_RATE

#define POS_UPDATE_RATE RATE_100_HZ
#define POS_UPDATE_DT 1.0/POS_UPDATE_RATE

void stateEstimatorInit(void)
{
  sensfusion6Init();
}

bool stateEstimatorTest(void)
{
  bool pass = true;

  pass &= sensfusion6Test();

  return pass;
}

#define SIGN(X) ((X>=0)?1:-1)
#define M_PI 3.14159265358979323846

void stateEstimator(state_t *state, const sensorData_t *sensorData, const uint32_t tick)
{
  if (RATE_DO_EXECUTE(ATTITUDE_UPDATE_RATE, tick)) {
    sensfusion6UpdateQ(sensorData->gyro.x, sensorData->gyro.y, sensorData->gyro.z,
                       sensorData->acc.x, sensorData->acc.y, sensorData->acc.z,
                       ATTITUDE_UPDATE_DT);
    sensfusion6GetEulerRPY(&state->attitude.roll, &state->attitude.pitch, &state->attitude.yaw);

    state->acc.z = sensfusion6GetAccZWithoutGravity(sensorData->acc.x,
                                                    sensorData->acc.y,
                                                    sensorData->acc.z);

    positionUpdateVelocity(state->acc.z, ATTITUDE_UPDATE_DT);

    // Simple naive estimation of the speed based on the current attitude
    float meanThrust = 0;
    for (int i=0; i<4; i++) {
      meanThrust += sensorData->thrusts[i];
    }
    meanThrust /= 4;

    if (meanThrust == 0) {
      state->acc.x = 0;
      state->velocity.x = 0;
    } else {
      state->acc.x = - sinf(state->attitude.pitch*M_PI/180.0) * meanThrust - 0.01*SIGN(state->velocity.x)*state->velocity.x*state->velocity.x;
      state->velocity.x += state->acc.x *  ATTITUDE_UPDATE_DT;
    }
  }

  if (RATE_DO_EXECUTE(POS_UPDATE_RATE, tick)) {
    // If position sensor data is preset, pass it throught
    // FIXME: The position sensor shall be used as an input of the estimator
    if (sensorData->position.timestamp) {
      state->position = sensorData->position;
    } else {
      positionEstimate(state, sensorData->baro.asl, POS_UPDATE_DT);
    }
  }
}
