/**
 * Authored by Marcus Greiff, November 2016.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * ============================================================================
 */
 
// Note that FreeRTOS must be included to use the timers, and that the
// script fails if they are included in the wrong order.
#include "commander_sequence.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "timers.h"
#include "queue.h"
#include "param.h"
#include "log.h"
#include "math.h"
#include "num.h"
#include "math.h"


// Defines and stores the flat output trajectory
trajectory_t trajObj[NUM_FLAT_OUTPUTS];

// Structure for preserving the LP filter history
static struct{
  float Y[FLAT_OUTPUT_DIMENSIONS][LP_FILTER_LENGTH];
  float U[FLAT_OUTPUT_DIMENSIONS][LP_FILTER_LENGTH];
  uint32_t ptr;
} filterHistory[NUM_FLAT_OUTPUTS];


// LP coefficients for derivative determination at 50 Hz
const float LP_denominator[LP_FILTER_LENGTH] = {
   1.0f                 , -3.441860465116279f  , 4.442401297998918f   , -2.548354232960619f  , 0.548192480346180f      // Denominator
};
const float LP_numerator[FLAT_OUTPUT_DIMENSIONS][LP_FILTER_LENGTH]   = {
  {0.000023692516762456f, 0.000094770067049822f, 0.000142155100574734f, 0.000094770067049822f, 0.000023692516762456f}, // Position
  {   0.002369251676246f,    0.004738503352491f,                  0.0f,  - 0.004738503352491f,   -0.002369251676246f}, // Velocity
  {   0.236925167624556f,                  0.0f,   -0.473850335249112f,                  0.0f,    0.236925167624556f}, // Acceleration
  {  23.692516762455604f,  -47.385033524911208f,                  0.0f,   47.385033524911208f,  -23.692516762455604f}, // Jerk
  {    2369.25167624556f,    -9477.00670498224f,    14215.51005747336f,    -9477.00670498224f,     2369.25167624556f}  // Snap
};

// Log variables for debugging (to be removed)
static uint32_t testCounter[6] = {0};
static uint8_t trajectoryState[4] = {0};
static float testVal[6] = {0};

// ~~~ The CRTP packet functions, structs and queques ~~~
#define TRAJECTORY_QUEUE_LENGTH (32)
static xQueueHandle packetQueue;

static inline bool sequenceCommanderHasPacket(CRTPPacket *pk) {
  return (pdTRUE == xQueueReceive(packetQueue, pk, 0));
}

static void sequenceCommanderCRTPCallback(CRTPPacket* pk) {
  // Queues the incoming packet
  xQueueSend(packetQueue, pk, 0);
}


static bool isInit = false;
void newCommanderInit() {
  if (isInit) {
    return;
  }
  packetQueue = xQueueCreate(TRAJECTORY_QUEUE_LENGTH, sizeof(CRTPPacket));
  xQueueReset(packetQueue);
  crtpRegisterPortCB(CRTP_PORT_TRAJECTORY, sequenceCommanderCRTPCallback);
  
  isInit = true;
}

// ~~~ Commander functionality ~~~
void sequenceCommanderGetSetpoint(setpoint_t *setpoint,
                                  state_t *state,
                                  uint32_t currentTick)
{
  // ~~~ Decode and read CRTP packages based on the entered data type ~~~
  CRTPPacket packet;
  bool continueReading = true;
  while (sequenceCommanderHasPacket(&packet) == true && continueReading){
    // Load data into trajectory object and determine if the queue should be
    // or cleared based on the information in the packet
    continueReading = decodePacket(&packet, setpoint, currentTick);
    if (continueReading != true){
      // Clear queue and exit while loop
      xQueueReset(packetQueue);
    }
  }

  trajectoryState[0] = (uint8_t)trajObj[0].status;
  trajectoryState[1] = (uint8_t)trajObj[1].status;
  trajectoryState[2] = (uint8_t)trajObj[0].status;
  trajectoryState[3] = (uint8_t)trajObj[1].status;
  
  // ~~~ Update references based on data ~~~
  if (trajObj[0].status == true &&
      trajObj[1].status == true &&
      trajObj[2].status == true &&
      trajObj[3].status == true)
    {
    if (RATE_DO_EXECUTE(REFERENCE_RATE, currentTick)) {
      // Iterate over the flat outputs separately
      for (int dimension = 0; dimension < NUM_FLAT_OUTPUTS; dimension++){
        // Act if the trajectory is fully loaded and and has some entries
        testCounter[5] = (uint32_t)trajObj[dimension].numberOfEntries;
        
        if (trajObj[dimension].numberOfEntries > 0){
          // ~~~ Reads the cache and loads the trajectory if applicable ~~~
          // All trajectories are read fomr time 0 (at synchronization) and with a time lag
          uint32_t syncTick = trajObj[dimension].syncronizationTick;
          float startTime = trajObj[dimension].startTime;
          float currentTime = (1.0f / RATE_MAIN_LOOP) * (float)(currentTick - syncTick) + startTime;
          
          // Compute current index in data field
          int index = -1;
          float totalTime = trajObj[dimension].startTime;
          for (int ii = 0; ii < trajObj[dimension].numberOfEntries; ii++){
            if (totalTime < currentTime){
              totalTime += trajObj[dimension].time[ii];
              index++;
            }
          }
    
          testCounter[dimension] = index;
          testCounter[4]++;
          
          if (index > -1){
            // Time and index of the active trajectory was used
            float startTime = totalTime - trajObj[dimension].time[index];

            
            /*if (dimension == 0){
              testVal[0] = trajObj[0].data[index][0];
              testVal[1] = trajObj[0].data[index][1];
              testVal[2] = trajObj[0].data[index][2];
              testVal[3] = trajObj[0].data[index][3];
              testVal[4] = trajObj[0].data[index][4];
              testVal[5] = trajObj[0].data[index][5];
            }*/

            if (trajObj[dimension].type[index] == 0) {
              // LP filter point path
              eval_point_path(setpoint, index, dimension);
            } else if (trajObj[dimension].type[index] == 1) {
              // Polynomial spline evaluation
              eval_polynomial(setpoint, startTime, currentTime, index, dimension);
            } else if (trajObj[dimension].type[index] == 2) {
              // Function evaluation
              eval_function(setpoint, currentTime, index, dimension);
            } else if (trajObj[dimension].type[index] == 3) {
              // Bezier curve evaluation
              eval_bezier(setpoint, startTime, currentTime, index, dimension);
            } else {
              // A mode other than {0,1,2,3} should not be possible
              configASSERT(false);
            }
          }
        }
      }
    }
  }
  
  // OBS! Two quick hacks are in place in order to the the controller settings
  // before the event TOC has been implemented, to simplify things for FOSDEM
  // We (1) initiate the controller when the desired elevation is above a limit
  // and the controller mode is hard coded - this could be changed to parameters
  // if need be.

  // TODO: Specify modes by means of event TOC/parameters
  setpoint->xmode = 0x4; // position control
  setpoint->ymode = 0x4; // position control
  setpoint->zmode = 0x4; // position control

  testVal[0] = setpoint->gamma[0][0];
  testVal[1] = setpoint->gamma[0][1];
  testVal[2] = setpoint->gamma[0][2];
  testVal[3] = setpoint->gamma[0][3];
              
  // fill setpoint structure
  for (int n = 0; n < 5; n++) {
    setpoint->x[n] = setpoint->gamma[0][n];
    setpoint->y[n] = setpoint->gamma[1][n];
    setpoint->z[n] = setpoint->gamma[2][n];
    setpoint->yaw[n] = setpoint->gamma[3][n];
  }
}

void eval_point_path(setpoint_t *setpoint, int index, int dim)
{
  /****************************************************************************
   * Simulates a fourth order system to find feasible derivative terms when 
   * using the linear path setpoints. Note that each derivative term is
   * simulated with a separate difference equation, but that the characteristic
   * polynomial of the system is invariant across the derivative terms on
   * on account of Tustin's approximation.
   ***************************************************************************/
  float Ycontrib[FLAT_OUTPUT_DIMENSIONS] = {0};
  float Ucontrib[FLAT_OUTPUT_DIMENSIONS] = {0};
  uint32_t p = filterHistory[dim].ptr;
  for (int n = 0; n < FLAT_OUTPUT_DIMENSIONS; n++){
    filterHistory[dim].U[n][p % FLAT_OUTPUT_DIMENSIONS] = trajObj[dim].data[index][0];
    for (int ii = 1; ii < FLAT_OUTPUT_DIMENSIONS; ii++){
      Ycontrib[n] -= LP_denominator[ii] * filterHistory[dim].Y[n][(ii + p) % FLAT_OUTPUT_DIMENSIONS];
    }
    for (int ii = 0; ii < FLAT_OUTPUT_DIMENSIONS; ii++){
      Ucontrib[n] += LP_numerator[n][ii] * filterHistory[dim].U[n][(ii + p) % FLAT_OUTPUT_DIMENSIONS];
    }
    filterHistory[dim].Y[n][p % FLAT_OUTPUT_DIMENSIONS] = Ycontrib[n] + Ucontrib[n];
    setpoint->gamma[dim][n] = Ycontrib[n] + Ucontrib[n];
  }
  filterHistory[dim].ptr--;
}

void eval_function(setpoint_t *setpoint, float t, int index, int dimension)
{
  /****************************************************************************
   * Evaluates a sinusoid function and it's derivative terms.
   ***************************************************************************/
  float A = trajObj[dimension].data[index][0];
  float B = trajObj[dimension].data[index][1];
  float w = trajObj[dimension].data[index][2];
  float p = trajObj[dimension].data[index][3];
  float vSin = sinf(w * t + p);
  float vCos = cosf(w * t + p);
  float w2 = w * w;
  float w3 = w2 * w;
  float w4 = w3 * w;
  setpoint->gamma[dimension][0] = A * vSin + B;
  setpoint->gamma[dimension][1] = A * w * vCos;
  setpoint->gamma[dimension][2] = -A * w2 * vSin;
  setpoint->gamma[dimension][3] = -A * w3 * vCos;
  setpoint->gamma[dimension][4] = A * w4 * vSin;
}

void eval_polynomial(setpoint_t *setpoint, float startTime, float currentTime, int index, int dimension)
{
  /****************************************************************************
   * Evaluates a polynomial function and it's derivative terms.
   ***************************************************************************/
  float t0 = 1.0f;
  float t1 = currentTime - startTime;
  float t2 = t1 * t1;
  float t3 = t2 * t1;
  float t4 = t3 * t1;
  float t5 = t4 * t1;
  float t[6] = {t0, t1, t2, t3, t4, t5};
  float c[6] = {trajObj[dimension].data[index][0], trajObj[dimension].data[index][1],
                trajObj[dimension].data[index][2], trajObj[dimension].data[index][3],
                trajObj[dimension].data[index][4], trajObj[dimension].data[index][5]};

  int maxorder = 5;
  // Iterates over the flat output derivatives
  for (int order = 0; order < maxorder; order++){
    setpoint->gamma[dimension][order] = 0.0f;
    // Write values
    for (int n = 0; n < (maxorder - order); n++){
      setpoint->gamma[dimension][order] += t[n] * c[n+order];
    }
    // Derive polynomial coefficients
    int count = 0;
    for (int ii = order; ii < maxorder; ii++){
        c[ii] *= count;
        count++;
    }
  }
}

void eval_bezier(setpoint_t *setpoint, float startTime, float currentTime, int index, int dimension){
  /****************************************************************************
   * Evaluates a bezier curve and it's derivative terms.
   ***************************************************************************/
  // TODO: Write out equations, currently does nothing.
  float t = (currentTime - startTime) / trajObj[dimension].time[index];
  float tp2 = t * t;
  float tp3 = t * tp2;
  float tm = 1.0f - t;
  float tmp2 = tm * tm;
  float tmp3 = tm * tmp2;
  float p0 = trajObj[dimension].data[index][0];
  float p1 = trajObj[dimension].data[index][1];
  float p2 = trajObj[dimension].data[index][2];
  float p3 = trajObj[dimension].data[index][3];
  setpoint->gamma[dimension][0] = tmp3 * p0 + 3.0f * tmp2 * t * p1 + 3.0f * tm * tp2 * p2 + tp3 * p3;
  setpoint->gamma[dimension][1] = 3.0f * tmp2 * (p1 - p0) + 6.0f * tm * t * ( p2 - p1 )  + 3.0f * tp2 * ( p3 - p2 );
  setpoint->gamma[dimension][2] = 6.0f * tm * (p2 - 2.0f * p1 + p0) + 6.0f * t * (p3 - 2.0f * p2 + p1);
  setpoint->gamma[dimension][3] = 6.0f * ( p3 - p0 ) + 18.0f * ( p1 - p2 );
  setpoint->gamma[dimension][4] = 0.0f;
}

bool decodePacket(CRTPPacket *pk, setpoint_t *setpoint, uint32_t currentTick){
  /****************************************************************************
   * Check how to cast data based on the first byte of data and fill the
   * trajectory structure based on the data contained in the CRTPPacket.
   *
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   *
   * packetType=0 - Synchronization (synchronization, clearing and settings of the trajectory)
   * 
   * |packetType|clear|circular0|..|circular3|number0|..|number3|time0|..|time3|
   * |int8      |int8 |int8     |..|int8     |int8   |..|int8   |int16|..|int16|
   *
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   *
   * packetType=1 - Trajectory data (part of a greater pre-loaded trajectory)
   *
   * |packetType|data0|..|data5|time |index|dimension|number|type|
   * |int8      |int16|..|int16|int16|int8 |int8     |int8  |int8|
   *
   * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
   *
   * packetType=2 - Event data (part of a greater pre-loaded trajectory)
   *
   * |packetType|data0|..|data5|time |index|dimension|number|type|
   * |int8      |int16|..|int16|int16|int8 |int8     |int8  |int8|
   *
   ***************************************************************************/
  
  if (pk->data[0] == 0){
    // ~~~ Synchronization packet ~~~
    //
    // This packet is used to clear a trajectory object, set basic settings
    // and can also be used to synchronize the trajectory. By default, the
    // tick starts at 0 and time at 0 when initializing the quadcopter. By
    // setting a synchronization tick and specifying a starttime allows for
    // synchronization while executing a trajectory.
    //
    // When clearing the trajectory, the trajectory status is set to false
    // This can only be set to true (allowing setpoint evaluation) by
    // providing a starttime and a synchronization tick, at which point
    // a check is made to see if all trajectory entries have been set. If so,
    // the reference trajectory is followed.

    synchronizationPacket_t *packet = (synchronizationPacket_t*)(pk->data); 
    
    // Fill the trajectory object
    for (int dim = 0; dim < NUM_FLAT_OUTPUTS; dim++){
      if (packet->synchronize == 1){
        // Fill trajectory settings and asserts that the loading is complete
        trajObj[dim].syncronizationTick = currentTick;
        trajObj[dim].status = assertTrajectoryLoaded(dim);
      } else {
        // Clear the trajectory object
        trajObj[dim].status = false;
        trajObj[dim].circular = (packet->circular[dim] == 1);
        trajObj[dim].numberOfEntries = packet->number[dim];
        trajObj[dim].startTime = half2single(packet->time[dim]);
        clearTrajectory(dim);
      }
    }
    // Clears queue and proceeds without evaluating any trajectory objects
    return true;  

  } else if (pk->data[0] == 1) {
    // ~~~ Trajectory data packet ~~~
    trajectoryPacket_t *packet = (trajectoryPacket_t*)(pk->data);
    
    uint8_t index = packet->index;
    uint8_t number = packet->number;
    uint8_t dimension = packet->dimension;
    uint8_t type = packet->type;
    float time = half2single(packet->time);

    // Sanity check for inputs, loading wonky data here would be very bad..!
    configASSERT(index <= number);              // The data is corrupted if the index > number
    configASSERT(dimension < MAX_DIMENSION);    // A total of four possible flat dimensions
    configASSERT(type < MAX_TRAJECTORY_TYPE);   // A total of five possible types
    configASSERT(time >= 0.0f);                 // No negative times allowed

    // Halt setpoint generation until all trajectories have been loaded
    trajObj[dimension].status = false;
    
    // Load data
    for (int ii = 0; ii < FLAT_OUTPUT_DIMENSIONS; ii++){
      trajObj[dimension].data[index][ii] = (float)half2single(packet->data[ii]);
    }
    trajObj[dimension].time[index] = time;
    trajObj[dimension].type[index] = type;
    trajObj[dimension].isset[index]  = true;
    
    // Takes the next element in the queue
    return true;
  } else {
    // Unsupported data packet, type > 1 is not defined.
    configASSERT(false); 
    return true;
  }
}

bool assertTrajectoryLoaded(int dimension){
  /****************************************************************************
   * Iterates over the elements in a trajectory along a flat dimension and
   * checks that all elements at indices up to *.numberOfEntries have been set
   ***************************************************************************/
  for (int index = 0; index < trajObj[dimension].numberOfEntries; index++){
    if (trajObj[dimension].isset[index] == false){
      return false;
    }
  }
  return true;
}

void clearTrajectory(int dimension){
  /****************************************************************************
   * Sets the isset flag to false for all elementss in all trajectories
   ***************************************************************************/
  for (int index = 0; index < MAX_TRAJECTORY_ENTRIES; index++){
    trajObj[dimension].isset[index] = false;
  }
}

void resetFilter(setpoint_t *setpoint, int dimension){
  /****************************************************************************
   * Resets the LP filter structure in the specified dimension, by enforcing
   * the current position, and the setting all derivative terms to zero
   ***************************************************************************/
}

LOG_GROUP_START(trajState)
LOG_ADD(LOG_UINT8, s1, &trajectoryState[0])
LOG_ADD(LOG_UINT8, s2, &trajectoryState[1])
LOG_ADD(LOG_UINT8, s3, &trajectoryState[2])
LOG_ADD(LOG_UINT8, s4, &trajectoryState[3])
LOG_GROUP_STOP(trajState)

LOG_GROUP_START(trajValues)
LOG_ADD(LOG_FLOAT, v1, &testVal[0])
LOG_ADD(LOG_FLOAT, v2, &testVal[1])
LOG_ADD(LOG_FLOAT, v3, &testVal[2])
LOG_ADD(LOG_FLOAT, v4, &testVal[3])
LOG_ADD(LOG_FLOAT, v5, &testVal[4])
LOG_ADD(LOG_FLOAT, v6, &testVal[5])
LOG_GROUP_STOP(trajValues)

LOG_GROUP_START(trajCounter)
LOG_ADD(LOG_UINT32, c1, &testCounter[0])
LOG_ADD(LOG_UINT32, c2, &testCounter[1])
LOG_ADD(LOG_UINT32, c3, &testCounter[2])
LOG_ADD(LOG_UINT32, c4, &testCounter[3])
LOG_ADD(LOG_UINT32, c5, &testCounter[4])
LOG_ADD(LOG_UINT32, c6, &testCounter[5])
LOG_GROUP_STOP(trajCounter)
