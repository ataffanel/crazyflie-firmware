/******************************************************************************
 * The sequence commander loads trajectories thorugh the parameter framework,
 * (should be revised to use packets instead) allows the loading and evaluation
 * of trajectories in the flat output space gamma=(x,y,z,yaw) \in\mathbb{R]^4.
 *
 * ~~~ Supported reference trajectory types ~~~
 * (0) Points in R^4 \in\mathbb{R]^4 corresponding to the flat outputs
 * (1) Polynomial trajectories in \in\mathbb{R]^4
 * (2) Basic sinusoid functions in \in\mathbb{R]^4
 * (3) Bezier cureves functions in \in\mathbb{R]^4 
 * (4) Scheduled events
 *
 * Each trajectory is represented as a trajectory object containing the a data
 * field which is specific for each trajectory type (see M_sequenceCommander.c
 * at the bottom of the script for clarifications as to the structure).
 *
 * In order to generate the flat output and their derivatives for flatness
 * generation, we require the fourth derivative of the trajectory. While simple
 * in the case of polynomial splines and sinusoidal functions, special care
 * must be taken when handling points. In the current implementation the
 * fourth order system
 *
 *          a^4 * s^i
 *   G(s) = ---------
 *          (a + s)^4
 *
 * is discretized with a = 7.5 at 50 Hz and simulated with (0), the point
 * trajectory, to provide well defined derivative terms.
 *
 * ~~~ Changelog ~~~
 * Editor        | Date       | Description
 *-----------------------------------------------------------------------------
 * Marcus Greiff | 28-11-2016 | Initial commit with dummy example so that
 *               |            | may be generated without prioviding a
 *               |            | trajectory file.
 *-----------------------------------------------------------------------------
 * Marcus Greiff | 11-12-2016 | Updated with packets for synchronization,
 *               |            | trajectory data and direct data.
 *-----------------------------------------------------------------------------
 * Marcus Greiff | 30-01-2016 | Andd events and make compliant with quaternion
 *               |            | controller.
 *****************************************************************************/

#ifndef __SEQUENCECOMMANDER_M_H__
#define __SEQUENCECOMMANDER_M_H__

// Maximum sizes of the trjectories
#define NUM_FLAT_OUTPUTS 4
#define FLAT_OUTPUT_DIMENSIONS 5
#define MAX_TRAJECTORY_ENTRIES 20
#define MAX_DIMENSION 6

#include <stdbool.h>
#include <stdint.h>

#include "stabilizer_types.h"
#include "crtp.h"

/******************************************************************************
 * Trajecory for a single flat output.
 *****************************************************************************/
typedef struct trajectory_s
{
  bool circular;                                        // Set to true if the trajectory should restart upon terminating
  bool status;                                          // Determines if the trajectory is complete (being loaded or missing data)
  bool isset [MAX_TRAJECTORY_ENTRIES];                  // Used to check that a complete trajectory has been loaded
  float data [MAX_TRAJECTORY_ENTRIES][MAX_DIMENSION];   // trajectory data
  float time [MAX_TRAJECTORY_ENTRIES];                  // time of each trajectory component [s]
  uint8_t type [MAX_TRAJECTORY_ENTRIES];                // trajectory type
  float startTime;                                      // start time of the trajectory [s]
  uint32_t syncronizationTick;                          // time at which the object was last synchronized
  uint8_t numberOfEntries;                              // number of splines/points
  bool isActiveLP;                                      // Flag for determining if the LP filtering is active 
} trajectory_t;

// ############################ Packet definitions ############################

// Synchronization data packet struct
typedef struct {         // total size: 18
  uint8_t packetType;    // Type, used to distinguish between package types (= 0 here)
  uint8_t synchronize;   // Specifies if the packet should be used to clear or to synchronize the data
  uint8_t circular[4];   // If the trajectories should be circular i.e. starting over
  uint8_t number[4];     // The trajectory mode along a specific dimension
  uint16_t time [4];     // Time to start after receiving the synchronization package

} __attribute__((packed)) synchronizationPacket_t;

// Trajectory data packet struct
typedef struct {         // total size: 18
  uint8_t packetType;    // PacketType, used to distinguish between package types (= 1 here)
  uint16_t data [6];     // Trajectory data 
  uint16_t time;         // Trajectory time 
  uint8_t index;         // Contains index of current element in the complete trajectory 
  uint8_t dimension;     // Contains the dimension of the current trajectory 
  uint8_t number;        // Contains total number of elements in the complete trajectory 
  uint8_t type;          // Contains total type of the trajectory {0,1,2,3,4}
} __attribute__((packed)) trajectoryPacket_t;

// The event table of contents
#define MAXIMUM_NUMBER_OF_EVENTS 20
typedef struct eventEntry_s {
  uint8_t ID;
  uint8_t status;
  uint8_t index;
} eventEntry_t;

typedef struct eventTrajectory_s {
  eventEntry_t events [MAXIMUM_NUMBER_OF_EVENTS];
  uint8_t number;
} eventTrajectory_t;

// ################################ Constants #################################

#define REFERENCE_RATE RATE_50_HZ
#define LP_FILTER_LENGTH 5    // LP filter horizon
#define POLYNOMIAL_DEGREE 5   // The maximum polynomial degree (for assertion)
#define MAX_TRAJECTORY_TYPE 5 // The total number of trajectory types (for assertion)

// ~~~ Functions ~~~

// Update setpoint based on the current controller mode
void sequenceCommanderGetSetpoint(setpoint_t *setpoint,
                                  state_t *state,
                                  uint32_t currentTick);

// Initialization
void newCommanderInit();

// Decoding of packets
bool decodePacket(CRTPPacket *pk,
                  setpoint_t *setpoint,
                  uint32_t currentTick);

// Evaluate point trajectory object at the current time
void eval_point_path(setpoint_t *setpoint,
                     int index,
                     int dimension);

// Evaluate polynomial trajectory object at the current time
void eval_polynomial(setpoint_t *setpoint,
                     float startTime,
                     float currentTime,
                     int index,
                     int dimension);

// Evaluate sinusodial trajectory object at the current time
void eval_function(setpoint_t *setpoint,
                   float currentTime,
                   int index,
                   int dimension);

// Evaluate Bezier trajectory object at the current time
void eval_bezier(setpoint_t *setpoint,
                 float startTime,
                 float currentTime,
                 int index,
                 int dimension);

// Evaluates polynomial
float polyval(float coeff[],
              float tp,
              int order);

// Assert that the trajectory has been fully loaded
bool assertTrajectoryLoaded(int dimension);

// Clear trajectry object
void clearTrajectory(int dimension);

// Reset the LP filter
void resetFilter(setpoint_t *setpoint, int dimension);

#endif //__SEQUENCECOMMANDER_M_H__
