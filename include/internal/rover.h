#ifndef TRIK_V4L2_DSP_FB_INTERNAL_ROVER_H_
#define TRIK_V4L2_DSP_FB_INTERNAL_ROVER_H_

#include <stdbool.h>
#include <time.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


extern float PK;
extern float DK;
extern float IK;
extern int SPEED;
extern int inverseMotorCoeff;
extern int irrEnable;

typedef struct RoverConfigMotorMsp
{
  int m_mspI2CBusId;
  int m_mspI2CDeviceId;
  int m_mspI2CMotorCmd;
  int m_powerForwardMin;
  int m_powerForwardMax;
  int m_powerBackwardMin;
  int m_powerBackwardMax;
} RoverConfigMotorMsp;

typedef struct RoverConfigHeadlamp
{
  int m_mspI2CBusId;
  int m_mspI2CDeviceId;
  int m_mspI2CHeadlampCmd;
  int m_powerMin;
  int m_powerMax;
} RoverConfigHeadlamp;


typedef struct RoverConfigIRRangefinder
{
  int m_mspI2CBusId;
  int m_mspI2CDeviceId;
  int m_mspI2CMotorCmd;
  int m_distanceMin;
  int m_distanceMax;
} RoverConfigIRRangefinder;


typedef struct RoverConfig // what user wants to set
{
  RoverConfigMotorMsp m_motorMsp1;
  RoverConfigMotorMsp m_motorMsp2;
  RoverConfigHeadlamp m_headlamp;
  RoverConfigIRRangefinder m_rangefinder;

  int m_zeroX;
  int m_zeroY;
  int m_zeroMass;
} RoverConfig;

typedef struct RoverMotorMsp
{
  int m_i2cBusFd;
  int m_mspI2CDeviceId;
  int m_mspI2CMotorCmd;
  int m_powerForwardMin;
  int m_powerForwardMax;
  int m_powerBackwardMin;
  int m_powerBackwardMax;
} RoverMotorMsp;

typedef struct RoverHeadlamp
{
  int m_i2cBusFd;
  int m_mspI2CDeviceId;
  int m_mspI2CHeadlampCmd;
  int m_powerMin;
  int m_powerMax;
} RoverHeadlamp;

typedef struct RoverIRRangefinder
{
  int m_i2cBusFd;
  int m_mspI2CDeviceId;
  int m_mspI2CMotorCmd;
  int m_distanceMin;
  int m_distanceMax;
} RoverIRRangefinder;


typedef struct RoverControlChasis
{
  RoverMotorMsp* m_motorLeft1;
  RoverMotorMsp* m_motorRight1;
  RoverHeadlamp* m_headlamp;
  RoverIRRangefinder* m_rangefinder;

  int         m_lastX; // -100..100
  int         m_lastYaw;   // -100..100
  int         m_zeroX;
  int         m_zeroY;
  int         m_zeroMass;
} RoverControlChasis;


typedef struct RoverOutput
{
  bool       m_opened;

  RoverMotorMsp m_motorMsp1;
  RoverMotorMsp m_motorMsp2;
  RoverHeadlamp m_headlamp;
  RoverIRRangefinder m_rangefinder;
  RoverControlChasis m_ctrlChasis;

  enum State
  {
    StateManual,
    StatePreparing,
    StateSearching,
    StateTracking
  } m_state;

  struct timespec m_stateEntryTime;

} RoverOutput;


int roverOutputInit(bool _verbose);
int roverOutputFini();

int roverOutputOpen(RoverOutput* _rover, const RoverConfig* _config);
int roverOutputClose(RoverOutput* _rover);
int roverOutputStart(RoverOutput* _rover);
int roverOutputStop(RoverOutput* _rover);
int roverOutputControlAuto(RoverOutput* _rover, int _targetX, int _targetY, int _targetMass);
int roverOutputControlManual(RoverOutput* _rover, int _ctrlChasisLR, int _ctrlChasisFB, int _ctrlHand, int _ctrlArm);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // !TRIK_V4L2_DSP_FB_INTERNAL_ROVER_H_
