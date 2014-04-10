#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>

#include <poll.h>
#include <stdint.h>
#include <linux/input.h>
#include <fcntl.h>

#include "internal/rover.h"

static int min_target_mass = 2; //noise treshold

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
{
  struct i2c_smbus_ioctl_data args;

  args.read_write = read_write;
  args.command = command;
  args.size = size;
  args.data = data;

  return ioctl(file, I2C_SMBUS, &args);
}

static inline __s32 i2c_smbus_read_word_data(int file, __u8 command)
{
  union i2c_smbus_data data;

  if (i2c_smbus_access(file,I2C_SMBUS_READ,command, I2C_SMBUS_WORD_DATA,&data))
    return -1;
  else
    return 0x0FFFF & data.word;
}

static int do_roverOpenMotorMsp(RoverOutput* _rover,
                                RoverMotorMsp* _motor,
                                const RoverConfigMotorMsp* _config)
{
  int res;

  if (_rover == NULL || _motor == NULL || _config == NULL)
    return EINVAL;

  char busPath[100];
  snprintf(busPath, sizeof(busPath), "/dev/i2c-%d", _config->m_mspI2CBusId);
  _motor->m_i2cBusFd = open(busPath, O_RDWR);
  if (_motor->m_i2cBusFd < 0)
  {
    res = errno;
    fprintf(stderr, "open(%s) failed: %d\n", busPath, res);
    _motor->m_i2cBusFd = -1;
    return res;
  }

  _motor->m_mspI2CDeviceId   = _config->m_mspI2CDeviceId;
  _motor->m_mspI2CMotorCmd   = _config->m_mspI2CMotorCmd;
  _motor->m_powerForwardMin  = _config->m_powerForwardMin;
  _motor->m_powerForwardMax  = _config->m_powerForwardMax;
  _motor->m_powerBackwardMin = _config->m_powerBackwardMin;
  _motor->m_powerBackwardMax = _config->m_powerBackwardMax;

  return 0;
}

static int do_roverOpenHeadlamp(RoverOutput* _rover,
                                RoverHeadlamp* _headlamp,
                                const RoverConfigHeadlamp* _config)
{
  int res;

  if (_rover == NULL || _headlamp == NULL || _config == NULL)
    return EINVAL;

  char busPath[100];
  snprintf(busPath, sizeof(busPath), "/dev/i2c-%d", _config->m_mspI2CBusId);
  _headlamp->m_i2cBusFd = open(busPath, O_RDWR);
  if (_headlamp->m_i2cBusFd < 0)
  {
    res = errno;
    fprintf(stderr, "open(%s) failed: %d\n", busPath, res);
    _headlamp->m_i2cBusFd = -1;
    return res;
  }

  _headlamp->m_mspI2CDeviceId    = _config->m_mspI2CDeviceId;
  _headlamp->m_mspI2CHeadlampCmd = _config->m_mspI2CHeadlampCmd;
  _headlamp->m_powerMin  = _config->m_powerMin;
  _headlamp->m_powerMax  = _config->m_powerMax;

  return 0;
}

static int do_roverCloseMotorMsp(RoverOutput* _rover,
                                 RoverMotorMsp* _motor)
{
  int res;

  if (_rover == NULL || _motor == NULL)
    return EINVAL;

  if (close(_motor->m_i2cBusFd) != 0)
  {
    res = errno;
    fprintf(stderr, "close() failed: %d\n", res);
    return res;
  }
  _motor->m_i2cBusFd = -1;

  return 0;
}


static int do_roverCloseHeadlamp(RoverOutput* _rover,
                                 RoverHeadlamp* _headlamp)
{
  int res;

  if (_rover == NULL || _headlamp == NULL)
    return EINVAL;

  if (close(_headlamp->m_i2cBusFd) != 0)
  {
    res = errno;
    fprintf(stderr, "close() failed: %d\n", res);
    return res;
  }
  _headlamp->m_i2cBusFd = -1;

  return 0;
}

static int do_roverOpenRangefinder(RoverOutput* _rover,
                                   RoverIRRangefinder* _rangefinder,
                                   const RoverConfigIRRangefinder* _config)
{
  int res;

  if (_rover == NULL || _rangefinder == NULL || _config == NULL)
    return EINVAL;

  char busPath[100];
  snprintf(busPath, sizeof(busPath), "/dev/i2c-%d", _config->m_mspI2CBusId);
  _rangefinder->m_i2cBusFd = open(busPath, O_RDWR);
  if (_rangefinder->m_i2cBusFd < 0)
  {
    res = errno;
    fprintf(stderr, "open(%s) failed: %d\n", busPath, res);
    _rangefinder->m_i2cBusFd = -1;
    return res;
  }

  _rangefinder->m_mspI2CDeviceId = _config->m_mspI2CDeviceId;
  _rangefinder->m_mspI2CMotorCmd = _config->m_mspI2CMotorCmd;
  _rangefinder->m_distanceMin    = _config->m_distanceMin;
  _rangefinder->m_distanceMax    = _config->m_distanceMax;

  return 0;
}

static int do_roverCloseRangefinder(RoverOutput* _rover,
                                    RoverIRRangefinder* _rangefinder)
{
  int res;

  if (_rover == NULL || _rangefinder == NULL)
    return EINVAL;

  if (close(_rangefinder->m_i2cBusFd) != 0)
  {
    res = errno;
    fprintf(stderr, "close() failed: %d\n", res);
    return res;
  }
  _rangefinder->m_i2cBusFd = -1;

  return 0;
}


static int do_roverOpen(RoverOutput* _rover,
                        const RoverConfig* _config)
{
  int res;

  if (_rover == NULL || _config == NULL)
    return EINVAL;

  if ((res = do_roverOpenMotorMsp(_rover, &_rover->m_motorMsp1, &_config->m_motorMsp1)) != 0)
  {
    return res;
  }

  if ((res = do_roverOpenMotorMsp(_rover, &_rover->m_motorMsp2, &_config->m_motorMsp2)) != 0)
  {
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp1);
    return res;
  }

  if ((res = do_roverOpenHeadlamp(_rover, &_rover->m_headlamp, &_config->m_headlamp)) != 0)
  {
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp2);
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp1);
    return res;
  }

  if ((res = do_roverOpenRangefinder(_rover, &_rover->m_rangefinder, &_config->m_rangefinder)) != 0)
  {
    do_roverCloseHeadlamp(_rover, &_rover->m_headlamp);
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp2);
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp1);
    return res;
  }

  return 0;
}

static int do_roverClose(RoverOutput* _rover)
{
  if (_rover == NULL)
    return EINVAL;

  do_roverCloseHeadlamp(_rover, &_rover->m_headlamp);
  do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp2);
  do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp1);
  do_roverCloseRangefinder(_rover, &_rover->m_rangefinder);

  return 0;
}


static int do_roverMotorMspSetPower(RoverOutput* _rover,
                                    RoverMotorMsp* _motor,
                                    int _power)
{
  int res;

  if (_rover == NULL || _motor == NULL)
    return EINVAL;

  int pwm;

  if (_power == 0) // neutral
    pwm = 0;
  else if (_power < 0) // back
  {
    if (_power < -100)
      pwm = _motor->m_powerBackwardMax;
    else
      pwm = _motor->m_powerBackwardMin + ((_motor->m_powerBackwardMax-_motor->m_powerBackwardMin)*(-_power))/100;
  }
  else // forward
  {
    if (_power > 100)
      pwm = _motor->m_powerForwardMax;
    else
      pwm = _motor->m_powerForwardMin + ((_motor->m_powerForwardMax-_motor->m_powerForwardMin)*_power)/100;
  }

  int devId = _motor->m_mspI2CDeviceId;
  if (ioctl(_motor->m_i2cBusFd, I2C_SLAVE, devId) != 0)
  {
    res = errno;
    fprintf(stderr, "ioctl(%d, I2C_SLAVE, %d) failed: %d\n", _motor->m_i2cBusFd, devId, res);
    return res;
  }

  unsigned char cmd[2];
  cmd[0] = (_motor->m_mspI2CMotorCmd)&0xff;
  cmd[1] =  inverseMotorCoeff*pwm&0xff;

  if ((res = write(_motor->m_i2cBusFd, &cmd, sizeof(cmd))) != sizeof(cmd))
  {
    if (res >= 0)
      res = E2BIG;
    else
      res = errno;
    fprintf(stderr, "write(%d) failed: %d\n", _motor->m_i2cBusFd, res);
    return res;
  }

  return 0;
}


static int do_roverHeadlampSetPower(RoverOutput* _rover,
                                    RoverHeadlamp* _headlamp,
                                    int _power)
{
  int res;

  if (_rover == NULL || _headlamp == NULL)
    return EINVAL;

  int pwm;

  if (_power == 0) // neutral
    pwm = 0;
  else if (_power > 100)
    pwm = _headlamp->m_powerMax;
  else
    pwm = _headlamp->m_powerMin + ((_headlamp->m_powerMax-_headlamp->m_powerMin)*_power)/100;


  int devId = _headlamp->m_mspI2CDeviceId;
  if (ioctl(_headlamp->m_i2cBusFd, I2C_SLAVE, devId) != 0)
  {
    res = errno;
    fprintf(stderr, "ioctl(%d, I2C_SLAVE, %d) failed: %d\n", _headlamp->m_i2cBusFd, devId, res);
    return res;
  }

  unsigned char cmd[2];
  cmd[0] = (_headlamp->m_mspI2CHeadlampCmd)&0xff;
  cmd[1] =  pwm&0xff;

  if ((res = write(_headlamp->m_i2cBusFd, &cmd, sizeof(cmd))) != sizeof(cmd))
  {
    if (res >= 0)
      res = E2BIG;
    else
      res = errno;
    fprintf(stderr, "write(%d) failed: %d\n", _headlamp->m_i2cBusFd, res);
    return res;
  }

  return 0;
}

static int do_roverRangefinderGetValue(RoverOutput* _rover,
                                       RoverIRRangefinder* _rangefinder)
{
  int res;

  int devId = _rangefinder->m_mspI2CDeviceId;
  if (ioctl(_rangefinder->m_i2cBusFd, I2C_SLAVE, devId) != 0)
  {
    res = errno;
    fprintf(stderr, "ioctl(%d, I2C_SLAVE, %d) failed: %d\n", _rangefinder->m_i2cBusFd, devId, res);
    return res;
  }

  /* Using SMBus commands */
  res = i2c_smbus_read_word_data(_rangefinder->m_i2cBusFd, 0x20);
  if (res < 0) {
    res = errno;
    fprintf(stderr, "read(%d) failed: %d\n", _rangefinder->m_i2cBusFd, res);
    /* ERROR HANDLING: i2c transaction failed */
  } 

  return res;
}

static int do_roverCtrlChasisSetup(RoverOutput* _rover, const RoverConfig* _config)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  chasis->m_motorLeft1  = &_rover->m_motorMsp1;
  chasis->m_motorRight1 = &_rover->m_motorMsp2;
  chasis->m_headlamp    = &_rover->m_headlamp;
  chasis->m_rangefinder = &_rover->m_rangefinder;
  chasis->m_lastX   = 0;
  chasis->m_lastYaw     = 0;
  chasis->m_zeroX = _config->m_zeroX;
  chasis->m_zeroY = _config->m_zeroY;
  chasis->m_zeroMass = _config->m_zeroMass;

  return 0;
}


static int do_roverCtrlChasisStart(RoverOutput* _rover)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  chasis->m_lastX = 0;
  chasis->m_lastYaw = 0;
  
  return 0;
}


static int do_roverCtrlChasisManual(RoverOutput* _rover, int _ctrlChasisLR, int _ctrlChasisFB)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, (_ctrlChasisFB+_ctrlChasisLR));
  do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, (_ctrlChasisFB-_ctrlChasisLR));

  return 0;
}


static int do_roverCtrlChasisPreparing(RoverOutput* _rover)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, 0);
  do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, 0);
  do_roverHeadlampSetPower(_rover, chasis->m_headlamp, 100);

  return 0;
}


//l1
static int sign(int _v)
{
  return (_v < 0) ? -1 : ((_v > 0) ? 1 : 0);
}

/*
static int powerIntegral(int _power, int _lastPower, int _percent)
{
  if (sign(_power) == sign(_lastPower))
    _power += (_lastPower * _percent) / 100 + sign(_lastPower);

  return _power;
}
*/

static int powerProportional(int _val, int _min, int _zero, int _max)
{
  int adj = _val - _zero;
  if (adj > 0)
  {
    if (_val >= _max)
      return 100;
    else
      return (+100*(_val-_zero)) / (_max-_zero); // _max!=_zero, otherwise (_val>=_max) matches
  }
  else if (adj < 0)
  {
    if (_val <= _min)
      return -100;
    else
      return (-100*(_val-_zero)) / (_min-_zero); // _min!=_zero, otherwise (_val<=_min) matches
  }
  else
    return 0;
}


static int m_max(int a, int b)
{
  return a >= b ? a : b;
}


static int m_min(int a, int b)
{
  return a <= b ? a : b;
}


static int do_roverCtrlChasisSearching(RoverOutput* _rover)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  if(chasis->m_lastX > 0)
  {
    do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, 100); 
    do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, 30);
  }
  else if (chasis->m_lastX < 0)
  {
    do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, 30);
    do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, 100);
  } 
  else // this should never happens
  {
    do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, 100);
    do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, 100);
  }

  return 0;
}


static int saturate(int min, int val, int max)
{
  return (val <= max) ? ((val >= min) ? val : min) : max;
}


static int do_roverCtrlChasisTracking(RoverOutput* _rover, int _targetX, int _targetY, int _targetMass)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  float x;
  int lastX = chasis->m_lastX;
  x = powerProportional(_targetX, -100, chasis->m_zeroX, 100);

  if(abs(x) < 10 && abs(lastX) < 10) //just move straight
  {
    do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, 100);
    do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, 100);

  } 
  else
  {
    float P, I, D;

    P = x * PK;
    D = (x - lastX) * DK;
    I = (x + lastX) * IK;

    float yaw;
    yaw = P + I + D;

    chasis->m_lastX = x;

  #if 1
    int speedL = SPEED+yaw;  
    int speedR = SPEED-yaw;

    int maxSpeed = m_max(speedL, speedR);
    int minSpeed = m_min(speedL, speedR);

    if (maxSpeed > 100)
    {
      speedL = saturate(0, speedL - (maxSpeed - 100),100);
      speedR = saturate(0, speedR - (maxSpeed - 100),100);
    }

    if (minSpeed < -100)
    {
      speedL = saturate(0, speedL - (minSpeed + 100),100);
      speedR = saturate(0, speedR - (minSpeed + 100),100);
    }

  #endif

    if(yaw != chasis->m_lastYaw)
    {
      chasis->m_lastYaw = yaw;

  //    fprintf(stderr, "Chasis l : %d x r : %d\n", speedL, speedR);
      do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, speedL);
      do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, speedR);
    }
  }

  return 0;
}

int roverOutputInit(bool _verbose)
{
  (void)_verbose;

  return 0;
}

int roverOutputFini()
{
  return 0;
}

int roverOutputOpen(RoverOutput* _rover, const RoverConfig* _config)
{
  int res = 0;

  if (_rover == NULL)
    return EINVAL;
  if (_rover->m_opened)
    return EALREADY;

  if ((res = do_roverOpen(_rover, _config)) != 0)
    return res;

  do_roverCtrlChasisSetup(_rover, _config);

  _rover->m_opened = true;

  return 0;
}

int roverOutputClose(RoverOutput* _rover)
{
  if (_rover == NULL)
    return EINVAL;
  if (!_rover->m_opened)
    return EALREADY;

  do_roverClose(_rover);

  _rover->m_opened = false;

  return 0;
}

int roverOutputStart(RoverOutput* _rover)
{
  if (_rover == NULL)
    return EINVAL;
  if (!_rover->m_opened)
    return ENOTCONN;

  _rover->m_state = StateManual;
  _rover->m_stateEntryTime.tv_sec = 0;
  _rover->m_stateEntryTime.tv_nsec = 0;

  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp1, 0);
  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp2, 0);
  do_roverHeadlampSetPower(_rover, &_rover->m_headlamp, 0); //100?

  return 0;
}

int roverOutputStop(RoverOutput* _rover)
{
  if (_rover == NULL)
    return EINVAL;
  if (!_rover->m_opened)
    return ENOTCONN;

  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp1, 0);
  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp2, 0);
  do_roverHeadlampSetPower(_rover, &_rover->m_headlamp, 0);

  return 0;
}

int roverOutputControlManual(RoverOutput* _rover, int _ctrlChasisLR, int _ctrlChasisFB, int _ctrlHand, int _ctrlArm)
{
  if (_rover == NULL)
    return EINVAL;
  if (!_rover->m_opened)
    return ENOTCONN;

  if (_rover->m_state != StateManual)
  {
    fprintf(stderr, "*** MANUAL MODE ***\n");
    _rover->m_state = StateManual;
    _rover->m_stateEntryTime.tv_sec = 0;
  }

  do_roverCtrlChasisManual(_rover, _ctrlChasisLR, _ctrlChasisFB);

  return 0;
}

int roverOutputControlAuto(RoverOutput* _rover, int _targetX, int _targetY, int _targetMass)
{

  if (_rover == NULL)
    return EINVAL;
  if (!_rover->m_opened)
    return ENOTCONN;

  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  if (_rover->m_stateEntryTime.tv_sec == 0)
    clock_gettime(CLOCK_MONOTONIC, &_rover->m_stateEntryTime);

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  long long msPassed = (now.tv_sec - _rover->m_stateEntryTime.tv_sec) * 1000
                     + (now.tv_nsec - _rover->m_stateEntryTime.tv_nsec) / 1000000;

  switch (_rover->m_state)
  {
    case StateManual:
      do_roverCtrlChasisManual(_rover, 0, 0);

      fprintf(stderr, "*** LEFT MANUAL MODE, PREPARING ***\n");
      _rover->m_state = StatePreparing;
      _rover->m_stateEntryTime.tv_sec = 0;
      break;

    case StatePreparing:
      do_roverCtrlChasisPreparing(_rover);
      fprintf(stderr, "*** PREPARED, SEARCHING ***\n");
      _rover->m_state = StateSearching;
      _rover->m_stateEntryTime.tv_sec = 0;
      break;

    case StateSearching:
      if (_targetMass > min_target_mass)
      {
        fprintf(stderr, "*** FOUND TARGET ***\n");
        chasis->m_lastX = 0;
        _rover->m_state = StateTracking;
        _rover->m_stateEntryTime.tv_sec = 0;
        break;
      }
      do_roverCtrlChasisSearching(_rover);
      break;

    case StateTracking:
      if (_targetMass <= min_target_mass)
      {
        fprintf(stderr, "*** LOST TARGET ***\n");
        _rover->m_state = StateSearching;
        _rover->m_stateEntryTime.tv_sec = 0;
        break;
      }
      do_roverCtrlChasisTracking(_rover, _targetX, _targetY, _targetMass);
      break;
  }

  return 0;
}
