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

#include "internal/rover.h"
#include "internal/ce.h"

#include <stdint.h>
#include <linux/input.h>
#include <fcntl.h>

bool m_chw=true;
bool going_home = false;
static int rotate_direction=1;
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

  _motor->m_mspI2CDeviceId = _config->m_mspI2CDeviceId;
  _motor->m_mspI2CMotorCmd = _config->m_mspI2CMotorCmd;
  _motor->m_powerMin       = _config->m_powerMin;
  _motor->m_powerMax       = _config->m_powerMax;

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

static int do_roverOpenMotor(RoverOutput* _rover,
                             RoverMotor* _motor,
                             const RoverConfigMotor* _config)
{
  int res;

  if (_rover == NULL || _motor == NULL || _config == NULL || _config->m_path == NULL)
    return EINVAL;

  _motor->m_fd = open(_config->m_path, O_WRONLY|O_SYNC, 0);
  if (_motor->m_fd < 0)
  {
    res = errno;
    fprintf(stderr, "open(%s) failed: %d\n", _config->m_path, res);
    _motor->m_fd = -1;
    return res;
  }

  _motor->m_powerBackFull    = _config->m_powerBackFull;
  _motor->m_powerBackZero    = _config->m_powerBackZero;
  _motor->m_powerNeutral     = _config->m_powerNeutral;
  _motor->m_powerForwardZero = _config->m_powerForwardZero;
  _motor->m_powerForwardFull = _config->m_powerForwardFull;

  return 0;
}

static int do_roverCloseMotor(RoverOutput* _rover,
                              RoverMotor* _motor)
{
  int res;

  if (_rover == NULL || _motor == NULL)
    return EINVAL;

  if (close(_motor->m_fd) != 0)
  {
    res = errno;
    fprintf(stderr, "close() failed: %d\n", res);
    return res;
  }
  _motor->m_fd = -1;

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

  if ((res = do_roverOpenMotorMsp(_rover, &_rover->m_motorMsp3, &_config->m_motorMsp3)) != 0)
  {
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp2);
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp1);
    return res;
  }

  if ((res = do_roverOpenMotorMsp(_rover, &_rover->m_motorMsp4, &_config->m_motorMsp4)) != 0)
  {
    do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp3);
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

  do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp4);
  do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp3);
  do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp2);
  do_roverCloseMotorMsp(_rover, &_rover->m_motorMsp1);

  return 0;
}

static int do_roverMotorMspSetPower(RoverOutput* _rover,
                                    RoverMotorMsp* _motor,
                                    int _power)
{
  int res;

  if (_rover == NULL || _motor == NULL)
    return EINVAL;

  int pwm = 0x0;

  if (_power == 0) // neutral
    pwm = 0x0;
  else if (_power < 0) // back
  {
    if (_power < -100)
      pwm = -_motor->m_powerMax;
    else
      pwm = -(_motor->m_powerMin + ((_motor->m_powerMax-_motor->m_powerMin)*(-_power))/100);
  }
  else // forward
  {
    if (_power > 100)
      pwm = _motor->m_powerMax;
    else
      pwm = _motor->m_powerMin + ((_motor->m_powerMax-_motor->m_powerMin)*_power)/100;
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
  cmd[1] =  pwm&0xff;

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

static int do_roverCtrlChasisSetup(RoverOutput* _rover, const RoverConfig* _config)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  chasis->m_motorLeft1  = &_rover->m_motorMsp1;
  chasis->m_motorLeft2  = &_rover->m_motorMsp2;
  chasis->m_motorRight1 = &_rover->m_motorMsp3;
  chasis->m_motorRight2 = &_rover->m_motorMsp4;
  chasis->m_lastSpeed = 0;
  chasis->m_lastYaw = 0;
  chasis->m_zeroX = _config->m_zeroX;
  chasis->m_zeroY = _config->m_zeroY;
  chasis->m_zeroMass = _config->m_zeroMass;

  return 0;
}

static int do_roverCtrlChasisPaused(RoverOutput* _rover){
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, 0);
  do_roverMotorMspSetPower(_rover, chasis->m_motorLeft2, 0);
  do_roverMotorMspSetPower(_rover, chasis->m_motorRight1, 0);
  do_roverMotorMspSetPower(_rover, chasis->m_motorRight2, 0);

  return 0;
}

static int do_roverCtrlLighting(RoverOutput* _rover)
{
  RoverControlChasis* chasis = &_rover->m_ctrlChasis;

  int red   = 256-(uint8_t)(treeColor >> 16);
  int green = 256-(uint8_t)(treeColor >> 8);
  int blue  = 256-(uint8_t)treeColor;

  do_roverMotorMspSetPower(_rover, chasis->m_motorLeft1, -(int)((((float)red)/256.0)*100.0));
  do_roverMotorMspSetPower(_rover, chasis->m_motorLeft2, -(int)((((float)green)/256.0)*100.0));
  do_roverMotorMspSetPower(_rover, chasis->m_motorRight1,-(int)((((float)blue)/256.0)*100.0));
  do_roverMotorMspSetPower(_rover, chasis->m_motorRight2, 100);

  return 0;
}

void roverSetPause(RoverOutput* _rover)
{
  _rover->m_state = _rover->m_state == StatePaused ? StateLighting : StatePaused;
  _rover->m_stateEntryTime.tv_sec = 0;
  fprintf(stderr, "paused\n"); //tmp
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

  _rover->m_opened = true;

  do_roverCtrlChasisSetup(_rover, _config);

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

  _rover->m_state = StatePaused;
  _rover->m_stateEntryTime.tv_sec = 0;
  _rover->m_stateEntryTime.tv_nsec = 0;

  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp1, 0);
  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp2, 0);
  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp3, 0);
  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp4, 0);

  do_roverCtrlLighting(_rover);

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
  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp3, 0);
  do_roverMotorMspSetPower(_rover, &_rover->m_motorMsp4, 0);

  return 0;
}

int roverOutputControlManual(RoverOutput* _rover, int _ctrlChasisLR, int _ctrlChasisFB, int _ctrlHand, int _ctrlArm)
{
  if (_rover == NULL)
    return EINVAL;
  if (!_rover->m_opened)
    return ENOTCONN;

  return 0;
}

int roverOutputControlAuto(RoverOutput* _rover, int _targetX, int _targetY, int _targetMass)
{
  if (_rover == NULL)
    return EINVAL;
  if (!_rover->m_opened)
    return ENOTCONN;
/*
  if (_rover->m_stateEntryTime.tv_sec == 0)
    clock_gettime(CLOCK_MONOTONIC, &_rover->m_stateEntryTime);

  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  long long msPassed = (now.tv_sec - _rover->m_stateEntryTime.tv_sec) * 1000
                     + (now.tv_nsec - _rover->m_stateEntryTime.tv_nsec) / 1000000;
*/
  switch (_rover->m_state)
  {
    case StatePaused:
      do_roverCtrlChasisPaused(_rover);
      break;

    case StateLighting:
      do_roverCtrlLighting(_rover);
      break;

  }

  return 0;
}
