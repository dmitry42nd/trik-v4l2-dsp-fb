#ifndef TRIK_V4L2_DSP_FB_INTERNAL_RC_H_
#define TRIK_V4L2_DSP_FB_INTERNAL_RC_H_

#include <stdbool.h>
#include <inttypes.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

extern char* rc_speech;

typedef struct RCConfig // what user wants to set
{
  int m_port;
  bool m_stdin;
  bool m_manualMode;

  int m_autoTargetDetectHue;
  int m_autoTargetDetectHueTolerance;
  int m_autoTargetDetectSat;
  int m_autoTargetDetectSatTolerance;
  int m_autoTargetDetectVal;
  int m_autoTargetDetectValTolerance;
} RCConfig;

typedef struct RCInput
{
  int                      m_stdinFd;
  int                      m_serverFd;
  int                      m_connectionFd;
  char*                    m_readBuffer;
  size_t                   m_readBufferSize;
  size_t                   m_readBufferUsed;

  bool                     m_manualMode;

  int                      m_manualCtrlChasisLR;
  int                      m_manualCtrlChasisFB;
  int                      m_manualCtrlHand;
  int                      m_manualCtrlArm;

  int                      m_autoTargetDetectHue;
  int                      m_autoTargetDetectHueTolerance;
  int                      m_autoTargetDetectSat;
  int                      m_autoTargetDetectSatTolerance;
  int                      m_autoTargetDetectVal;
  int                      m_autoTargetDetectValTolerance;
} RCInput;


int rcInputInit(bool _verbose);
int rcInputFini();

int rcInputOpen(RCInput* _rc, const RCConfig* _config);
int rcInputClose(RCInput* _rc);
int rcInputStart(RCInput* _rc);
int rcInputStop(RCInput* _rc);

int rcInputReadStdin(RCInput* _rc);
int rcInputAcceptConnection(RCInput* _rc);
int rcInputReadConnection(RCInput* _rc);

bool rcInputIsManualMode(RCInput* _rc);
int rcInputGetManualCommand(RCInput* _rc, int* _ctrlChasisLR, int* _ctrlChasisFB, int* _ctrlHand, int* _ctrlArm);
int rcInputGetAutoTargetDetect(RCInput* _rc,
                               int* _detectHueFrom, int* _detectHueTo,
                               int* _detectSatFrom, int* _detectSatTo,
                               int* _detectValFrom, int* _detectValTo);

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // !TRIK_V4L2_DSP_FB_INTERNAL_RC_H_
