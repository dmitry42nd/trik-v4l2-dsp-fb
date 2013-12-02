#ifndef TRIK_V4L2_DSP_ld_INTERNAL_LD_H_
#define TRIK_V4L2_DSP_ld_INTERNAL_LD_H_
/*Line detector*/
#include <stdbool.h>
#include <inttypes.h>

//#include <linux/ld.h>

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus


typedef struct LDConfig // what user wants to set
{
  const char* m_path;
} LDConfig;

typedef struct LineDetector
{
  int                      m_targetX; //percentage
} LineDetector;


int lineDetectorInit(bool _verbose);
int lineDetectorFini();

int lineDetectorOpen(LineDetector* _ld, const LDConfig* _config);
int lineDetectorClose(LineDetector* _ld);
int lineDetectorStart(LineDetector* _ld);
int lineDetectorStop(LineDetector* _ld);

int lineDetectorPutTargetX(LineDetector* _ld, void** _framePtr, size_t* _frameSize);//change params!
int lineDetectorGetTargetX(LineDetector* _ld);//change params!

#ifdef __cplusplus
} // extern "C"
#endif // __cplusplus

#endif // !TRIK_V4L2_DSP_ld_INTERNAL_ld_H_
