#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>

#include <linux/videodev2.h>
#include <libv4l2.h>

#include "internal/v4l2.h"



static int do_v4l2inputOpen(V4L2Input* _v4l2, const char* _path)
{
  int res;

  if (_v4l2 == NULL || _path == NULL)
    return EINVAL;

  _v4l2->m_fd = v4l2_open(_path, O_RDWR|O_NONBLOCK, 0);
  if (_v4l2->m_fd < 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_open(%s) failed: %d\n", _path, res);
    _v4l2->m_fd = -1;
    return res;
  }

  return 0;
}

static int do_v4l2inputClose(V4L2Input* _v4l2)
{
  int res;
  if (_v4l2 == NULL)
    return EINVAL;

  if (v4l2_close(_v4l2->m_fd) != 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_close() failed: %d\n", res);
    return res;
  }

  return 0;
}

static int do_v4l2inputSetFormat(V4L2Input* _v4l2, size_t _width, size_t _height, uint32_t _format)
{
  int res;

  if (_v4l2 == NULL)
    return EINVAL;

  memset(&_v4l2->m_imageFormat, 0, sizeof(_v4l2->m_imageFormat));
  _v4l2->m_imageFormat.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  _v4l2->m_imageFormat.fmt.pix.width = _width;
  _v4l2->m_imageFormat.fmt.pix.height = _height;
  _v4l2->m_imageFormat.fmt.pix.pixelformat = _format;
  _v4l2->m_imageFormat.fmt.pix.field = V4L2_FIELD_NONE;

  if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_S_FMT, &_v4l2->m_imageFormat) != 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_S_FMT) failed: %d\n", res);
    return res;
  }

  // on return, m_imageFormat contains actually used image format

  return 0;
}

static int do_v4l2inputUnsetFormat(V4L2Input* _v4l2)
{
  if (_v4l2 == NULL)
    return EINVAL;

  memset(&_v4l2->m_imageFormat, 0, sizeof(_v4l2->m_imageFormat));

  return 0;
}

static int do_v4l2inputMmapBuffers(V4L2Input* _v4l2)
{
  int res = 0;

  assert(sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers) == sizeof(_v4l2->m_bufferSize)/sizeof(*_v4l2->m_bufferSize));
  if (_v4l2 == NULL)
    return EINVAL;

  struct v4l2_requestbuffers requestBuffers;
  memset(&requestBuffers, 0, sizeof(requestBuffers));
  requestBuffers.count = sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers);
  requestBuffers.type = _v4l2->m_imageFormat.type;
  requestBuffers.memory = V4L2_MEMORY_MMAP;

  if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_REQBUFS, &requestBuffers) != 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) failed: %d\n", res);
    goto exit;
  }

  if (requestBuffers.count <= 0)
  {
    res = ENOSPC;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) returned no buffers\n");
    goto exit;
  }
  else if (requestBuffers.count < sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers))
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) returned only %"PRIu32" buffers of %zu requested\n",
            requestBuffers.count, sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers));
  else if (requestBuffers.count > sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers))
  {
    fprintf(stderr, "v4l2_ioctl(VIDIOC_REQBUFS) returned %"PRIu32" buffers, used only %zu\n",
            requestBuffers.count, sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers));
    requestBuffers.count = sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers);
  }

  size_t bufferIndex;
  for (bufferIndex = 0; bufferIndex < requestBuffers.count; ++bufferIndex)
  {
    struct v4l2_buffer buffer;
    memset(&buffer, 0, sizeof(buffer));
    buffer.index = bufferIndex;
    buffer.type = _v4l2->m_imageFormat.type;
    buffer.memory = V4L2_MEMORY_MMAP;

    if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_QUERYBUF, &buffer) != 0)
    {
      res = errno;
      fprintf(stderr, "v4l2_ioctl(VIDIOC_QUERYBUF, index %zu) failed: %d\n", bufferIndex, res);
      goto exit_unmap;
    }

    _v4l2->m_bufferSize[bufferIndex] = buffer.length;
    _v4l2->m_buffers[bufferIndex] = v4l2_mmap(NULL, buffer.length,
                                              PROT_READ|PROT_WRITE, MAP_SHARED,
                                              _v4l2->m_fd, buffer.m.offset);
    if (_v4l2->m_buffers[bufferIndex] == MAP_FAILED)
    {
      res = errno;
      fprintf(stderr, "v4l2_mmap(index %zu, size %"PRIu32", offset %"PRIu32") failed: %d\n",
              bufferIndex, buffer.length, buffer.m.offset, res);
      goto exit_unmap;
    }
  }

  for (/*bufferIndex*/; bufferIndex < sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers); ++bufferIndex)
  { // fill unused buffers
    _v4l2->m_buffers[bufferIndex] = MAP_FAILED;
    _v4l2->m_bufferSize[bufferIndex] = 0;
  }

  return 0;


  size_t idx;
exit_unmap:
  for (idx = 0; idx < bufferIndex; ++idx)
  {
    if (v4l2_munmap(_v4l2->m_buffers[idx], _v4l2->m_bufferSize[idx]) != 0)
      // do not update res!
      fprintf(stderr, "v4l2_munmap(index %zu, ptr %p, size %"PRIu32") failed: %d\n",
              idx, _v4l2->m_buffers[idx], _v4l2->m_bufferSize[idx], errno);
  }

exit:
  return res;
}

static int do_v4l2inputMunmapBuffers(V4L2Input* _v4l2)
{
  int res = 0;

  assert(sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers) == sizeof(_v4l2->m_bufferSize)/sizeof(*_v4l2->m_bufferSize));
  if (_v4l2 == NULL)
    return EINVAL;

  size_t bufferIndex;
  for (bufferIndex = 0; bufferIndex < sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers); ++bufferIndex)
  {
    if (   _v4l2->m_buffers[bufferIndex] != MAP_FAILED
        && v4l2_munmap(_v4l2->m_buffers[bufferIndex], _v4l2->m_bufferSize[bufferIndex]) != 0)
    {
      res = errno; // last error will be returned
      fprintf(stderr, "v4l2_munmap(index %zu, ptr %p, size %"PRIu32") failed: %d\n",
              bufferIndex, _v4l2->m_buffers[bufferIndex], _v4l2->m_bufferSize[bufferIndex], res);
    }
    _v4l2->m_buffers[bufferIndex] = MAP_FAILED;
    _v4l2->m_bufferSize[bufferIndex] = 0;
  }

  return res;
}

static int do_v4l2inputStart(V4L2Input* _v4l2)
{
  int res = 0;

  assert(sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers) == sizeof(_v4l2->m_bufferSize)/sizeof(*_v4l2->m_bufferSize));
  if (_v4l2 == NULL)
    return EINVAL;

  size_t bufferIndex;
  for (bufferIndex = 0; bufferIndex < sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers); ++bufferIndex)
    if (_v4l2->m_buffers[bufferIndex] != MAP_FAILED)
    {
      struct v4l2_buffer buffer;
      memset(&buffer, 0, sizeof(buffer));
      buffer.index = bufferIndex;
      buffer.type = _v4l2->m_imageFormat.type;
      buffer.memory = V4L2_MEMORY_MMAP;

      if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_QBUF, &buffer) != 0)
      {
        res = errno;
        fprintf(stderr, "v4l2_ioctl(VIDIOC_QBUF, index %zu) failed: %d\n", bufferIndex, res);
        goto exit_stop;
      }
    }

  enum v4l2_buf_type capture = _v4l2->m_imageFormat.type;
  if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_STREAMON, &capture) != 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_STREAMON) failed: %d\n", res);
    goto exit_stop;
  }

  return 0;


exit_stop:
  capture = _v4l2->m_imageFormat.type;
  if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_STREAMOFF, &capture) != 0)
    // do not update res!
    fprintf(stderr, "v4l2_ioctl(VIDIOC_STREAMOFF) failed: %d\n", errno);

//exit:
  return res;
}

static int do_v4l2inputStop(V4L2Input* _v4l2)
{
  int res = 0;

  assert(sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers) == sizeof(_v4l2->m_bufferSize)/sizeof(*_v4l2->m_bufferSize));
  if (_v4l2 == NULL)
    return EINVAL;

  enum v4l2_buf_type capture = _v4l2->m_imageFormat.type;
  if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_STREAMOFF, &capture) != 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_STREAMOFF) failed: %d\n", res);
    return res;
  }

  return 0;
}

static int do_v4l2inputGetFrame(V4L2Input* _v4l2, void** _framePtr, size_t* _frameSize, size_t* _frameIndex)
{
  int res = 0;

  assert(sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers) == sizeof(_v4l2->m_bufferSize)/sizeof(*_v4l2->m_bufferSize));
  if (_v4l2 == NULL || _framePtr == NULL || _frameSize == NULL || _frameIndex == NULL)
    return EINVAL;

  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.type = _v4l2->m_imageFormat.type;
  buffer.memory = V4L2_MEMORY_MMAP;

  if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_DQBUF, &buffer) != 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_DQBUF) failed: %d\n", res);
    return res;
  }

  if (   buffer.index >= sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers)
      || _v4l2->m_buffers[buffer.index] == MAP_FAILED)
  {
    res = ECHRNG;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_DQBUF) returned invalid buffer index %"PRIu32"\n", buffer.index);
    return res;
  }

  *_frameIndex = buffer.index;
  *_framePtr = _v4l2->m_buffers[buffer.index];
  *_frameSize = buffer.bytesused;

  return 0;
}

static int do_v4l2inputPutFrame(V4L2Input* _v4l2, size_t _frameIndex)
{
  int res = 0;

  assert(sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers) == sizeof(_v4l2->m_bufferSize)/sizeof(*_v4l2->m_bufferSize));
  if (_v4l2 == NULL)
    return EINVAL;

  if (   _frameIndex >= sizeof(_v4l2->m_buffers)/sizeof(*_v4l2->m_buffers)
      || _v4l2->m_buffers[_frameIndex] == MAP_FAILED)
    return ECHRNG;

  struct v4l2_buffer buffer;
  memset(&buffer, 0, sizeof(buffer));
  buffer.index = _frameIndex;
  buffer.type = _v4l2->m_imageFormat.type;
  buffer.memory = V4L2_MEMORY_MMAP;

  if (v4l2_ioctl(_v4l2->m_fd, VIDIOC_QBUF, &buffer) != 0)
  {
    res = errno;
    fprintf(stderr, "v4l2_ioctl(VIDIOC_QBUF, index %zu) failed: %d\n", _frameIndex, res);
    return res;
  }

  return 0;
}




int v4l2inputOpen(V4L2Input* _v4l2, const V4L2Config* _config)
{
  int ret = 0;

  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->m_fd != -1)
    return EALREADY;

  ret = do_v4l2inputOpen(_v4l2, _config->m_path);
  if (ret != 0)
    goto exit;

  ret = do_v4l2inputSetFormat(_v4l2, _config->m_width, _config->m_height, _config->m_format);
  if (ret != 0)
    goto exit_close;

  ret = do_v4l2inputMmapBuffers(_v4l2);
  if (ret != 0)
    goto exit_unset_format;

  return 0;


 exit_unset_format:
  do_v4l2inputUnsetFormat(_v4l2);
 exit_close:
  do_v4l2inputClose(_v4l2);
 exit:
  return ret;
}

int v4l2inputClose(V4L2Input* _v4l2)
{
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->m_fd == -1)
    return EALREADY;

  do_v4l2inputMunmapBuffers(_v4l2);
  do_v4l2inputUnsetFormat(_v4l2);
  do_v4l2inputClose(_v4l2);

  return 0;
}

int v4l2inputStart(V4L2Input* _v4l2)
{
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->m_fd == -1)
    return ENOTCONN;

  return do_v4l2inputStart(_v4l2);
}

int v4l2inputStop(V4L2Input* _v4l2)
{
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->m_fd == -1)
    return ENOTCONN;

  return do_v4l2inputStop(_v4l2);
}

int v4l2inputGetFrame(V4L2Input* _v4l2, void** _framePtr, size_t* _frameSize, size_t* _frameIndex)
{
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->m_fd == -1)
    return ENOTCONN;

  return do_v4l2inputGetFrame(_v4l2, _framePtr, _frameSize, _frameIndex);
}

int v4l2inputPutFrame(V4L2Input* _v4l2, size_t _frameIndex)
{
  if (_v4l2 == NULL)
    return EINVAL;
  if (_v4l2->m_fd == -1)
    return ENOTCONN;

  return do_v4l2inputPutFrame(_v4l2, _frameIndex);
}

