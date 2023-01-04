#include "include/GxIAPI.h"
#include <cstdio>

GX_DEV_HANDLE openCamera(GX_OPEN_PARAM stOpenParam)
{
  GX_DEV_HANDLE hDevice = NULL;
  GX_STATUS status;

  status = GXOpenDevice(&stOpenParam, &hDevice);

  return hDevice;
}


GX_STATUS setFPS(GX_DEV_HANDLE hDevice, float fps)
{
  GX_STATUS status;

  status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
  status = GXSetFloat(hDevice, GX_FLOAT_ACQUISITION_FRAME_RATE, fps);
  return status;
}
