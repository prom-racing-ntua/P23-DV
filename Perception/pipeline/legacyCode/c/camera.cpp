#include "include/GxIAPI.h"
#include "include/helpers.h"
#include <cstdio>
#include <unistd.h>
//#define SN_CAMERA1  KE0220030138

static void GX_STDC OnFrameCallbackFun(GX_FRAME_CALLBACK_PARAM* pFrame)
{
  if (pFrame->status == 0)
  {
    printf("Received image: %d\n", pFrame->nFrameID);
    for (int i = 0; i < sizeof(pFrame->pImgBuf); i++){
      printf("Pixel Value of %d: %d\n",i, pFrame->pImgBuf);
    }
  }
  return;
}


int main(int argc, char** argv)
{
  printf("Hello World\n");

  // Variable Initializations
  GX_STATUS status;
  uint32_t nDeviceNum = 0;
  GX_OPEN_PARAM stOpenParam;
  int cnt = 0;
  //Initialize Library and check for errors
  status = GXInitLib();
  if (status != GX_STATUS_SUCCESS){
    printf("Library Init Failed\n");
    return 0;
  }

  // Enumerate Devices (1 second for Timeout)
  status = GXUpdateDeviceList(&nDeviceNum, 1000);

  //If any device found
  if (status == GX_STATUS_SUCCESS && nDeviceNum > 0){
    GX_DEVICE_BASE_INFO *pBaseinfo = new GX_DEVICE_BASE_INFO[nDeviceNum];
    size_t nSize = nDeviceNum * sizeof(GX_DEVICE_BASE_INFO);
    status = GXGetAllDeviceBaseInfo(pBaseinfo, &nSize);
    printf("Cameras: %d, nSize: %d, status code: %d\n", nDeviceNum, nSize, status);
    //delete []pBaseinfo;

    //Set Open Parameters
    GX_DEV_HANDLE hDevice  = NULL;

    stOpenParam.accessMode = GX_ACCESS_EXCLUSIVE;
    stOpenParam.openMode = GX_OPEN_INDEX;
    //stOpenParam.pszContent = "1";
    stOpenParam.openMode = GX_OPEN_SN;
    stOpenParam.pszContent = pBaseinfo[0].szSN;

    //Open Device
    //status = GXOpenDevice(&stOpenParam, &hDevice);
    hDevice = openCamera(stOpenParam);
    printf("Camera Opened Successfully, status code: %d\n", status);


    //Start Acquasition
    status = GXRegisterCaptureCallback(hDevice, NULL, OnFrameCallbackFun);

    // status = GXSetEnum(hDevice, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ACQUISITION_FRAME_RATE_MODE_ON);
    // status = GXSetFloat(hDevice,GX_FLOAT_ACQUISITION_FRAME_RATE, 20.0);
    status = setFPS(hDevice, 20.0);

    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_START);

    sleep(1);

    status = GXSendCommand(hDevice, GX_COMMAND_ACQUISITION_STOP);

    status = GXCloseDevice(hDevice);
    printf("Camera closed Successfully, status code: %d\n", status);
  }

  //Close Library - Exit Program
  status = GXCloseLib();
  printf("Library Closed Successfully, status code: %d\n", status);
  return 0;
}
