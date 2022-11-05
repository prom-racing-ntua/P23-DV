


//Opens Device via SN and returns the hDevice id (kinda like file descriptor
//each camera has a unique hDevice ID)
GX_DEV_HANDLE openCamera(GX_OPEN_PARAM stOpenParam);

//Sets Aqcusition mode to selectable FPS mode and sets FPS to specific value
//For continuous acqusition only
//returns the GX_STATUS of the command
GX_STATUS setFPS(GX_DEV_HANDLE hDevice, float fps);
