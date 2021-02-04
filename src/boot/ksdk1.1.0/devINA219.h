#ifndef WARP_BUILD_ENABLE_DEVINA219
#define WARP_BUILD_ENABLE_DEVINA219
#endif
void initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
WarpStatus readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes);
void		printSensorDataINA219(bool hexModeFlag);
WarpStatus writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t value);
