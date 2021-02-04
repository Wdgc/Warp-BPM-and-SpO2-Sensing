#ifndef WARP_BUILD_ENABLE_DEVMAX30102
#define WARP_BUILD_ENABLE_DEVMAX30102
#endif

void initMAX30102(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer);
WarpStatus readSensorRegisterMAX30102(uint8_t deviceRegister, int numberOfBytes);
WarpStatus writeSensorRegisterMAX30102(uint8_t deviceRegister, uint8_t value);
WarpStatus readNextSample(void);

//register adresses 
#define INTERRUPT_STATUS_1	0x00
#define INTERRUPT_STATUS_2	0x01
#define INTERRUPT_ENABLE_1	0x02
#define INTERRUPT_ENABLE_2	0x03
#define FIFO_CONFIG 0x08
#define MODE_CONFIG 0x09
#define SPO2_CONFIG 0x0A
#define LED1_PULSE_AMPLITUDE 0x0C
#define LED2_PULSE_AMPLITUDE 0x0D
#define FIFO_WRITE 0x04
#define FIFO_READ 0x06
#define FIFO_DATA 0x07



