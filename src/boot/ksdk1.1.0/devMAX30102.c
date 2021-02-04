#include <stdlib.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
#include "devMAX30102.h"

extern volatile WarpI2CDeviceState	deviceMAX30102State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;
extern volatile uint32_t gWarpI2cTimeoutMilliseconds;
uint8_t bytesOnFIFO;
uint32_t mask = 0x03FFFF;
extern volatile uint32_t IRsample;
extern volatile uint32_t Redsample;

WarpStatus
writeSensorRegisterMAX30102(uint8_t deviceRegister, uint8_t value)
{
	uint8_t payloadByte[1], commandByte[1];
    i2c_status_t	status;

	i2c_device_t slave =
	{
		.address = deviceMAX30102State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;
    payloadByte[0] = value;
	status = I2C_DRV_MasterSendDataBlocking(
						0,
						&slave,
						commandByte,
						1,
						payloadByte,
						1,
						gWarpI2cTimeoutMilliseconds); 
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
		SEGGER_RTT_WriteString(0, "\ruh oh\n");
	}

	return kWarpStatusOK;
}


WarpStatus
readSensorRegisterMAX30102(uint8_t deviceRegister, int numberOfBytes)
{	
	if (numberOfBytes == 0)
	{
		return;
	}

	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;
	

	i2c_device_t slave =
	{
		.address = deviceMAX30102State.i2cAddress, 
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceMAX30102State.i2cBuffer,
							numberOfBytes,
							gWarpI2cTimeoutMilliseconds);


	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}


void
initMAX30102(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	SEGGER_RTT_WriteString(0, "\rMAX30102 function start\n");
	readSensorRegisterMAX30102(INTERRUPT_STATUS_1, 1); //Clears any power up interrups
	SEGGER_RTT_WriteString(0, "\rMAX30102 power interrupt cleared\n");
	OSA_TimeDelay(100);
	deviceStatePointer->i2cAddress	= i2cAddress;
    writeSensorRegisterMAX30102(MODE_CONFIG,0x40); //resets all regsisters
	OSA_TimeDelay(100);
	SEGGER_RTT_WriteString(0, "\rMAX30102 reset command passed\n");
	OSA_TimeDelay(100);
	
	return;
}


WarpStatus
readNextSample(void)
{


	WarpStatus i2cReadStatus_FIFO_READ, i2cReadStatus_FIFO_WRITE, i2cReadStatus_FIFO_DATA;


	//Check pointer locations to see how many samples are available on FIFO. NB reading these registries will incriment them. 
	i2cReadStatus_FIFO_READ = readSensorRegisterMAX30102(FIFO_READ, 1);
	uint8_t read_pointer = deviceMAX30102State.i2cBuffer[0];
	i2cReadStatus_FIFO_WRITE = readSensorRegisterMAX30102(FIFO_WRITE, 1);
	uint8_t write_pointer = deviceMAX30102State.i2cBuffer[0];

	if((read_pointer =! write_pointer))//checking if samples are availble
	{
		bytesOnFIFO = 6 * (write_pointer-read_pointer); // One sample = 6 bytes (IR + Red LEDs are 3 bytes each)
	
	}
	else{return;}
	
	writeSensorRegisterMAX30102(FIFO_READ,0x00);

	i2cReadStatus_FIFO_DATA = readSensorRegisterMAX30102(FIFO_DATA, 6); 
	
	if ((i2cReadStatus_FIFO_READ | i2cReadStatus_FIFO_WRITE | i2cReadStatus_FIFO_DATA) != kWarpStatusOK)
	{
        SEGGER_RTT_printf(0, "\nError with I2C read FIFO registers");
		return;
	}

	uint8_t data[6]; 
	for (int i = 0; i < 6; i++)//change to samples on FIFO?
	{
		data[i] = deviceMAX30102State.i2cBuffer[i];
	}

	IRsample = (data[3] <<16 | data[4] << 8 | data[5]) & mask;
	Redsample = (data[0] <<16 | data[1] << 8 | data[2]) & mask; //Sample not on 23:18, Mask above MSB 
	
	//Reset read/write pointers to beginning of buffer
	writeSensorRegisterMAX30102(FIFO_READ,0x00);
	writeSensorRegisterMAX30102(FIFO_WRITE,0x00);
	
	/*if (i2cReadStatus_FIFO_DATA == kWarpStatusOK)
	{
		SEGGER_RTT_printf(0, "\n%d,%d", IRsample, Redsample);
	}*/
	
	return kWarpStatusOK;
}