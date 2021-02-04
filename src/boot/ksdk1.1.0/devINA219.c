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


extern volatile WarpI2CDeviceState	deviceINA219State;
extern volatile uint32_t		gWarpI2cBaudRateKbps;



void
initINA219(const uint8_t i2cAddress, WarpI2CDeviceState volatile *  deviceStatePointer)
{
	SEGGER_RTT_WriteString(0, "\rinitINA219 function start\n");
	deviceStatePointer->i2cAddress	= i2cAddress;

	return;
}

WarpStatus
writeSensorRegisterINA219(uint8_t deviceRegister, uint16_t value)
{
	uint8_t highBits = value >> 8;
	uint8_t lowBits = value & 0xFF;
	uint8_t payloadByte[2] = {highBits, lowBits};
	uint8_t commandByte[1]	= {0xFF};
	i2c_status_t	status;

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	commandByte[0] = deviceRegister;

	status = I2C_DRV_MasterSendDataBlocking(
						0,
						&slave,
						commandByte,
						1,
						payloadByte,
						2,
						100);
	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}



WarpStatus
readSensorRegisterINA219(uint8_t deviceRegister, int numberOfBytes)
{
	uint8_t		cmdBuf[1] = {0xFF};
	i2c_status_t	status;
	USED(numberOfBytes);

	i2c_device_t slave =
	{
		.address = deviceINA219State.i2cAddress,
		.baudRate_kbps = gWarpI2cBaudRateKbps
	};

	cmdBuf[0] = deviceRegister;

	status = I2C_DRV_MasterReceiveDataBlocking(
							0,
							&slave,
							cmdBuf,
							1,
							(uint8_t *)deviceINA219State.i2cBuffer,
							numberOfBytes,
							500);


	if (status != kStatus_I2C_Success)
	{
		return kWarpStatusDeviceCommunicationFailed;
	}

	return kWarpStatusOK;
}

void
printSensorDataINA219(bool hexModeFlag)
{
	uint8_t	readSensorRegisterValueMSB;
	uint8_t	readSensorRegisterValueLSB;
	uint16_t		readSensorRegisterValueCombined;
	WarpStatus	i2cReadStatus;


	/* Can only send a byte at a time.
	 *First byte returned by INA219 is MSB.
	 * Need to join MSB with LSB.
	 */

	i2cReadStatus = readSensorRegisterINA219(0x04, 2 /* numberOfBytes */);
	readSensorRegisterValueMSB = deviceINA219State.i2cBuffer[0];
	readSensorRegisterValueLSB = deviceINA219State.i2cBuffer[1];
	readSensorRegisterValueCombined = ((readSensorRegisterValueMSB << 8) || (readSensorRegisterValueLSB));



	if (i2cReadStatus != kWarpStatusOK)
	{
		SEGGER_RTT_WriteString(0, " ----,");
	}
	else
	{
		if (hexModeFlag)
		{
			SEGGER_RTT_printf(0, " 0x%02x 0x%02x,", readSensorRegisterValueMSB, readSensorRegisterValueLSB);
		}
		else
		{
			SEGGER_RTT_printf(0, " %d,", readSensorRegisterValueCombined);
		}
	}
}


