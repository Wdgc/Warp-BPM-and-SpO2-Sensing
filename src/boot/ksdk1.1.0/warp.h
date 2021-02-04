#include "fsl_spi_master_driver.h"

#define	min(x,y)	((x) < (y) ? (x) : (y))
#define	USED(x)		(void)(x)






typedef enum
{
	kWarpStatusOK				= 0,

	kWarpStatusDeviceNotInitialized,
	kWarpStatusDeviceCommunicationFailed,
	kWarpStatusBadDeviceCommand,

	/*
	 *	Generic comms error
	 */
	kWarpStatusCommsError,

	/*
	 *	Power mode routines
	 */
	kWarpStatusPowerTransitionErrorVlpr2Wait,
	kWarpStatusPowerTransitionErrorVlpr2Stop,
	kWarpStatusPowerTransitionErrorRun2Vlpw,
	kWarpStatusPowerTransitionErrorVlpr2Vlpr,
	kWarpStatusErrorPowerSysSetmode,
	kWarpStatusBadPowerModeSpecified,


	/*
	 *	Always keep this as the last item.
	 */
	kWarpStatusMax
} WarpStatus;

typedef enum
{
	/*
	 *	NOTE: This order is depended on by POWER_SYS_SetMode()
	 *
	 *	See KSDK13APIRM.pdf Section 55.5.3
	 */
	kWarpPowerModeWAIT,
	kWarpPowerModeSTOP,
	kWarpPowerModeVLPR,
	kWarpPowerModeVLPW,
	kWarpPowerModeVLPS,
	kWarpPowerModeVLLS0,
	kWarpPowerModeVLLS1,
	kWarpPowerModeVLLS3,
	kWarpPowerModeRUN,
} WarpPowerMode;



typedef enum
{
	kWarpModeDisableAdcOnSleep	= (1 << 0),
} WarpModeMask;


typedef enum
{
	kWarpSizesI2cBufferBytes		= 6, //Increased to 6, but will need higher if pulling more than one sample off FIFO at any one time
	kWarpSizesSpiBufferBytes		= 4, /* Was 3 bytes */
	kWarpSizesBME680CalibrationValuesCount	= 41,
} WarpSizes;

typedef struct
{
	uint8_t			i2cAddress;
	
	uint8_t			i2cBuffer[kWarpSizesI2cBufferBytes];

	WarpStatus		deviceStatus;
} WarpI2CDeviceState;



typedef struct
{
	/*
	 *	For holding ksdk-based error codes
	 */
	spi_status_t		ksdk_spi_status;



	uint8_t			spiSourceBuffer[kWarpSizesSpiBufferBytes];
	uint8_t			spiSinkBuffer[kWarpSizesSpiBufferBytes];
	WarpStatus		deviceStatus;
} WarpSPIDeviceState;

typedef struct
{

	WarpStatus		deviceStatus;
} WarpUARTDeviceState;

typedef struct
{
	uint8_t	errorCount;
} WarpPowerManagerCallbackStructure;





WarpStatus	warpSetLowPowerMode(WarpPowerMode powerMode, uint32_t sleepSeconds);
void		enableI2Cpins(uint8_t pullupValue);
void		disableI2Cpins(void);
void		enableSPIpins(void);
void		disableSPIpins(void);
