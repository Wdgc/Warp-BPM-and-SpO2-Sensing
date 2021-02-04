/*
	Authored 2016-2018. Phillip Stanley-Marbell.

	Additional contributions, 2018 onwards: Jan Heck, Chatura Samarakoon, Youchao Wang, Sam Willis.

	All rights reserved.

	Redistribution and use in source and binary forms, with or without
	modification, are permitted provided that the following conditions
	are met:

	*	Redistributions of source code must retain the above
		copyright notice, this list of conditions and the following
		disclaimer.

	*	Redistributions in binary form must reproduce the above
		copyright notice, this list of conditions and the following
		disclaimer in the documentation and/or other materials
		provided with the distribution.

	*	Neither the name of the author nor the names of its
		contributors may be used to endorse or promote products
		derived from this software without specific prior written
		permission.

	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
	"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
	LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
	FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
	COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
	INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
	BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
	CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
	LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
	ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
	POSSIBILITY OF SUCH DAMAGE.
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fsl_misc_utilities.h"
#include "fsl_device_registers.h"
#include "fsl_i2c_master_driver.h"
#include "fsl_spi_master_driver.h"
#include "fsl_rtc_driver.h"
#include "fsl_clock_manager.h"
#include "fsl_power_manager.h"
#include "fsl_mcglite_hal.h"
#include "fsl_port_hal.h"
#include "fsl_lpuart_driver.h"

#include "gpio_pins.h"
#include "SEGGER_RTT.h"
#include "warp.h"
//#include "arm_math.h"

#define WARP_FRDMKL03



#	include "devSSD1331.h"
#	include "devMAX30102.h"



#define WARP_BUILD_ENABLE_SEGGER_RTT_PRINTF



/*
*	BTstack includes WIP
*/
// #include "btstack_main.h"


#define						kWarpConstantStringI2cFailure		"\rI2C failed, reg 0x%02x, code %d\n"
#define						kWarpConstantStringErrorInvalidVoltage	"\rInvalid supply voltage [%d] mV!"
#define						kWarpConstantStringErrorSanity		"\rSanity check failed!"




#ifdef WARP_BUILD_ENABLE_DEVMAX30102						
volatile WarpI2CDeviceState 			deviceMAX30102State;
#endif
/*
 *	TODO: move this and possibly others into a global structure
 */
volatile i2c_master_state_t			i2cMasterState;
volatile spi_master_state_t			spiMasterState;
volatile spi_master_user_config_t		spiUserConfig;


/*
 *	TODO: move magic default numbers into constant definitions.
 */
volatile uint32_t			gWarpI2cBaudRateKbps		= 200;
volatile uint32_t			gWarpUartBaudRateKbps		= 1;
volatile uint32_t			gWarpSpiBaudRateKbps		= 200;
volatile uint32_t			gWarpSleeptimeSeconds		= 0;
volatile WarpModeMask			gWarpMode			= kWarpModeDisableAdcOnSleep;
volatile uint32_t			gWarpI2cTimeoutMilliseconds	= 5;
volatile uint32_t			gWarpSpiTimeoutMicroseconds	= 5;
volatile uint32_t			gWarpMenuPrintDelayMilliseconds	= 10;
volatile uint32_t			gWarpSupplySettlingDelayMilliseconds = 1;


/*
 *	Calculation numbers
 */
//Finger placement test
bool active =false;

//Sample Aquisition
volatile uint32_t IRsample;
volatile uint32_t Redsample;

//DC Filter
int RedDCtemp;
int RedDCinput;
int RedDCresult;
volatile int RedDCtempPrevious =0;
int IRDCtemp;
int IRDCinput;
int IRDCresult;
volatile int IRDCtempPrevious =0;
const float DCalpha = 0.95; 

//mean filter
volatile int sumMean;
volatile int bufferMean[5];
volatile uint8_t indexMean;
volatile float mean;
volatile float resultMean;
//volatile uint8_t countMean;

//SpO2 calculation
int SSqRed = 0;
int SSqIR = 0;
volatile uint16_t SpO2Counter; 
volatile float SqSumRatio;
volatile float SpO2Result;

//volatile uint8_t RMSint;
uint8_t SpO2ResultInt;

int DCFilterRed;	
int DCFilterIR;

//bpm calculation
bool beatBool =false;
volatile uint8_t beat =0;
volatile uint8_t sampleCounter=0;
volatile uint8_t bpm =0;
volatile uint8_t bpmPrevious =0;


//void				devMAX30102configure(const uint8_t i2cAddress);
float				meanDiffFilter(int value);
int 				IRDC_filter(uint32_t value);
int 				RedDC_filter(uint32_t value);
void 				reportSpO2(uint8_t);
int 				sqrt(int);
int					log(int);
void 				setScreen(void);
void				displayBPM(uint8_t value);

//float 			ln(float y);
//int 				msb(int v);


/*
 *	From KSDK power_manager_demo.c <<BEGIN>>>
 */

clock_manager_error_code_t clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData);

/*
 *	static clock callback table.
 */
clock_manager_callback_user_config_t		clockManagerCallbackUserlevelStructure =
									{
										.callback	= clockManagerCallbackRoutine,
										.callbackType	= kClockManagerCallbackBeforeAfter,
										.callbackData	= NULL
									};

static clock_manager_callback_user_config_t *	clockCallbackTable[] =
									{
										&clockManagerCallbackUserlevelStructure
									};

clock_manager_error_code_t
clockManagerCallbackRoutine(clock_notify_struct_t *  notify, void *  callbackData)
{
	clock_manager_error_code_t result = kClockManagerSuccess;

	switch (notify->notifyType)
	{
		case kClockManagerNotifyBefore:
			break;
		case kClockManagerNotifyRecover:
		case kClockManagerNotifyAfter:
			break;
		default:
			result = kClockManagerError;
		break;
	}

	return result;
}


/*
 *	Override the RTC IRQ handler
 */
void
RTC_IRQHandler(void)
{
	if (RTC_DRV_IsAlarmPending(0))
	{
		RTC_DRV_SetAlarmIntCmd(0, false);
	}
}

/*
 *	Override the RTC Second IRQ handler
 */
void
RTC_Seconds_IRQHandler(void)
{
	gWarpSleeptimeSeconds++;
}

/*
 *	Power manager user callback
 */
power_manager_error_code_t callback0(power_manager_notify_struct_t *  notify,
					power_manager_callback_data_t *  dataPtr)
{
	WarpPowerManagerCallbackStructure *		callbackUserData = (WarpPowerManagerCallbackStructure *) dataPtr;
	power_manager_error_code_t			status = kPowerManagerError;

	switch (notify->notifyType)
	{
		case kPowerManagerNotifyBefore:
			status = kPowerManagerSuccess;
			break;
		case kPowerManagerNotifyAfter:
			status = kPowerManagerSuccess;
			break;
		default:
			callbackUserData->errorCount++;
			break;
	}

	return status;
}


void
enableSPIpins(void)
{
	CLOCK_SYS_EnableSpiClock(0);

	/*	Warp KL03_SPI_MISO	--> PTA6	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAlt3);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAlt3);

	/*	Warp KL03_SPI_SCK	--> PTB0	(ALT3)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAlt3);


	/*
	 *	Initialize SPI master. See KSDK13APIRM.pdf Section 70.4
	 *
	 */
	uint32_t			calculatedBaudRate;
	spiUserConfig.polarity		= kSpiClockPolarity_ActiveHigh;
	spiUserConfig.phase		= kSpiClockPhase_FirstEdge;
	spiUserConfig.direction		= kSpiMsbFirst;
	spiUserConfig.bitsPerSec	= gWarpSpiBaudRateKbps * 1000;
	SPI_DRV_MasterInit(0 /* SPI master instance */, (spi_master_state_t *)&spiMasterState);
	SPI_DRV_MasterConfigureBus(0 /* SPI master instance */, (spi_master_user_config_t *)&spiUserConfig, &calculatedBaudRate);
}



void
disableSPIpins(void)
{
	SPI_DRV_MasterDeinit(0);


	/*	Warp KL03_SPI_MISO	--> PTA6	(GPI)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 6, kPortMuxAsGpio);

	/*	Warp KL03_SPI_MOSI	--> PTA7	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTA_BASE, 7, kPortMuxAsGpio);

	/*	Warp KL03_SPI_SCK	--> PTB0	(GPIO)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 0, kPortMuxAsGpio);

	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MOSI);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_MISO);
	GPIO_DRV_ClearPinOutput(kWarpPinSPI_SCK);


	CLOCK_SYS_DisableSpiClock(0);
}

void
configureI2Cpins(uint8_t pullupValue)
{
#ifdef WARP_BUILD_ENABLE_DEVISL23415
	/*
	 *	Configure the two ISL23415 DCPs over SPI
	 */
	uint8_t valuesDCP[2] = {pullupValue, pullupValue};
	writeDeviceRegisterISL23415(kWarpISL23415RegWR, valuesDCP, 4);
#endif
}

void
enableI2Cpins(uint8_t pullupValue)
{
	CLOCK_SYS_EnableI2cClock(0);

	/*	Warp KL03_I2C0_SCL	--> PTB3	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAlt2);

	/*	Warp KL03_I2C0_SDA	--> PTB4	(ALT2 == I2C)		*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAlt2);


	I2C_DRV_MasterInit(0 /* I2C instance */, (i2c_master_state_t *)&i2cMasterState);

	configureI2Cpins(pullupValue);
}



void
disableI2Cpins(void)
{
	I2C_DRV_MasterDeinit(0 /* I2C instance */);


	/*	Warp KL03_I2C0_SCL	--> PTB0	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 3, kPortMuxAsGpio);

	/*	Warp KL03_I2C0_SDA	--> PTB1	(GPIO)			*/
	PORT_HAL_SetMuxMode(PORTB_BASE, 4, kPortMuxAsGpio);


	/*
	 *	Reset DCP configuration
	 */
	configureI2Cpins(0x80); /* Defaults DCP configuration ISL datasheet FN7780 Rev 2.00 - page 14 */

	/*
	 *	Drive the I2C pins low
	 */
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SDA);
	GPIO_DRV_ClearPinOutput(kWarpPinI2C0_SCL);


	CLOCK_SYS_DisableI2cClock(0);
}



int
main(void)
{
	uint8_t					key;
	uint16_t				menuI2cPullupValue = 32768;

	rtc_datetime_t				warpBootDate;

	power_manager_user_config_t		warpPowerModeWaitConfig;
	power_manager_user_config_t		warpPowerModeStopConfig;
	power_manager_user_config_t		warpPowerModeVlpwConfig;
	power_manager_user_config_t		warpPowerModeVlpsConfig;
	power_manager_user_config_t		warpPowerModeVlls0Config;
	power_manager_user_config_t		warpPowerModeVlls1Config;
	power_manager_user_config_t		warpPowerModeVlls3Config;
	power_manager_user_config_t		warpPowerModeRunConfig;

	const power_manager_user_config_t	warpPowerModeVlprConfig = {
							.mode			= kPowerManagerVlpr,
							.sleepOnExitValue	= false,
							.sleepOnExitOption	= false
						};

	power_manager_user_config_t const *	powerConfigs[] = {
							/*
							 *	NOTE: This order is depended on by POWER_SYS_SetMode()
							 *
							 *	See KSDK13APIRM.pdf Section 55.5.3
							 */
							&warpPowerModeWaitConfig,
							&warpPowerModeStopConfig,
							&warpPowerModeVlprConfig,
							&warpPowerModeVlpwConfig,
							&warpPowerModeVlpsConfig,
							&warpPowerModeVlls0Config,
							&warpPowerModeVlls1Config,
							&warpPowerModeVlls3Config,
							&warpPowerModeRunConfig,
						};

	WarpPowerManagerCallbackStructure			powerManagerCallbackStructure;

	/*
	 *	Callback configuration structure for power manager
	 */
	const power_manager_callback_user_config_t callbackCfg0 = {
							callback0,
							kPowerManagerCallbackBeforeAfter,
							(power_manager_callback_data_t *) &powerManagerCallbackStructure};

	/*
	 *	Pointers to power manager callbacks.
	 */
	power_manager_callback_user_config_t const *	callbacks[] = {
								&callbackCfg0
						};



	/*
	 *	Enable clock for I/O PORT A and PORT B
	 */
	CLOCK_SYS_EnablePortClock(0);
	CLOCK_SYS_EnablePortClock(1);



	/*
	 *	Setup board clock source.
	 */
	g_xtal0ClkFreq = 32768U;



	/*
	 *	Initialize KSDK Operating System Abstraction layer (OSA) layer.
	 */
	OSA_Init();



	/*
	 *	Setup SEGGER RTT to output as much as fits in buffers.
	 *
	 *	Using SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL can lead to deadlock, since
	 *	we might have SWD disabled at time of blockage.
	 */
	SEGGER_RTT_ConfigUpBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_NO_BLOCK_TRIM);


	SEGGER_RTT_WriteString(0, "\n\n\n\rBooting Warp, in 3... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "2... ");
	OSA_TimeDelay(200);
	SEGGER_RTT_WriteString(0, "1...\n\r");
	OSA_TimeDelay(200);



	/*
	 *	Configure Clock Manager to default, and set callback for Clock Manager mode transition.
	 *
	 *	See "Clocks and Low Power modes with KSDK and Processor Expert" document (Low_Power_KSDK_PEx.pdf)
	 */
	CLOCK_SYS_Init(	g_defaultClockConfigurations,
			CLOCK_CONFIG_NUM,
			&clockCallbackTable,
			ARRAY_SIZE(clockCallbackTable)
			);
	CLOCK_SYS_UpdateConfiguration(CLOCK_CONFIG_INDEX_FOR_RUN, kClockManagerPolicyForcible);



	/*
	 *	Initialize RTC Driver
	 */
	RTC_DRV_Init(0);



	/*
	 *	Set initial date to 1st January 2016 00:00, and set date via RTC driver
	 */
	warpBootDate.year	= 2016U;
	warpBootDate.month	= 1U;
	warpBootDate.day	= 1U;
	warpBootDate.hour	= 0U;
	warpBootDate.minute	= 0U;
	warpBootDate.second	= 0U;
	RTC_DRV_SetDatetime(0, &warpBootDate);



	/*
	 *	Setup Power Manager Driver
	 */
	memset(&powerManagerCallbackStructure, 0, sizeof(WarpPowerManagerCallbackStructure));


	warpPowerModeVlpwConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpwConfig.mode = kPowerManagerVlpw;

	warpPowerModeVlpsConfig = warpPowerModeVlprConfig;
	warpPowerModeVlpsConfig.mode = kPowerManagerVlps;

	warpPowerModeWaitConfig = warpPowerModeVlprConfig;
	warpPowerModeWaitConfig.mode = kPowerManagerWait;

	warpPowerModeStopConfig = warpPowerModeVlprConfig;
	warpPowerModeStopConfig.mode = kPowerManagerStop;

	warpPowerModeVlls0Config = warpPowerModeVlprConfig;
	warpPowerModeVlls0Config.mode = kPowerManagerVlls0;

	warpPowerModeVlls1Config = warpPowerModeVlprConfig;
	warpPowerModeVlls1Config.mode = kPowerManagerVlls1;

	warpPowerModeVlls3Config = warpPowerModeVlprConfig;
	warpPowerModeVlls3Config.mode = kPowerManagerVlls3;

	warpPowerModeRunConfig.mode = kPowerManagerRun;

	POWER_SYS_Init(	&powerConfigs,
			sizeof(powerConfigs)/sizeof(power_manager_user_config_t *),
			&callbacks,
			sizeof(callbacks)/sizeof(power_manager_callback_user_config_t *)
			);



	/*
	 *	Switch CPU to Very Low Power Run (VLPR) mode
	 */
	warpSetLowPowerMode(kWarpPowerModeVLPR, 0);



	/*
	 *	Initialize the GPIO pins with the appropriate pull-up, etc.,
	 *	defined in the inputPins and outputPins arrays (gpio_pins.c).
	 *
	 *	See also Section 30.3.3 GPIO Initialization of KSDK13APIRM.pdf
	 */
	GPIO_DRV_Init(inputPins  /* input pins */, outputPins  /* output pins */);



	/*
	 *	Toggle LED3 (kWarpPinSI4705_nRST)
	 */
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_SetPinOutput(kWarpPinSI4705_nRST);
	OSA_TimeDelay(200);
	GPIO_DRV_ClearPinOutput(kWarpPinSI4705_nRST);



	/*
	 *	Initialize all the sensors
	 */


//#ifdef WARP_BUILD_ENABLE_DEVMAX30102
	//initMAX30102(     0x57   /* i2cAddress */,       &deviceMAX30102State      );		
//#endif



	/*
	 *	TODO: initialize the kWarpPinKL03_VDD_ADC, write routines to read the VDD and temperature
	 */






	devSSD1331init();


		
		/*
		 *	We break up the prints with small delays to allow us to use small RTT print
		 *	buffers without overrunning them when at max CPU speed.
		 */
		SEGGER_RTT_WriteString(0, "\r\n\n\n\n[ *\t\t\t\tW\ta\tr\tp\t(rev. b)\t\t\t* ]\n");
		OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
		SEGGER_RTT_WriteString(0, "\r[  \t\t\t\t      Cambridge / Physcomplab   \t\t\t\t  ]\n\n");
		OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);

		/*
		 *	Initialise/configure PPG sensor.
		 *  Prepare OLED for results.
		 */

		enableI2Cpins(menuI2cPullupValue);
		SEGGER_RTT_printf(0, "\nAttempting MAX30102 config ");
		initMAX30102(     0x57   ,       &deviceMAX30102State      );
		OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);	
		devMAX30102configure();
		OSA_TimeDelay(gWarpMenuPrintDelayMilliseconds);
		setScreen();

		
		while (1)
		{
			/*
			*  Check to see if finger is placed correctly
			*/
			readNextSample();
			if ( (Redsample>1000) || (IRsample>1000) )
			{
				active=true;
				SEGGER_RTT_printf(0, "\nFinger Detected, starting measurements");
				
				writeSensorRegisterMAX30102(FIFO_READ,0x00);
				writeSensorRegisterMAX30102(FIFO_WRITE,0x00); //Reset read/write pointers to known state before beginning measurements
			}

			while (active)
			{
				float normalisedRed;
				SSqIR = 0;
				SSqRed = 0;
			
				/*
				* Sample aquisition/filtering
				*/
				readNextSample();
				DCFilterRed = RedDC_filter(Redsample);
				DCFilterIR = IRDC_filter(IRsample); //DC removal

				SSqIR += (DCFilterIR * DCFilterIR);
				SSqRed += (DCFilterRed * DCFilterRed); //Sum of Squares used for SpO2 calculation	
					
				normalisedRed = meanDiffFilter(DCFilterRed); //Normalising Red signal for BPM detection
				
				SpO2Counter++;
				sampleCounter++;


				/*
				* BPM: Check to see if passing High threshold.
				*/
				if (normalisedRed > 19 && beatBool==false)
				{
					//SEGGER_RTT_WriteString(0,"b" );
					beat++;
					beatBool = true;
					
					if(beat==1)
					{
						sampleCounter=0; //Start samples count if this is first peak
					}
				}

				/*
				* BPM: Check to see if passing low threshold
				*/
				if (normalisedRed < -10  && beatBool == true)
				{
					beatBool = false;
					//SEGGER_RTT_WriteString(0, "\nthreshold down crossed");
				}

				/*
				* Calculate BPM over 4 beat range (beat=5).
				*/
				if (beat ==5)
				{	
					bpm = 240000/((11 * sampleCounter)); //one sample takes 10.35mS (excluding Spo2 code as sample is reset)
					SEGGER_RTT_printf(0, "\nBPM = %d (samples counted =%d)", bpm, sampleCounter); 
					
					
					if ((bpm > 20) && (bpm<200)) //Sanity check -limits BPM to physiological range
					{
						if ((bpm != bpmPrevious))//Only update screen if different from previous BPM
						{
							clearSection(63, 0, 110, 30);
							displayBPM(bpm);
						}
						bpmPrevious = bpm;
					}
					
					//reset state machine
					beat=0;
					sampleCounter=0;
					beatBool=false;
					
				}

				if (SpO2Counter >999) //this has to be high so that BPM isn't constatly interrupted -potentially faulty at low BPMs
				{
					SqSumRatio = log( (sqrt((SSqRed/1000))) ) / log( (sqrt((SSqIR/1000)) )); //RMS  	
					SpO2Result = (125.0 - (SqSumRatio *25)); //Calibrated SpO2 estimation
					
					SpO2ResultInt = (int) SpO2Result; //RTTprintf() cannot return f.p. values
					SEGGER_RTT_printf(0, "\nSpO2=%d\n", SpO2ResultInt);
						
					if((SpO2Result<105)) //to acount for zeroing errors from non-f.p. returns for log() and sqrt()
					{
						SpO2ResultInt+= 6.0; //additional calibration step
						clearSection(63, 20, 110, 50);
						reportSpO2(SpO2ResultInt);
					}
												
					//reset to get next bpm measurement
					sampleCounter = 0;
					SSqIR = 0;
					SSqRed = 0;	
					beatBool =false;
					SpO2Counter = 0;
					beat=0;					
				}				
			}
		OSA_TimeDelay(2000);
		}
	return 0;
}





void
devMAX30102configure(const uint8_t i2cAddress)
{ SEGGER_RTT_printf(0, "\nattempting MAX30102 config ");
	return (writeSensorRegisterMAX30102(MODE_CONFIG,0x40) |  //resets all regsiters

			writeSensorRegisterMAX30102(INTERRUPT_ENABLE_1, 0x00) | // Data ready & FIFO almost full interrupt = Off (0xc0), Ambmiant light overf = Off

			writeSensorRegisterMAX30102(INTERRUPT_ENABLE_2, 0x00) | // Temp conversion finished interupt (can read temp) =OFF

			writeSensorRegisterMAX30102(FIFO_CONFIG, 0x40) | // SET FIFO: Sample averaging = 4, FIFO rolls on full = False, Almos full interupt value =False [prev 0x00]

			writeSensorRegisterMAX30102(MODE_CONFIG, 0x03) |  // SET MODE: 0x03 SP02 (red+IR readings and timings)

			writeSensorRegisterMAX30102(SPO2_CONFIG, 0x72) | // SET SPO2: 18-bit ADC resolution = min, Sample rate = 800 Hz, Pulse width = 215us (17 bit ADC)

			writeSensorRegisterMAX30102(LED1_PULSE_AMPLITUDE, 0x24) | // SET LED1 (RED) PULSE AMPLITUDE ~7mA

			writeSensorRegisterMAX30102(LED2_PULSE_AMPLITUDE, 0x25)  // SET LED2 (IR) PULSE AMPLITUDE ~7mA

	); //N.B. This breaks if I2C write commands not passed as return (?)
} 

void
displayBPM(uint8_t value)
{
	int i = 80;
	while (value)
	{
		writeDigit(i, 60, value % 10);
		value /= 10;
		i -= 6;
	}
	return;
}

void 
setScreen(void)
{
	writeCharacter(24, 60, 'b');
	writeCharacter(30, 60, 'p');
	writeCharacter(36, 60, 'm');
	writeCharacter(42,60, ':');
	
	writeDigit(24, 25, 5);
	writeCharacter(30, 25, 'p');
	writeDigit(36, 25, 0);
	writeDigit(42, 25, 2);
	writeCharacter(48,60, ':');
	return;
}


int 
IRDC_filter(uint32_t value)
{
	IRDCinput = value;
	IRDCtemp = IRDCinput + (DCalpha * IRDCtempPrevious);
	IRDCresult = IRDCtemp - IRDCtempPrevious;
	IRDCtempPrevious = IRDCtemp;

	return IRDCresult;
}

int 
RedDC_filter(uint32_t value)
{
	RedDCinput = value;
	RedDCtemp = RedDCinput + (DCalpha * RedDCtempPrevious);
	RedDCresult = RedDCtemp - RedDCtempPrevious;
	RedDCtempPrevious = RedDCtemp;

	return RedDCresult;
}

float
meanDiffFilter(int DCFilterRed)
{
	sumMean -= bufferMean[indexMean];
	bufferMean[indexMean] = DCFilterRed;
	sumMean += bufferMean[indexMean];
	
	indexMean++;
	indexMean = indexMean % 5;
	
	/*if (countMean < 5)
	{countMean++;}*/

	mean = sumMean / 5/*countMean*/;
	resultMean= mean - DCFilterRed;
	return resultMean;
	
}

void
reportSpO2(uint8_t value)
{
	int i = 80;
	while (value)
	{
		writeDigit(i, 25, value % 10);
		value /= 10;
		i -= 6;
	}
	return;
}

/*
Potentially impliment?
f.p. log function, array based DC filter
*/
/*
float 
ln(float y) 
{
    int log2;
    float divisor, x, result;

    log2 = msb((int)y); // See: https://stackoverflow.com/a/4970859/6630230
    divisor = (float)(1 << log2);
    x = y / divisor;    // normalized value between [1.0, 2.0]

    result = -1.7417939 + (2.8212026 + (-1.4699568 + (0.44717955 - 0.056570851 * x) * x) * x) * x;
    result += ((float)log2) * 0.69314718; // ln(2) = 0.69314718

    return result;
}

int
msb(int v)
{
	int r = 0;

	while (v >>= 1) {
    	r++;
	}
	return r;
}

void
Red_DC_filter(uint32_t Redsample[])
{
	for (int i = 0, i < bufferSize, i++)
	{
		input = Redsample[i];
		tempDC = input + (alpha * previousTempDC);
		resultDC[i] = tempDC - previousTempDC
		previousTempDC = temporaryDC
	}
	return resultDC[i];
}*/