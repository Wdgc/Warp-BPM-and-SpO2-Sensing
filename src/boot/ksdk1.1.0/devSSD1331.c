
#include <stdint.h>

#include "fsl_spi_master_driver.h"
#include "fsl_port_hal.h"

#include "SEGGER_RTT.h"
#include "gpio_pins.h"
#include "warp.h"
#include "devSSD1331.h"

volatile uint8_t	inBuffer[32];
volatile uint8_t	payloadBytes[32];

uint8_t first_char_flag; //remove

/*
 *	Override Warp firmware's use of these pins and define new aliases.
 */
enum
{
	kSSD1331PinMOSI		= GPIO_MAKE_PIN(HW_GPIOA, 8),
	kSSD1331PinSCK		= GPIO_MAKE_PIN(HW_GPIOA, 9),
	kSSD1331PinCSn		= GPIO_MAKE_PIN(HW_GPIOB, 13),
	kSSD1331PinDC		= GPIO_MAKE_PIN(HW_GPIOA, 12),
	kSSD1331PinRST		= GPIO_MAKE_PIN(HW_GPIOB, 0),
};

void clearSection(uint8_t col_start, uint8_t row_start, uint8_t col_end, uint8_t row_end)
{
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(col_start); // Column start address
	writeCommand(row_start); // Row start address
	writeCommand(col_end);   // Column end address
	writeCommand(row_end);   // Row end address
	return;
}

/*
Charater writing commands taken from: https://github.com/lawjb/heart-rate-monitor/blob/master/src/boot/ksdk1.1.0/devSSD1331.c
*/
void writeDigit(uint8_t column, uint8_t row, uint8_t digit)
{
	row = 63 - row; // Screen is upside down
	switch (digit)
	{
	case 0:
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 8);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		break;
	}
	case 1:
	{
		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 2); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 2); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 2:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row + 4); // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 8); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		break;
	}
	case 3:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 4:
	{
		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 4); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}

	case 5:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 4); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 6:
	{
		for (int i = 0; i < 9; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 8); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 7:
	{

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row);		  // Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row);		  // Row start address
		writeCommand(column);	 // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 8:
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 8);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case 9:
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 4);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 4); // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	}
	return;
}

// Writes a character symbol in a varying sized box
void writeCharacter(uint8_t column, uint8_t row, char character)
{
	row = 63 - row; // Screen is upside down
	switch (character)
	{
	case 'o':
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row);		  // Row start
		writeCommand(column + 3); // Col end
		writeCommand(row + 3);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		break;
	}
	case 'C':
	{
		for (int i = 0; i < 9; i += 8)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column);	 // Column start address
			writeCommand(row + i);	// Row start address
			writeCommand(column + 4); // Column end address
			writeCommand(row + i);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 8); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		break;
	}
	case 'b':
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row + 4);	// Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 8);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);  // Column start address
		writeCommand(row);	 // Row start address
		writeCommand(column);  // Column end address
		writeCommand(row + 8); // Row end address
		writeCommand(0xFF);	// Red
		writeCommand(0xFF);	// Green
		writeCommand(0xFF);	// Blue

		break;
	}
	case 'p':
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column);	 // Col start
		writeCommand(row + 4);	// Row start
		writeCommand(column + 4); // Col end
		writeCommand(row + 8);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0x00);		  // Fill red
		writeCommand(0x00);		  // Fill green
		writeCommand(0x00);		  // Fill blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);   // Column start address
		writeCommand(row + 8);  // Row start address
		writeCommand(column);   // Column end address
		writeCommand(row + 11); // Row end address
		writeCommand(0xFF);		// Red
		writeCommand(0xFF);		// Green
		writeCommand(0xFF);		// Blue

		break;
	}
	case 'm':
	{
		for (int i = 0; i < 5; i += 4)
		{
			writeCommand(kSSD1331CommandDRAWLINE);
			writeCommand(column + i); // Column start address
			writeCommand(row + 4);	// Row start address
			writeCommand(column + i); // Column end address
			writeCommand(row + 8);	// Row end address
			writeCommand(0xFF);		  // Red
			writeCommand(0xFF);		  // Green
			writeCommand(0xFF);		  // Blue
		}

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 2); // Column start address
		writeCommand(row + 6);	// Row start address
		writeCommand(column + 2); // Column end address
		writeCommand(row + 8);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 2); // Column end address
		writeCommand(row + 6);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column + 2); // Column start address
		writeCommand(row + 6);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue

		break;
	}
	case '.':
	{
		writeCommand(kSSD1331CommandDRAWRECT);
		writeCommand(column + 1); // Col start
		writeCommand(row + 7);	// Row start
		writeCommand(column + 2); // Col end
		writeCommand(row + 8);	// Row end
		writeCommand(0xFF);		  // Line red
		writeCommand(0xFF);		  // Line green
		writeCommand(0xFF);		  // Line blue
		writeCommand(0xFF);		  // Fill red
		writeCommand(0xFF);		  // Fill green
		writeCommand(0xFF);		  // Fill blue
		break;
	}
	case '-':
	{
		writeCommand(kSSD1331CommandDRAWLINE);
		writeCommand(column);	 // Column start address
		writeCommand(row + 4);	// Row start address
		writeCommand(column + 4); // Column end address
		writeCommand(row + 4);	// Row end address
		writeCommand(0xFF);		  // Red
		writeCommand(0xFF);		  // Green
		writeCommand(0xFF);		  // Blue
		break;
	}
	}
	return;
}


/*
//Alternative write commands. Adapted from Paul Staron ssd1331.cpp: https://os.mbed.com/users/star297/code/ssd1331/docs/tip/ssd1331_8cpp_source.html

void PutChar(uint8_t column,uint8_t row, int value) //removed external font
{
        if(value == '\n') {
            char_x = 0;
            char_y = char_y + Y_height;
        }
        if ((value < 31) || (value > 127)) return;   // test char range
        if (char_x + X_width > width) {
            char_x = 0;
            char_y = char_y + Y_height;
            if (char_y >= height - Y_height) {
                char_y = 0;
            }
        }

        
        int i,j,w,lpx,lpy,k,l,xw;
        unsigned char Temp=0;
        j = 0; i = 0;
        w = X_width;
        lpx=2;
	lpy=2;
        xw = X_width;
        
        for(i=0; i<xw; i++) {
            for ( l=0; l<lpx; l++) {
                Temp = font6x8[value-32][i];
                
                uint8_t line_length = 0;
                
                for(j=Y_height-1; j>=0; j--) {
                    
                    for (k=0; k<lpy; k++) {    
                        if( (Temp & 0x80)==0x80) {
                            line_length++;
                        }else if(line_length>0){
                            line(char_x+(i*lpx)+l, char_y+(((j+1)*lpy)-1)-k, char_x+(i*lpx)+l,  char_y+(((j+1)*lpy)-1)-k + line_length-1, Char_Color);
                            line_length = 0;
                        }
                        
                        
                    }
                    Temp = Temp << 1;
                }
            }
        }
        lpx=2;
		lpy=2;
        char_x += (w*lpx);
}*/



 /*//Needed for writing characters
void line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint16_t color)
{
    if  ( x1 > width ) x1 = width;
    if  ( y1 > height ) y1 = height;
    if  ( x2 > width ) x2 = width;
    if  ( y2 > height ) y2 = height;

    unsigned char cmd[11]= { 0 };
    cmd[0] = GAC_FILL_ENABLE_DISABLE;
    cmd[1] = 0;      // fill 0, empty 0
    writeCommand_buf(&cmd, 2);
    cmd[0] = GAC_DRAW_LINE;
    cmd[1] = (unsigned char)x1;
    cmd[2] = (unsigned char)y1;
    cmd[3] = (unsigned char)x2;
    cmd[4] = (unsigned char)y2;
    cmd[5] = (unsigned char)(((color>>11)&0x1F)<<1);    // Blue
    cmd[6] = (unsigned char)((color>>5)&0x3F);          // Green
    cmd[7] = (unsigned char)((color&0x1F)<<1);          // Red

    writeCommand_buf(&cmd, 8);
    
}*/


 //Write character
 /*
int 
charactertoscreen(int character, uint8_t x)
{
    uint8_t y = 10;
    PutChar(x, y, character);
    x += X_width;
    
return x;
}

void reset_cursor(void)
{
    char_x = 0;
    char_y = 0;
}
*/

void
clearscreen(void)
{
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
}

static int
writeCommand(uint8_t commandByte)
{
	spi_status_t status;

	/*
	 *	Drive /CS low.
	 *
	 *	Make sure there is a high-to-low transition by first driving high, delay, then drive low.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	/*
	 *	Drive DC low (command).
	 */
	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	payloadBytes[0] = commandByte;
	status = SPI_DRV_MasterTransferBlocking(0	/* master instance */,
					NULL		/* spi_master_user_config_t */,
					(const uint8_t * restrict)&payloadBytes[0],
					(uint8_t * restrict)&inBuffer[0],
					1		/* transfer size */,
					1000		/* timeout in microseconds (unlike I2C which is ms) */);

	/*
	 *	Drive /CS high
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}
/*
int writeCommand_buf(uint8_t* commandByteBuf, uint8_t len)
{
	spi_status_t status;

	
	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);
	OSA_TimeDelay(10);
	GPIO_DRV_ClearPinOutput(kSSD1331PinCSn);

	GPIO_DRV_ClearPinOutput(kSSD1331PinDC);

	status = SPI_DRV_MasterTransferBlocking(0	//master instance ,
					NULL	// spi_master_user_config_t ,
					(const uint8_t * restrict)commandByteBuf,
					(uint8_t * restrict)&inBuffer[0],
					len		// transfer size,
					1000	// timeout in microseconds (unlike I2C which is ms)
					);

	GPIO_DRV_SetPinOutput(kSSD1331PinCSn);

	return status;
}*/

int
devSSD1331init(void)
{
	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Re-configure SPI to be on PTA8 and PTA9 for MOSI and SCK respectively.
	 */
	PORT_HAL_SetMuxMode(PORTA_BASE, 8u, kPortMuxAlt3);
	PORT_HAL_SetMuxMode(PORTA_BASE, 9u, kPortMuxAlt3);

	enableSPIpins();

	/*
	 *	Override Warp firmware's use of these pins.
	 *
	 *	Reconfigure to use as GPIO.
	 */
	PORT_HAL_SetMuxMode(PORTB_BASE, 13u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTA_BASE, 12u, kPortMuxAsGpio);
	PORT_HAL_SetMuxMode(PORTB_BASE, 0u, kPortMuxAsGpio);


	/*
	 *	RST high->low->high.
	 */
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_ClearPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);
	GPIO_DRV_SetPinOutput(kSSD1331PinRST);
	OSA_TimeDelay(100);

	/*
	 *	Initialization sequence, borrowed from https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino
	 */
	writeCommand(kSSD1331CommandDISPLAYOFF);	// 0xAE
	writeCommand(kSSD1331CommandSETREMAP);		// 0xA0
	writeCommand(0x72);				// RGB Color
	writeCommand(kSSD1331CommandSTARTLINE);		// 0xA1
	writeCommand(0x0);
	writeCommand(kSSD1331CommandDISPLAYOFFSET);	// 0xA2
	writeCommand(0x0);
	writeCommand(kSSD1331CommandNORMALDISPLAY);	// 0xA4
	writeCommand(kSSD1331CommandSETMULTIPLEX);	// 0xA8
	writeCommand(0x3F);				// 0x3F 1/64 duty
	writeCommand(kSSD1331CommandSETMASTER);		// 0xAD
	writeCommand(0x8E);
	writeCommand(kSSD1331CommandPOWERMODE);		// 0xB0
	writeCommand(0x0B);
	writeCommand(kSSD1331CommandPRECHARGE);		// 0xB1
	writeCommand(0x31);
	writeCommand(kSSD1331CommandCLOCKDIV);		// 0xB3
	writeCommand(0xF0);				// 7:4 = Oscillator Frequency, 3:0 = CLK Div Ratio (A[3:0]+1 = 1..16)
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8A
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGEB);	// 0x8B
	writeCommand(0x78);
	writeCommand(kSSD1331CommandPRECHARGEA);	// 0x8C
	writeCommand(0x64);
	writeCommand(kSSD1331CommandPRECHARGELEVEL);	// 0xBB
	writeCommand(0x3A);
	writeCommand(kSSD1331CommandVCOMH);		// 0xBE
	writeCommand(0x3E);
	writeCommand(kSSD1331CommandMASTERCURRENT);	// 0x87
	writeCommand(0x0F);
	writeCommand(kSSD1331CommandCONTRASTA);		// 0x81
	writeCommand(0x91);
	writeCommand(kSSD1331CommandCONTRASTB);		// 0x82
	writeCommand(0xFF);
	writeCommand(kSSD1331CommandCONTRASTC);		// 0x83
	writeCommand(0x7D);
	writeCommand(kSSD1331CommandDISPLAYON);		// Turn on oled panel
	

	/*
	 *	To use fill commands, you will have to issue a command to the display to enable them. See the manual.
	 */
	writeCommand(kSSD1331CommandFILL);
	writeCommand(0x01);
	

	/*
	 *	Clear Screen
	 */
	writeCommand(kSSD1331CommandCLEAR);
	writeCommand(0x00);
	writeCommand(0x00);
	writeCommand(0x5F);
	writeCommand(0x3F);
	

       //Creating green screen
        writeCommand(kSSD1331CommandDRAWRECT);
        writeCommand(0x00);
        writeCommand(0x00);
        writeCommand(0x5F);
        writeCommand(0x3F);
        writeCommand(0x00);
        writeCommand(0xFF);
        writeCommand(0x00);
        writeCommand(0x00);
        writeCommand(0xFF);
	writeCommand(0x00);

	/*
	 *	Read the manual for the SSD1331 (SSD1331_1.2.pdf) to figure
	 *	out how to fill the entire screen with the brightest shade
	 *	of green.
	 */

	OSA_TimeDelay(1000);
	clearscreen();


	SEGGER_RTT_WriteString(0, "\r\n\tDone with draw rectangle...\n");



	return 0;
}                    
