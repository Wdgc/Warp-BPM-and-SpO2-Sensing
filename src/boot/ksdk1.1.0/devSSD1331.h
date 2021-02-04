/*
 *	See https://github.com/adafruit/Adafruit-SSD1331-OLED-Driver-Library-for-Arduino for the Arduino driver.
 *  Otherwise https://github.com/lawjb/heart-rate-monitor/blob/master/src/boot/ksdk1.1.0/devSSD1331.h 
 *    (Commented commands relate to  https://os.mbed.com/users/star297/code/ssd1331/docs/tip/ssd1331_8cpp_source.html )
 */
#include <stdint.h>

typedef enum
{
	kSSD1331ColororderRGB		= 1,
	kSSD1331DelaysHWFILL		= 3,
	kSSD1331DelaysHWLINE		= 1,
} SSD1331Constants;

typedef enum
{
	kSSD1331CommandDRAWLINE		= 0x21,
	kSSD1331CommandDRAWRECT		= 0x22,
	kSSD1331CommandCLEAR		= 0x25,
	kSSD1331CommandFILL		= 0x26,
	kSSD1331CommandSETCOLUMN	= 0x15,
	kSSD1331CommandSETROW		= 0x75,
	kSSD1331CommandCONTRASTA	= 0x81,
	kSSD1331CommandCONTRASTB	= 0x82,
	kSSD1331CommandCONTRASTC	= 0x83,
	kSSD1331CommandMASTERCURRENT	= 0x87,
	kSSD1331CommandSETREMAP		= 0xA0,
	kSSD1331CommandSTARTLINE	= 0xA1,
	kSSD1331CommandDISPLAYOFFSET	= 0xA2,
	kSSD1331CommandNORMALDISPLAY	= 0xA4,
	kSSD1331CommandDISPLAYALLON	= 0xA5,
	kSSD1331CommandDISPLAYALLOFF	= 0xA6,
	kSSD1331CommandINVERTDISPLAY	= 0xA7,
	kSSD1331CommandSETMULTIPLEX	= 0xA8,
	kSSD1331CommandSETMASTER	= 0xAD,
	kSSD1331CommandDISPLAYOFF	= 0xAE,
	kSSD1331CommandDISPLAYON	= 0xAF,
	kSSD1331CommandPOWERMODE	= 0xB0,
	kSSD1331CommandPRECHARGE	= 0xB1,
	kSSD1331CommandCLOCKDIV		= 0xB3,
	kSSD1331CommandPRECHARGEA	= 0x8A,
	kSSD1331CommandPRECHARGEB	= 0x8B,
	kSSD1331CommandPRECHARGEC	= 0x8C,
	kSSD1331CommandPRECHARGELEVEL	= 0xBB,
	kSSD1331CommandVCOMH		= 0xBE,
} SSD1331Commands;




void clearscreen(void);
int	devSSD1331init(void);
static int writeCommand(uint8_t commandByte);
void clearSection(uint8_t col_start, uint8_t row_start, uint8_t col_end, uint8_t row_end);
void writeDigit(uint8_t column, uint8_t row, uint8_t digit);
void writeCharacter(uint8_t column, uint8_t row, char character);

/*  
//Ultimately not needed?  
// Basic RGB color definitions         RGB values                     
 
#define Black           0x0000      //   0,   0,   0 
#define LightGrey       0xC618      // 192, 192, 192 
#define DarkGrey        0x7BEF      // 128, 128, 128 
#define Red             0xF800      // 255,   0,   0 
#define Green           0x07E0      //   0, 255,   0 
#define Cyan            0x07FF      //   0, 255, 255 
#define Blue            0x001F      //   0,   0, 255 
#define Magenta         0xF81F      // 255,   0, 255 
#define Yellow          0xFFE0      // 255, 255,   0 
#define White           0xFFFF      // 255, 255, 255 
*/


/*
//From os.mbed. Can probably delete now.

void PutChar(uint8_t x,uint8_t y,int a);
void line(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint16_t color);
int charactertoscreen(int character, uint8_t x);
void reset_cursor(void);
static int writeCommand_buf(uint8_t* commandByteBuf, uint8_t len);

unsigned char* font;
uint16_t Char_Color;    
uint16_t BGround_Color; 


uint8_t _x;
uint8_t _y;

uint8_t _x1;
uint8_t _x2;
uint8_t _y1;
uint8_t _y2;
uint8_t char_x;
uint8_t char_y;
uint8_t chr_size;
uint8_t cwidth;       
uint8_t cvert;        

#define width   96-1        // Max X axial direction in screen
#define height  64-1        // Max Y axial direction in screen
#define Set_Column_Address  0x15
#define Set_Row_Address     0x75
#define contrastA           0x81
#define contrastB           0x82
#define contrastC           0x83
#define display_on          0xAF
#define display_off         0xAE
#define X_width 6
#define Y_height 10 //18?
#define GAC_DRAW_LINE           0x21    // Draw Line
#define GAC_FILL_ENABLE_DISABLE 0x26    // Enable Fill*/