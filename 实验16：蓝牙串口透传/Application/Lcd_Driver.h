 #ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H		
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"	    
#include "nrf_drv_spi.h"

#include "main.h"
/****************************************
CS  P0.26 --- LCD_TFT片选
RST P0.25 --- LCD_TFT复位
AO  P0.24 --- LCD_TFT数据/命令切换
SDA P0.23 --- LCD_TFT数据
SCK P0.22 --- LCD_TFT时钟
****************************************/
#define   LCD_CS     NRF_GPIO_PIN_MAP(1,10)  //LCD_TFT片选
#define   LCD_RST    NRF_GPIO_PIN_MAP(1,11)  //LCD_TFT复位
#define   LCD_DC     NRF_GPIO_PIN_MAP(1,12)  //LCD_TFT数据/命令切换
#define   LCD_SDA    NRF_GPIO_PIN_MAP(1,13)  //LCD_TFT数据
#define   LCD_CLK    NRF_GPIO_PIN_MAP(1,14)  //LCD_TFT时钟




//SPI显示屏接口
//复位引脚LCD_RST输出高低电平定义
#define SPILCD_RST_SET nrf_gpio_pin_set(LCD_RST)
#define SPILCD_RST_CLR nrf_gpio_pin_clear(LCD_RST)

//数据/命令引脚LCD_DC输出高低电平定义
#define LCD_DC_SET     nrf_gpio_pin_set(LCD_DC)
#define LCD_DC_CLR     nrf_gpio_pin_clear(LCD_DC) 

  
//LCD重要参数集
typedef struct  
{ 					    
	uint16_t width;			//LCD 宽度
	uint16_t height;	  //LCD 高度
	uint8_t	wramcmd;		//开始写gram指令
	uint8_t  setxcmd;		//设置x坐标指令
	uint8_t  setycmd;		//设置y坐标指令	 
}_lcd_dev; 	  

//LCD参数
extern _lcd_dev lcddev;	//管理LCD重要参数
//LCD的画笔颜色和背景色	   
extern uint16_t  POINT_COLOR;//默认红色    
extern uint16_t  BACK_COLOR; //背景颜色.默认为白色


//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD端口定义---------------- 
//#define	LCD_REST PBout(1) //LCD REST    		 PB1 	    
//LCD地址结构体
typedef struct
{
	uint16_t LCD_REG;
	uint16_t LCD_RAM;
} LCD_TypeDef;
//使用NOR/SRAM的 Bank1.sector4,地址位HADDR[27,26]=11 A10作为数据命令区分线 
//注意设置时STM32内部会右移一位对其! 111110=0X3E			    
#define LCD_BASE        ((uint32_t)(0x60000000 | 0x0007FFFE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)
//////////////////////////////////////////////////////////////////////////////////

//画笔颜色
#define WHITE         	 0xFFFF
#define BLACK         	 0x0000	  
#define BLUE         	   0x001F  
#define BRED             0XF81F
#define GRED 			       0XFFE0
#define GBLUE			       0X07FF
#define RED           	 0xF800
#define MAGENTA       	 0xF81F
#define GREEN         	 0x07E0
#define CYAN          	 0x7FFF
#define YELLOW        	 0xFFE0
#define BROWN 			     0XBC40 //棕色
#define BRRED 			     0XFC07 //棕红色
#define GRAY  			     0X8430 //灰色
//GUI颜色

#define DARKBLUE      	 0X01CF	//深蓝色
#define LIGHTBLUE      	 0X7D7C	//浅蓝色  
#define GRAYBLUE       	 0X5458 //灰蓝色
//以上三色为PANEL的颜色 
 
#define LIGHTGREEN     	 0X841F //浅绿色
//#define LIGHTGRAY        0XEF5B //浅灰色(PANNEL)
#define LGRAY 			 0XC618 //浅灰色(PANNEL),窗体背景色

#define LGRAYBLUE        0XA651 //浅灰蓝色(中间层颜色)
#define LBBLUE           0X2B12 //浅棕蓝色(选择条目的反色)
	    															  
void LCD_Init(void);													   	//初始化

void LCD_DisplayOn(void);																	//开显示
void LCD_DisplayOff(void);																//关显示
void LCD_Clear(uint16_t Color);	 													//清屏
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);					//设置光标
void LCD_DrawPoint(uint16_t x,uint16_t y);								//画点
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);																		//快速画点
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r);																							//画圆
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);														//画线
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);												//画矩形
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);										//填充单色
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);							//填充指定颜色
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);										//显示一个字符
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size);  									//显示一个数字
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode);			//显示 数字
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p);//显示一个字符串,12/16字体
	  
void showimage(uint16_t x,uint16_t y); //显示40*40图片
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WriteRAM_Prepare(void);
void LCD_WR_DATA8(uint8_t da);   //写8位数据  
 
void showhanzi16(unsigned int x,unsigned int y,unsigned char index);//16*16汉字
void showhanzi32(unsigned int x,unsigned int y,unsigned char index);//32*32汉字						  		 
#endif  
	 
	 



