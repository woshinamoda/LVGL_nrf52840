 #ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H		
#include <stdint.h>
#include "nrf_gpio.h"
#include "nrf_delay.h"	    
#include "nrf_drv_spi.h"

#include "main.h"
/****************************************
CS  P0.26 --- LCD_TFTƬѡ
RST P0.25 --- LCD_TFT��λ
AO  P0.24 --- LCD_TFT����/�����л�
SDA P0.23 --- LCD_TFT����
SCK P0.22 --- LCD_TFTʱ��
****************************************/
#define   LCD_CS     NRF_GPIO_PIN_MAP(1,10)  //LCD_TFTƬѡ
#define   LCD_RST    NRF_GPIO_PIN_MAP(1,11)  //LCD_TFT��λ
#define   LCD_DC     NRF_GPIO_PIN_MAP(1,12)  //LCD_TFT����/�����л�
#define   LCD_SDA    NRF_GPIO_PIN_MAP(1,13)  //LCD_TFT����
#define   LCD_CLK    NRF_GPIO_PIN_MAP(1,14)  //LCD_TFTʱ��




//SPI��ʾ���ӿ�
//��λ����LCD_RST����ߵ͵�ƽ����
#define SPILCD_RST_SET nrf_gpio_pin_set(LCD_RST)
#define SPILCD_RST_CLR nrf_gpio_pin_clear(LCD_RST)

//����/��������LCD_DC����ߵ͵�ƽ����
#define LCD_DC_SET     nrf_gpio_pin_set(LCD_DC)
#define LCD_DC_CLR     nrf_gpio_pin_clear(LCD_DC) 

  
//LCD��Ҫ������
typedef struct  
{ 					    
	uint16_t width;			//LCD ���
	uint16_t height;	  //LCD �߶�
	uint8_t	wramcmd;		//��ʼдgramָ��
	uint8_t  setxcmd;		//����x����ָ��
	uint8_t  setycmd;		//����y����ָ��	 
}_lcd_dev; 	  

//LCD����
extern _lcd_dev lcddev;	//����LCD��Ҫ����
//LCD�Ļ�����ɫ�ͱ���ɫ	   
extern uint16_t  POINT_COLOR;//Ĭ�Ϻ�ɫ    
extern uint16_t  BACK_COLOR; //������ɫ.Ĭ��Ϊ��ɫ


//////////////////////////////////////////////////////////////////////////////////	 
//-----------------LCD�˿ڶ���---------------- 
//#define	LCD_REST PBout(1) //LCD REST    		 PB1 	    
//LCD��ַ�ṹ��
typedef struct
{
	uint16_t LCD_REG;
	uint16_t LCD_RAM;
} LCD_TypeDef;
//ʹ��NOR/SRAM�� Bank1.sector4,��ַλHADDR[27,26]=11 A10��Ϊ�������������� 
//ע������ʱSTM32�ڲ�������һλ����! 111110=0X3E			    
#define LCD_BASE        ((uint32_t)(0x60000000 | 0x0007FFFE))
#define LCD             ((LCD_TypeDef *) LCD_BASE)
//////////////////////////////////////////////////////////////////////////////////

//������ɫ
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
#define BROWN 			     0XBC40 //��ɫ
#define BRRED 			     0XFC07 //�غ�ɫ
#define GRAY  			     0X8430 //��ɫ
//GUI��ɫ

#define DARKBLUE      	 0X01CF	//����ɫ
#define LIGHTBLUE      	 0X7D7C	//ǳ��ɫ  
#define GRAYBLUE       	 0X5458 //����ɫ
//������ɫΪPANEL����ɫ 
 
#define LIGHTGREEN     	 0X841F //ǳ��ɫ
//#define LIGHTGRAY        0XEF5B //ǳ��ɫ(PANNEL)
#define LGRAY 			 0XC618 //ǳ��ɫ(PANNEL),���屳��ɫ

#define LGRAYBLUE        0XA651 //ǳ����ɫ(�м����ɫ)
#define LBBLUE           0X2B12 //ǳ����ɫ(ѡ����Ŀ�ķ�ɫ)
	    															  
void LCD_Init(void);													   	//��ʼ��

void LCD_DisplayOn(void);																	//����ʾ
void LCD_DisplayOff(void);																//����ʾ
void LCD_Clear(uint16_t Color);	 													//����
void LCD_SetCursor(uint16_t Xpos, uint16_t Ypos);					//���ù��
void LCD_DrawPoint(uint16_t x,uint16_t y);								//����
void LCD_Fast_DrawPoint(uint16_t x,uint16_t y,uint16_t color);																		//���ٻ���
void Draw_Circle(uint16_t x0,uint16_t y0,uint8_t r);																							//��Բ
void LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);														//����
void LCD_DrawRectangle(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);												//������
void LCD_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t color);										//��䵥ɫ
void LCD_Color_Fill(uint16_t sx,uint16_t sy,uint16_t ex,uint16_t ey,uint16_t *color);							//���ָ����ɫ
void LCD_ShowChar(uint16_t x,uint16_t y,uint8_t num,uint8_t size,uint8_t mode);										//��ʾһ���ַ�
void LCD_ShowNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size);  									//��ʾһ������
void LCD_ShowxNum(uint16_t x,uint16_t y,uint32_t num,uint8_t len,uint8_t size,uint8_t mode);			//��ʾ ����
void LCD_ShowString(uint16_t x,uint16_t y,uint16_t width,uint16_t height,uint8_t size,uint8_t *p);//��ʾһ���ַ���,12/16����
	  
void showimage(uint16_t x,uint16_t y); //��ʾ40*40ͼƬ
void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_WriteRAM_Prepare(void);
void LCD_WR_DATA8(uint8_t da);   //д8λ����  
 
void showhanzi16(unsigned int x,unsigned int y,unsigned char index);//16*16����
void showhanzi32(unsigned int x,unsigned int y,unsigned char index);//32*32����						  		 
#endif  
	 
	 



