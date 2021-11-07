#ifndef  __LCD_H
#define	__LCD_H

#include "stm32f4xx_hal.h"
#include "lcd_fonts.h"

// 1. 设置LTDC时钟，这里设置为33MHz，即刷新率在60帧左右，过高或者过低都会造成闪烁 
// 2. 这里为了方便计算数值应在10-70之间，单位为MHz，具体设置时钟的代码在 SystemClock_Config()
// 3. 这里的时钟并不是越快越好！！！
// 4. 过高的时钟会加大对SDRAM的占用，容易造成花屏，并且屏幕本身不支持这么高的刷新率，还会导致屏幕闪烁显示异常等
// 5. 过低的时钟会导致刷新率太低，屏幕会有闪烁
// 6  开双层显示，RGB565和ARGB8888模式下，LTDC时钟最大只能设置为25M（不同的面板会有所差异，需要根据实际情况调整）
// 7. 开双层显示，RGB888和ARGB8888模式下，LTDC时钟最大只能设置为20M（不同的面板会有所差异，需要根据实际情况调整）
#define 	LCD_CLK   33 	// 33M的时钟，刷新率在60Hz左右


// 1. 如果只用单层，该参数定义为1即可，使用双层的话，需要修改为 2
// 2. FK429M1 核心板 使用的是外部 SDRAM 作为显存，起始地址0xD0000000，SDRAM大小为16M
// 3. 显存所需空间 = 分辨率 * 每个像素所占字节数，例如 800*480的屏，使用16位色（RGB565或者AEGB1555），需要显存 800*480*2 = 768000 字节
// 4. 不管是单层显示还是双层显示，都不能超过 SDRAM 的大小
//	5. 如果用户需要双层显示，则 layer1 应设置为带透明色的格式，即 ColorMode_1 设置为 ARGB8888 、 ARGB1555 或者 ARGB4444
// 6. 如果只是单层，推荐使用RGB565，这样可以大大减小对系统资源的占用
// 7. 如果用户开启双层，推荐两层都是16位色，因为SDRAM的速度有限，使用过高的颜色格式容易造成花屏，这时需要降低LTDC的时钟
// 8. 开双层显示，RGB565和ARGB8888模式下，LTDC时钟最大只能设置为25M（不同的面板会有所差异，需要根据实际情况调整）
// 9. 开双层显示，RGB888和ARGB8888模式下，LTDC时钟最大只能设置为20M（不同的面板会有所差异，需要根据实际情况调整）

#define 	LCD_NUM_LAYERS  2			//定义显示的层数，429可驱动两层显示


#define	ColorMode_0   LTDC_PIXEL_FORMAT_RGB565   		//定义层0的颜色格式
//#define	ColorMode_0   LTDC_PIXEL_FORMAT_ARGB1555   
//#define	ColorMode_0   LTDC_PIXEL_FORMAT_ARGB4444
//#define	ColorMode_0   LTDC_PIXEL_FORMAT_RGB888  
//#define	ColorMode_0   LTDC_PIXEL_FORMAT_ARGB8888   

#if  LCD_NUM_LAYERS == 2		

//	#define	ColorMode_1   LTDC_PIXEL_FORMAT_RGB565   	//定义层1的颜色格式
	#define	ColorMode_1   LTDC_PIXEL_FORMAT_ARGB1555   
//	#define	ColorMode_1   LTDC_PIXEL_FORMAT_ARGB4444   
// #define	ColorMode_1   LTDC_PIXEL_FORMAT_RGB888   
//	#define	ColorMode_1   LTDC_PIXEL_FORMAT_ARGB8888   
	
#endif

// 显示方向参数
// 使用示例：LCD_DisplayDirection(Direction_H) ，设置屏幕横屏显示
// 使用示例：LCD_DisplayDirection(Direction_V) ，设置屏幕竖直显示

#define	Direction_H	0		//LCD横屏显示
#define	Direction_V	1		//LCD竖屏显示

// 设置变量显示时多余位补0还是补空格
// 只有 LCD_DisplayNumber() 显示整数 和 LCD_DisplayDecimals()显示小数 这两个函数用到
// 使用示例： LCD_ShowNumMode(Fill_Zero) 设置多余位填充0，例如 123 可以显示为 000123
//
#define  Fill_Zero  0		//填充0
#define  Fill_Space 1		//填充空格


/*---------------------------------------- 常用颜色 ------------------------------------------------------

 1. 这里为了方便用户使用，定义的是32位颜色，然后再通过代码自动转换成对应颜色格式所需要的的颜色
 2. 32位的颜色中，从高位到低位分别对应 A、R、G、B  4个颜色通道，其中A表示透明通道
 3. 最多可设置255级透明色，ff表示不透明，0表示完全透明
 4. 除非使用ARGB1555和ARGB8888等支持透明色的颜色格式，不然透明色不起作用，其中ARGB1555仅支持一位
	 透明色，即仅有透明和不透明两种状态，ARGB4444有16级透明度，ARGB8888支持255级透明度
 5. 用户可以在电脑用调色板获取24位RGB颜色，然后再补充透明通道得到32位的颜色，再将此32位颜色输入
	 LCD_SetColor()或LCD_SetBackColor()就可以显示出相应的颜色。 
 6. 使用示例：纯蓝色的RGB值为0x0000FF，如果不需要透明色，则对应的32位颜色值为 0xff0000FF
 7. 以下定义的颜色都设置为不透明，用户可根据需求自行定义对应的颜色								*/

#define 	LCD_WHITE       0xffFFFFFF		// 纯白色
#define 	LCD_BLACK       0xff000000		// 纯黑色
                           
#define 	LCD_BLUE        0xff0000FF		//	纯蓝色
#define 	LCD_GREEN       0xff00FF00    //	纯绿色
#define 	LCD_RED         0xffFF0000    //	纯红色
#define 	LCD_CYAN        0xff00FFFF    //	蓝绿色
#define 	LCD_MAGENTA     0xffFF00FF    //	紫红色
#define 	LCD_YELLOW      0xffFFFF00    //	黄色
#define 	LCD_GREY        0xff2C2C2C    //	灰色
													
#define 	LIGHT_BLUE      0xff8080FF    //	亮蓝色
#define 	LIGHT_GREEN     0xff80FF80    //	亮绿色
#define 	LIGHT_RED       0xffFF8080    //	亮红色
#define 	LIGHT_CYAN      0xff80FFFF    //	亮蓝绿色
#define 	LIGHT_MAGENTA   0xffFF80FF    //	亮紫红色
#define 	LIGHT_YELLOW    0xffFFFF80    //	亮黄色
#define 	LIGHT_GREY      0xffA3A3A3    //	亮灰色
													
#define 	DARK_BLUE       0xff000080    //	暗蓝色
#define 	DARK_GREEN      0xff008000    //	暗绿色
#define 	DARK_RED        0xff800000    //	暗红色
#define 	DARK_CYAN       0xff008080    //	暗蓝绿色
#define 	DARK_MAGENTA    0xff800080    //	暗紫红色
#define 	DARK_YELLOW     0xff808000    //	暗黄色
#define 	DARK_GREY       0xff404040    //	暗灰色


/*---------------------------------------------------------- 函数声明 -------------------------------------------------------*/
	
void  LTDC_Init(void);  //LCD初始化
void  LCD_Clear(void); //清屏
void  LCD_ClearRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);	// 局部清屏函数

void  LCD_SetLayer(uint8_t Layerx); 					//	设置层
void  LCD_SetColor(uint32_t Color); 				   //	设置画笔颜色
void  LCD_SetBackColor(uint32_t Color);  				//	设置背景颜色
void  LCD_DisplayDirection(uint8_t direction);  	//	设置显示方向

//>>>>>	显示ASCII字符
void  LCD_SetFont(pFONT *fonts);												//	设置ASCII字体
void 	LCD_DisplayChar(uint16_t x, uint16_t y,uint8_t c);				//	显示单个ASCII字符
void 	LCD_DisplayString( uint16_t x, uint16_t y, char *p);	 		//	显示ASCII字符串

//>>>>>	显示中文字符，包括ASCII码
void 	LCD_SetTextFont(pFONT *fonts);										// 设置文本字体，包括中文和ASCII字体
void 	LCD_DisplayChinese(uint16_t x, uint16_t y, char *pText);		// 显示单个汉字
void 	LCD_DisplayText(uint16_t x, uint16_t y, char *pText) ;		// 显示字符串，包括中文和ASCII字符

//>>>>>	显示整数或小数
void  LCD_ShowNumMode(uint8_t mode);		// 设置显示模式，多余位填充空格还是填充0
void  LCD_DisplayNumber( uint16_t x, uint16_t y, int32_t number,uint8_t len) ;					// 显示整数
void  LCD_DisplayDecimals( uint16_t x, uint16_t y, double number,uint8_t len,uint8_t decs);	// 显示小数

//>>>>>	绘制图片

void 	LCD_DrawImage(uint16_t x,uint16_t y,uint16_t width,uint16_t height,const uint8_t *pImage)  ;

//>>>>>	2D图形绘制函数

void  LCD_DrawPoint(uint16_t x,uint16_t y,uint32_t color);   								//画点
uint32_t 	LCD_ReadPoint(uint16_t x,uint16_t y);												//读点
void  LCD_DrawLine(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);					//画线
void  LCD_DrawRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//画矩形
void  LCD_DrawCircle(uint16_t x, uint16_t y, uint16_t r);									//画圆
void  LCD_DrawEllipse(int x, int y, int r1, int r2);											//画椭圆

//>>>>>	区域填充函数

void  LCD_FillRect(uint16_t x, uint16_t y, uint16_t width, uint16_t height);			//填充矩形
void  LCD_FillCircle(uint16_t x, uint16_t y, uint16_t r);									//填充圆

/*-------------------------------------------------------- LCD相关参数 -------------------------------------------------------*/

#define HBP  80	// 根据屏幕的手册进行设置
#define VBP  40
#define HSW  1
#define VSW  1
#define HFP  200
#define VFP  22

#define LCD_Width     	800				//	LCD的像素长度
#define LCD_Height    	480				//	LCD的像素宽度
#define LCD_MemoryAdd   0xD0000000 		//	显存的起始地址  


	
#if ( ColorMode_0 == LTDC_PIXEL_FORMAT_RGB565 || ColorMode_0 == LTDC_PIXEL_FORMAT_ARGB1555 || ColorMode_0 ==LTDC_PIXEL_FORMAT_ARGB4444 )
	#define BytesPerPixel_0		2		//16位色模式每个像素占2字节
#elif ColorMode_0 == LTDC_PIXEL_FORMAT_RGB888
	#define BytesPerPixel_0		3		//24位色模式每个像素占3字节
#else
	#define BytesPerPixel_0		4		//32位色模式每个像素占4字节
#endif	

#if LCD_NUM_LAYERS == 2

	#if ( ColorMode_1 == LTDC_PIXEL_FORMAT_RGB565 || ColorMode_1 == LTDC_PIXEL_FORMAT_ARGB1555 || ColorMode_1 == LTDC_PIXEL_FORMAT_ARGB4444 )
		#define BytesPerPixel_1		2	//16位色模式每个像素占2字节
	#elif ColorMode_1 == LTDC_PIXEL_FORMAT_RGB888	
		#define BytesPerPixel_1		3	//24位色模式每个像素占3字节
	#else	
		#define BytesPerPixel_1		4	//32位色模式每个像素占4字节
	#endif	

	#define LCD_MemoryAdd_OFFSET   LCD_Width * LCD_Height * BytesPerPixel_0 	 //第二层的显存的偏移地址 

#endif

/*-------------------------------- LCD低2位引脚，用于旧面板识别 ------------------------------------------*/
//// 旧面板识别修改，此处只对旧款5寸屏有用，也就是 RGB050M1  V1.0的版本
//// 旧款5寸屏只引出了18位的接口，低2位数据脚直接接地，如果对应的引脚为低电平则可以确定接入的是旧款5寸屏
//// 其它面板无需理会此处，用户移植的时候可以直接删掉

#define  LTDC_R0_PIN									GPIO_PIN_15								
#define	LTDC_R0_PORT								GPIOI									

#define  LTDC_R1_PIN									GPIO_PIN_0								
#define	LTDC_R1_PORT								GPIOJ		

#define  LTDC_G0_PIN									GPIO_PIN_7								
#define	LTDC_G0_PORT								GPIOJ									

#define  LTDC_G1_PIN									GPIO_PIN_8								
#define	LTDC_G1_PORT								GPIOJ		

#define  LTDC_B0_PIN									GPIO_PIN_12								
#define	LTDC_B0_PORT								GPIOJ									

#define  LTDC_B1_PIN									GPIO_PIN_13								
#define	LTDC_B1_PORT								GPIOJ		

/*----------------------------------------- LCD背光引脚 --------------------------------------*/

#define  LTDC_Black_PIN									GPIO_PIN_13								
#define	LTDC_Black_PORT								GPIOD									
#define 	GPIO_LTDC_Black_CLK_ENABLE        	   __HAL_RCC_GPIOD_CLK_ENABLE()	 	



  
#endif 



