/*
  Longan Nano LCD driver
*/

#include "lcd.h"
#include "oledfont.h"
#include "bmp.h"

u16 BACK_COLOR;	// Background color

void delay_ms(uint32_t count);

/*
  Function description: LCD serial data write function (write one byte)
  Entry data: dat: byte to be written
  Return value: None
  Note: DC must already be set (for data) or reset (for command)
*/
void LCD_Writ_Bus(u8 dat) 
{
#if SPI0_CFG == 1
	OLED_CS_Clr();

	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_TBE));
        spi_i2s_data_transmit(SPI0, dat);
	while(RESET == spi_i2s_flag_get(SPI0, SPI_FLAG_RBNE));
        spi_i2s_data_receive(SPI0);

	OLED_CS_Set();
#elif SPI0_CFG == 2
	spi_dma_enable(SPI0, SPI_DMA_TRANSMIT);
#else
	u8 i;
	OLED_CS_Clr();
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;
	}	
  OLED_CS_Set();	
#endif
}


/*
  Function description: LCD write 8-bit data
  Entry data: dat: data to be written
  Return value: None
*/
void LCD_WR_DATA8(u8 dat)
{
	OLED_DC_Set();  // Write data
	LCD_Writ_Bus(dat);
}


/*
  Function description: LCD write 16-bit data
  Entry data: dat: 16-bit data to be written
  Return value: None
*/
void LCD_WR_DATA(u16 dat)
{
	OLED_DC_Set();  // Write data
	LCD_Writ_Bus(dat>>8);
	LCD_Writ_Bus(dat);
}


/*
  Function description: LCD write 8-bit command
  Entry data: dat: command to be written
  Return value: None
*/
void LCD_WR_REG(u8 dat)
{
	OLED_DC_Clr();  // Write command
	LCD_Writ_Bus(dat);
}


/*
  Function description: Set start and end addresses
  Entry data: x1, x2 set the start and end column address
              y1, y2 set the start and end row address
  Return value: None
*/
void LCD_Address_Set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		LCD_WR_REG(0x2a);  // Column address setting
		LCD_WR_DATA(x1+26);
		LCD_WR_DATA(x2+26);
		LCD_WR_REG(0x2b);  // row address setting
		LCD_WR_DATA(y1+1);
		LCD_WR_DATA(y2+1);
		LCD_WR_REG(0x2c);  // Memory write
	}
	else if(USE_HORIZONTAL==1)
	{
		LCD_WR_REG(0x2a);  // Column address setting
		LCD_WR_DATA(x1+26);
		LCD_WR_DATA(x2+26);
		LCD_WR_REG(0x2b);  // row address setting
		LCD_WR_DATA(y1+1);
		LCD_WR_DATA(y2+1);
		LCD_WR_REG(0x2c);  // Memory write
	}
	else if(USE_HORIZONTAL==2)
	{
		LCD_WR_REG(0x2a);  // Column address setting
		LCD_WR_DATA(x1+1);
		LCD_WR_DATA(x2+1);
		LCD_WR_REG(0x2b);  // row address setting
		LCD_WR_DATA(y1+26);
		LCD_WR_DATA(y2+26);
		LCD_WR_REG(0x2c);  // Memory write
	}
	else
	{
		LCD_WR_REG(0x2a);  // Column address setting
		LCD_WR_DATA(x1+1);
		LCD_WR_DATA(x2+1);
		LCD_WR_REG(0x2b);  // row address setting
		LCD_WR_DATA(y1+26);
		LCD_WR_DATA(y2+26);
		LCD_WR_REG(0x2c);  // Memory write
	}
}

#if SPI0_CFG == 2
/*!
    \brief      configure the DMA peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void dma_config(void)
{
	dma_parameter_struct dma_init_struct;

    /* SPI0 transmit dma config:DMA0,DMA_CH2 */
    dma_deinit(DMA0, DMA_CH2);
    dma_struct_para_init(&dma_init_struct);

    dma_init_struct.periph_addr  = (uint32_t)&SPI_DATA(SPI0);
    dma_init_struct.memory_addr  = (uint32_t)image;
    dma_init_struct.direction    = DMA_MEMORY_TO_PERIPHERAL;
    dma_init_struct.memory_width = DMA_MEMORY_WIDTH_8BIT;
    dma_init_struct.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
    dma_init_struct.priority     = DMA_PRIORITY_LOW;
    dma_init_struct.number       = FRAME_SIZE;
    dma_init_struct.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
    dma_init_struct.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
    dma_init(DMA0, DMA_CH2, &dma_init_struct);
    /* configure DMA mode */
    dma_circulation_disable(DMA0, DMA_CH2);
    dma_memory_to_memory_disable(DMA0, DMA_CH2);
}
#endif

#if SPI0_CFG == 1
/*!
    \brief      configure the SPI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
void spi_config(void)
{
    spi_parameter_struct spi_init_struct;
    /* deinitilize SPI and the parameters */
    OLED_CS_Set();
    spi_struct_para_init(&spi_init_struct);

    /* SPI0 parameter config */
    spi_init_struct.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
    spi_init_struct.device_mode          = SPI_MASTER;
    spi_init_struct.frame_size           = SPI_FRAMESIZE_8BIT;
    spi_init_struct.clock_polarity_phase = SPI_CK_PL_HIGH_PH_2EDGE;
    spi_init_struct.nss                  = SPI_NSS_SOFT;
    spi_init_struct.prescale             = SPI_PSC_8;
    spi_init_struct.endian               = SPI_ENDIAN_MSB;
    spi_init(SPI0, &spi_init_struct);

	spi_crc_polynomial_set(SPI0,7);
	spi_enable(SPI0);
}
#endif

/*
  Function description: LCD initialization function
  Entry data: None
  Return value: None
*/
void Lcd_Init(void)
{
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);

#if SPI0_CFG == 1
 	rcu_periph_clock_enable(RCU_AF);
	rcu_periph_clock_enable(RCU_SPI0);
	/* SPI0 GPIO config: NSS/PA4, SCK/PA5, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 |GPIO_PIN_6| GPIO_PIN_7);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

	spi_config();

#elif SPI0_CFG == 2
    rcu_periph_clock_enable(RCU_DMA0);
    rcu_periph_clock_enable(RCU_SPI0);

	/* SPI0 GPIO config: NSS/PA4, SCK/PA5, MOSI/PA7 */
    gpio_init(GPIOA, GPIO_MODE_AF_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_7);
    /* SPI0 GPIO config: MISO/PA6 */
    gpio_init(GPIOA, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_6);

	dma_config();

	dma_channel_enable(DMA0,DMA_CH2);
#elif SPI0_CFG == 3
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_5 | GPIO_PIN_7);
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_2);

	gpio_bit_reset(GPIOA, GPIO_PIN_5 | GPIO_PIN_7);
	gpio_bit_reset(GPIOB, GPIO_PIN_2);
#endif

	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_50MHZ, GPIO_PIN_0 | GPIO_PIN_1);
	gpio_bit_reset(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);

	OLED_RST_Clr();
	delay_ms(200);
	OLED_RST_Set();
	delay_ms(20);
	OLED_BLK_Set();

	LCD_WR_REG(0x11); 
	delay_ms(100);

	LCD_WR_REG(0x21); 

	LCD_WR_REG(0xB1); 
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);

	LCD_WR_REG(0xB2);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);

	LCD_WR_REG(0xB3); 
	LCD_WR_DATA8(0x05);  
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x05);
	LCD_WR_DATA8(0x3A);
	LCD_WR_DATA8(0x3A);

	LCD_WR_REG(0xB4);
	LCD_WR_DATA8(0x03);

	LCD_WR_REG(0xC0);
	LCD_WR_DATA8(0x62);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x04);

	LCD_WR_REG(0xC1);
	LCD_WR_DATA8(0xC0);

	LCD_WR_REG(0xC2);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x00);

	LCD_WR_REG(0xC3);
	LCD_WR_DATA8(0x8D);
	LCD_WR_DATA8(0x6A);   

	LCD_WR_REG(0xC4);
	LCD_WR_DATA8(0x8D); 
	LCD_WR_DATA8(0xEE); 

	LCD_WR_REG(0xC5);  /*VCOM*/
	LCD_WR_DATA8(0x0E);    

	LCD_WR_REG(0xE0);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x07);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x12);
	LCD_WR_DATA8(0x27);
	LCD_WR_DATA8(0x37);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x10);

	LCD_WR_REG(0xE1);
	LCD_WR_DATA8(0x10);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x03);
	LCD_WR_DATA8(0x0F);
	LCD_WR_DATA8(0x06);
	LCD_WR_DATA8(0x02);
	LCD_WR_DATA8(0x08);
	LCD_WR_DATA8(0x0A);
	LCD_WR_DATA8(0x13);
	LCD_WR_DATA8(0x26);
	LCD_WR_DATA8(0x36);
	LCD_WR_DATA8(0x00);
	LCD_WR_DATA8(0x0D);
	LCD_WR_DATA8(0x0E);
	LCD_WR_DATA8(0x10);

	LCD_WR_REG(0x3A); 
	LCD_WR_DATA8(0x05);

	LCD_WR_REG(0x36);
	if(USE_HORIZONTAL==0)LCD_WR_DATA8(0x08);
	else if(USE_HORIZONTAL==1)LCD_WR_DATA8(0xC8);
	else if(USE_HORIZONTAL==2)LCD_WR_DATA8(0x78);
	else LCD_WR_DATA8(0xA8);

	LCD_WR_REG(0x29); 
} 


/*
  Function description: LCD clear screen function
  Entry data: Color: color to set as background
  Return value: None
*/
void LCD_Clear(u16 Color)
{
	u16 i,j;  	
	LCD_Address_Set(0,0,LCD_W-1,LCD_H-1);
    for(i=0;i<LCD_W;i++)
	  {
			for (j=0;j<LCD_H;j++)
				{
					LCD_WR_DATA(Color);
				}
	  }
}


/*
  Function description: LCD display Chinese characters
  Entry data:  x, y: start coordinates
              index: Chinese character number
               size: font size
  Return value: None
*/
void LCD_ShowChinese(u16 x,u16 y,u8 index,u8 size,u16 color)	
{  
	u8 i,j;
	u8 *temp,size1;
	if(size==16){temp=Hzk16;}               // Choose a font size
	if(size==32){temp=Hzk32;}
    LCD_Address_Set(x,y,x+size-1,y+size-1); // Set a region of Chinese characters
    size1=size*size/8;                      // The bytes occupied by a Chinese character
	temp+=index*size1;                      // Start of writing
	for(j=0;j<size1;j++)
	{
		for(i=0;i<8;i++)
		{
		 	if((*temp&(1<<i))!=0) {         // Read from the lower bit of the data
				LCD_WR_DATA(color);         // set dot color
			}
			else
			{
				LCD_WR_DATA(BACK_COLOR);    // set dot color to background
			}
		}
		temp++;
	 }
}


/*
  Function description: LCD display Chinese characters
  Entry data: x, y: start coordinates
  Return value: None
*/
void LCD_DrawPoint(u16 x,u16 y,u16 color)
{
	LCD_Address_Set(x,y,x,y); // Set cursor position
	LCD_WR_DATA(color);
} 


/*
  Function description: LCD draws a large dot
  Entry data: x, y: start coordinates
  Return value: None
*/
void LCD_DrawPoint_big(u16 x,u16 y,u16 color)
{
	LCD_Fill(x-1,y-1,x+1,y+1,color);
} 


/*
  Function description: fill color in the specified area
  Entry data: xsta, ysta:  start coordinates
              xend, yend:  end coordinates
  Return value: None
*/
void LCD_Fill(u16 xsta,u16 ysta,u16 xend,u16 yend,u16 color)
{          
	u16 i,j; 
	LCD_Address_Set(xsta,ysta,xend,yend);          //Set cursor position
	for(i=ysta;i<=yend;i++)
	{													   	 	
		for(j=xsta;j<=xend;j++)LCD_WR_DATA(color); //Set cursor position
	} 					  	    
}


/*
  Function description: draw a line
  Entry data: x1, y1:  start coordinates
              x2, y2:  end coordinates
  Return value: None
******************************************************************************/
void LCD_DrawLine(u16 x1,u16 y1,u16 x2,u16 y2,u16 color)
{
	u16 t; 
	int xerr=0,yerr=0,delta_x,delta_y,distance;
	int incx,incy,uRow,uCol;
	delta_x=x2-x1;                       // Calculate coordinate increments
	delta_y=y2-y1;
	uRow=x1;                             // Coordinates of starting point of drawing
	uCol=y1;
	if(delta_x>0)incx=1;                 // Set single step direction
	else if (delta_x==0)incx=0;          // Vertical line
	else {incx=-1;delta_x=-delta_x;}
	if(delta_y>0)incy=1;
	else if (delta_y==0)incy=0;          // Horizontal line
	else {incy=-1;delta_y=-delta_x;}
	if(delta_x>delta_y)distance=delta_x; // Pick basic incremental axis
	else distance=delta_y;
	for(t=0;t<distance+1;t++)
	{
		LCD_DrawPoint(uRow,uCol,color);  // Dot
		xerr+=delta_x;
		yerr+=delta_y;
		if(xerr>distance)
		{
			xerr-=distance;
			uRow+=incx;
		}
		if(yerr>distance)
		{
			yerr-=distance;
			uCol+=incy;
		}
	}
}


/*
  Function description: draw a rectangle
  Entry data: x1, y1:  start coordinates
              x2, y2:  end coordinates
  Return value: None
*/
void LCD_DrawRectangle(u16 x1, u16 y1, u16 x2, u16 y2,u16 color)
{
	LCD_DrawLine(x1,y1,x2,y1,color);
	LCD_DrawLine(x1,y1,x1,y2,color);
	LCD_DrawLine(x1,y2,x2,y2,color);
	LCD_DrawLine(x2,y1,x2,y2,color);
}


/*
  Function description: draw circle
  Entry data: x0, y0:  center coordinates
                   r:  radius
  Return value: None
*/
void Draw_Circle(u16 x0,u16 y0,u8 r,u16 color)
{
	int a,b;
	// int di;
	a=0;b=r;	  
	while(a<=b)
	{
		LCD_DrawPoint(x0-b,y0-a,color);  //3           
		LCD_DrawPoint(x0+b,y0-a,color);  //0           
		LCD_DrawPoint(x0-a,y0+b,color);  //1                
		LCD_DrawPoint(x0-a,y0-b,color);  //2             
		LCD_DrawPoint(x0+b,y0+a,color);  //4               
		LCD_DrawPoint(x0+a,y0-b,color);  //5
		LCD_DrawPoint(x0+a,y0+b,color);  //6 
		LCD_DrawPoint(x0-b,y0+a,color);  //7
		a++;
		if((a*a+b*b)>(r*r)) // Determine whether the points to be drawn are too far away
		{
			b--;
		}
	}
}


/*
  Function description: display characters
  Entry data: x, y:  start point coordinates
               num:  characters to display
              mode:  1: transparent mode
                     0: non-transparent mode
  Return value: None
*/
void LCD_ShowChar(u16 x,u16 y,u8 num,u8 mode,u16 color)
{
    u8 temp;
    u8 pos,t;
	  u16 x0=x;    
    if(x>LCD_W-8 || y>LCD_H-16)return;	// Outside of display area
	num=num-' ';                        // Get offset value
	LCD_Address_Set(x,y,x+8-1,y+16-1);  // Set cursor position
	if(!mode)
	{
		// non-trasparent mode
		for(pos=0;pos<16;pos++)
		{ 
			temp=asc2_1608[(u16)num*16+pos];  // load 1608 font character
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)LCD_WR_DATA(color);
				else LCD_WR_DATA(BACK_COLOR);
				temp>>=1;
				x++;
		    }
			x=x0;
			y++;
		}	
	}else
	{
		// Transparent mode
		for(pos=0;pos<16;pos++)
		{
		    temp=asc2_1608[(u16)num*16+pos]; // load 1608 font character
			for(t=0;t<8;t++)
		    {                 
		        if(temp&0x01)LCD_DrawPoint(x+t,y+pos,color); //Draw a dot
		        temp>>=1; 
		    }
		}
	}   	   	 	  
}


/*
  Function description: display string
  Entry data: x, y:  start point coordinates
                *p:  string start address
  Return value: None
  Note: If character position is outside the display area
        display is cleared in red color and the character
		position is set to (0,0)
*/
void LCD_ShowString(u16 x,u16 y,const u8 *p,u16 color)
{         
    while(*p!='\0')
    {       
        if(x>LCD_W-8){x=0;y+=16;}
        if(y>LCD_H-16){y=x=0;LCD_Clear(RED);}
        LCD_ShowChar(x,y,*p,0,color);
        x+=8;
        p++;
    }  
}


/*
  Function description: display string
  Entry data: x, y:  start point coordinates
                *p:  string start address
              mode:  1: transparent mode
                     0: non-transparent mode
  Return value: None
  Note: If character position is outside the display area
        the character is not displayed
*/
//----------------------------------------------------------
void LCD_ShowStr(u16 x,u16 y,const u8 *p,u16 color, u8 mode)
{         
    while(*p!='\0')
    {       
		if (*p == '\r') {
			// clear to the end of line
			while (x < LCD_W) {
		        LCD_ShowChar(x,y,' ',mode,color);
				x += 8;
			}
			break;
		}
		else if (*p == '\n') {
			// clear to the end of screen
			while (y < LCD_H) {
		        LCD_ShowChar(x,y,' ',mode,color);
				if (x > (LCD_W-8)) {x=0; y+=16;}
				x += 8;
			}
			break;
		}
        if (x > (LCD_W-8)) {x=0;y+=16;}
        if (y > (LCD_H-16)) break;
        LCD_ShowChar(x,y,*p,mode,color);
        x+=8;
        p++;
    }  
}


/*
  Function description: Calculate m^n
  Entry data: m: base
              n: exponent
  Return value: m^n
*/
u32 mypow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}


/*
  Function description: display 16-bit integer numbers
  Entry data: x, y:  start point coordinates
               num:  number to display
               len:  number of digits to display
  Return value: None
*/
void LCD_ShowNum(u16 x,u16 y,u16 num,u8 len,u16 color)
{         	
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/mypow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				LCD_ShowChar(x+8*t,y,' ',0,color);
				continue;
			}else enshow=1; 
		 	 
		}
	 	LCD_ShowChar(x+8*t,y,temp+48,0,color); 
	}
} 


/*
  Function description: display float number with 2 decimal places
  Entry data: x, y:  start point coordinates
               num:  float number to display
               len:  number of digits to display
  Return value: None
*/
void LCD_ShowNum1(u16 x,u16 y,float num,u8 len,u16 color)
{         	
	u8 t,temp;
	// u8 enshow=0;
	u16 num1;
	num1=num*100;
	for(t=0;t<len;t++)
	{
		temp=(num1/mypow(10,len-t-1))%10;
		if(t==(len-2))
		{
			LCD_ShowChar(x+8*(len-2),y,'.',0,color);
			t++;
			len+=1;
		}
	 	LCD_ShowChar(x+8*t,y,temp+48,0,color);
	}
}


/*
  Function description: display the image 
  Entry data: x1, y1:  start coordinates
              x2, y2:  end coordinates
              *image:  pointer to image buffer
  Return value: None
  Note: image buffere contains 16-bit pixel colors
        and its size must be (x2-x1+1) * (y2-y1+1) * 2
  */
void LCD_ShowPicture(u16 x1, u16 y1, u16 x2, u16 y2, u8 *image)
{
	int i;
	int size = (x2-x1+1) * (y2-y1+1) * 2;
	LCD_Address_Set(x1,y1,x2,y2);
	for(i=0;i<size;i++)
	{ 	
		LCD_WR_DATA8(image[i]);
	}			
}

/*
  Function description: display 160x26 logo provided in bmp.h file
  Entry data: y: y position (0 ~ 53)
  Return value: None
*/
void LCD_ShowLogo(u16 y)
{
	int i;
	if ((y+26) >= 80) return;
	LCD_Address_Set(0,y,159,y+25);
	for(i=0;i<4160;i++)
	{
		LCD_WR_DATA8((u8)(logo_bmp[i] >> 8));
		LCD_WR_DATA8((u8)logo_bmp[i]);
	}			
}
