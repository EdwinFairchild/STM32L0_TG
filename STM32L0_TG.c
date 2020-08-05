#include <stm32l0xx_ll_bus.h>
#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_ll_utils.h>
#include <stm32l0xx_ll_spi.h>
#include <stm32l0xx_ll_usart.h>
#include <stm32l0xx_ll_adc.h>
#include <stm32l0xx_ll_dma.h>

#include <math.h>
//---------------| CL |------------------------
#include  "CL_CONFIG.h"
#include "CL_printMsg.h"
#include "CL_delay.h"
#include "CL_systemClockUpdate.h"

//------------- | ST7735     | ----------------
#include "st7735_cfg.h"
#include "st7735.h"
#include "testimg.h"
#include "testimg2.h"

//---------------| GLOBALS |-------------------
uint16_t posVal[2] ,xVal_pre, yVal_pre = 0;
//---------------| Prototypes |----------------
void init_CL(void);
void initLed(void);
void spiInit(void);
void spiSend(uint8_t *data, uint8_t len);
void spiSend16(uint16_t *data, uint32_t len);
void tft_gpio_init(void); 
void crayLoop(void)
{
	ST7735_FillScreen(ST7735_BLACK);

	for (int x = 0; x < ST7735_WIDTH; x++) {
		ST7735_DrawPixel(x, 0, ST7735_RED);
		ST7735_DrawPixel(x, ST7735_HEIGHT - 1, ST7735_RED);
	}

	for (int y = 0; y < ST7735_HEIGHT; y++) {
		ST7735_DrawPixel(0, y, ST7735_RED);
		ST7735_DrawPixel(ST7735_WIDTH - 1, y, ST7735_RED);
	}

	//delayMS(3000);

	// Check fonts
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "Font_7x10, red on black, lorem ipsum dolor sit amet", Font_7x10, ST7735_RED, ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10, "Font_11x18, green, lorem ipsum", Font_11x18, ST7735_GREEN, ST7735_BLACK);
	ST7735_WriteString(0, 3 * 10 + 3 * 18, "Font_16x26", Font_16x26, ST7735_BLUE, ST7735_BLACK);
	//delayMS(2000);

	// Check colors
	ST7735_FillScreen(ST7735_BLACK);
	ST7735_WriteString(0, 0, "BLACK", Font_11x18, ST7735_WHITE, ST7735_BLACK);
	//delayMS(500);

	ST7735_FillScreen(ST7735_BLUE);
	ST7735_WriteString(0, 0, "BLUE", Font_11x18, ST7735_BLACK, ST7735_BLUE);
	//delayMS(500);

	ST7735_FillScreen(ST7735_RED);
	ST7735_WriteString(0, 0, "RED", Font_11x18, ST7735_BLACK, ST7735_RED);
	//delayMS(500);

	ST7735_FillScreen(ST7735_GREEN);
	ST7735_WriteString(0, 0, "GREEN", Font_11x18, ST7735_BLACK, ST7735_GREEN);
	//delayMS(500);

	ST7735_FillScreen(ST7735_CYAN);
	ST7735_WriteString(0, 0, "CYAN", Font_11x18, ST7735_BLACK, ST7735_CYAN);
	//delayMS(500);

	ST7735_FillScreen(ST7735_MAGENTA);
	ST7735_WriteString(0, 0, "MAGENTA", Font_11x18, ST7735_BLACK, ST7735_MAGENTA);
	//delayMS(500);

	ST7735_FillScreen(ST7735_YELLOW);
	ST7735_WriteString(0, 0, "YELLOW", Font_11x18, ST7735_BLACK, ST7735_YELLOW);
	//delayMS(500);

	ST7735_FillScreen(ST7735_WHITE);
	ST7735_WriteString(0, 0, "WHITE", Font_11x18, ST7735_BLACK, ST7735_WHITE);
	//delayMS(500);
}
void MX_ADC_Init(void);
long map(long x, long in_min, long in_max, long out_min, long out_max);
void MX_DMA_Init(void);
void ADC_DMA_ini(void);
void drawAnimation(uint16_t x, uint16_t y);
int main(void)
{
	init_CL();
	initLed();
	tft_gpio_init();
	spiInit();
	ST7735_Init();
	ST7735_SetRotation(3);
	ST7735_FillScreen(ST7735_BLACK);
	//ST7735_printMsg(10, 10, "hello World %d" , 56);
	uint16_t  xV, yV , xPre , yPre = 0;
	//---------------------| ADC

	ADC_DMA_ini();
	LL_ADC_REG_StartConversion(ADC1);
	
	uint16_t  num , numPrev = 0;
	for (;;)
	{
		//ST7735_FillScreen(ST7735_BLACK);
		// LL_ADC_REG_StartConversion(ADC1);
		
		delayMS(10);
		xV = map(posVal[0] , 0 , 1023,0,160);
		yV = map(posVal[1], 0, 1023, 0, 128);
		ST7735_printMsg(40, 10, "( %d , %d ) ", posVal[0] , posVal[1]);
		if (xV != xPre || yV != yPre) //if at a new spot
			{                                               
			
				//ST7735_DrawCircle(xPre, yPre, 10, ST7735_BLACK);
				drawAnimation(xV, yV);
				xPre = xV;
				//ST7735_DrawCircle(xV, yV, 10, ST7735_YELLOW);
				yPre = yV;
			}
		else
			drawAnimation(xPre, yPre);
		
		
		
	}
}//-------------------------------------------------------------------
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void init_CL(void)
{
	setClockTo32Mhz();	
	CL_delay_init();
	CL_printMsg_init_Default(false);	
	CL_printMsg("\nHello\n");
}//-------------------------------------------------------------------
void initLed(void)
{
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_3, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_3, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_3, LL_GPIO_SPEED_FREQ_LOW);
	
}//-------------------------------------------------------------------
void spiInit(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SPI1);
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);

	LL_GPIO_InitTypeDef spiPort;
	LL_GPIO_StructInit(&spiPort);
	
	spiPort.OutputType  = LL_GPIO_OUTPUT_PUSHPULL;
	spiPort.Mode		= LL_GPIO_MODE_ALTERNATE;
	spiPort.Pin			= LL_GPIO_PIN_5;
	spiPort.Speed		= LL_GPIO_SPEED_FREQ_HIGH;
	
	LL_GPIO_Init(GPIOA, &spiPort);
	//GPIOA->AFR[0] = 5;
	
	spiPort.Pin = LL_GPIO_PIN_7;
	LL_GPIO_Init(GPIOA, &spiPort);

	// config SPI
	LL_SPI_InitTypeDef mySPI;
	LL_SPI_StructInit(&mySPI); 
	

	
	
	mySPI.BaudRate			= LL_SPI_BAUDRATEPRESCALER_DIV2;    //40Mbits/s
	mySPI.Mode				= LL_SPI_MODE_MASTER;
	mySPI.TransferDirection = LL_SPI_HALF_DUPLEX_TX;
	mySPI.NSS				= LL_SPI_NSS_SOFT;
	mySPI.DataWidth			= LL_SPI_DATAWIDTH_8BIT;
	LL_SPI_Init(SPI1, &mySPI);
	LL_SPI_Enable(SPI1);	
}//-------------------------------------------------------------------
void spiSend(uint8_t *data, uint8_t len)
{
	uint8_t volatile *spidr = ((__IO uint8_t *)&SPI1->DR);

	while (len > 0)
	{
		//while (!(SPI1->SR&SPI_SR_TXE)) 
		//;
		len--;
		//*spidr = (uint8_t)*data++;
		LL_SPI_TransmitData8(SPI1, *data++);
	}	
	while ((SPI1->SR&SPI_SR_BSY)) ;
		
}//-------------------------------------------------------------------
void spiSend16(uint16_t *data, uint32_t len)
{	
	uint16_t volatile *spidr = ((__IO uint16_t *)&SPI1->DR);

	while (len > 0)
	{
		while (!(SPI1->SR&SPI_SR_TXE)) ;
		len--;
		*spidr = *data++;
		//LL_SPI_TransmitData16(SPI1,  (uint16_t)*data++);
	}	
	while ((SPI1->SR&SPI_SR_BSY)) ;		
}//-------------------------------------------------------------------
void tft_gpio_init(void)
{
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOA);
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4);
	
	LL_GPIO_InitTypeDef spiPort;
	LL_GPIO_StructInit(&spiPort);
	
	spiPort.Mode 	= LL_GPIO_MODE_OUTPUT;
	spiPort.Speed	= LL_GPIO_SPEED_FREQ_HIGH;
	spiPort.Pin			= LL_GPIO_PIN_1 | LL_GPIO_PIN_2 | LL_GPIO_PIN_3 | LL_GPIO_PIN_4; 
	spiPort.OutputType  = LL_GPIO_OUTPUT_PUSHPULL;
	
	LL_GPIO_Init(GPIOA, &spiPort);
}
void MX_ADC_Init(void)
{



	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };
	LL_ADC_InitTypeDef ADC_InitStruct = { 0 };
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);  
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	/**ADC GPIO Configuration  
	PB0   ------> ADC_IN8
	PB1   ------> ADC_IN9 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	
	LL_DMA_SetPeriphRequest(DMA1, LL_DMA_CHANNEL_1, LL_DMA_REQUEST_0);
	LL_DMA_SetDataTransferDirection(DMA1, LL_DMA_CHANNEL_1, LL_DMA_DIRECTION_PERIPH_TO_MEMORY);
	LL_DMA_SetChannelPriorityLevel(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PRIORITY_LOW);
	LL_DMA_SetMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MODE_CIRCULAR);
	LL_DMA_SetPeriphIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PERIPH_NOINCREMENT);
	LL_DMA_SetMemoryIncMode(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MEMORY_INCREMENT);
	LL_DMA_SetPeriphSize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_PDATAALIGN_HALFWORD);
	LL_DMA_SetMemorySize(DMA1, LL_DMA_CHANNEL_1, LL_DMA_MDATAALIGN_HALFWORD);	
	LL_DMA_SetDataLength(DMA1, LL_DMA_CHANNEL_1, 1);	
	LL_DMA_SetMemoryAddress(DMA1, LL_DMA_CHANNEL_1 , posVal[0]);	
	LL_DMA_SetPeriphAddress(DMA1, LL_DMA_CHANNEL_1, (uint32_t)&ADC1->DR);	

	LL_DMA_EnableChannel(DMA1, LL_DMA_CHANNEL_1);
	
	
	
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_8);
	/** Configure Regular Channel 
	*/
	//LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_9);
	
	
	/** Common config 
	*/
	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_CONTINUOUS;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_UNLIMITED;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_OVERWRITTEN;
	LL_ADC_REG_Init(ADC1, &ADC_REG_InitStruct);
	LL_ADC_SetSamplingTimeCommonChannels(ADC1, LL_ADC_SAMPLINGTIME_1CYCLE_5);
	LL_ADC_SetOverSamplingScope(ADC1, LL_ADC_OVS_DISABLE);
	LL_ADC_REG_SetSequencerScanDirection(ADC1, LL_ADC_REG_SEQ_SCAN_DIR_FORWARD);
	LL_ADC_SetCommonFrequencyMode(__LL_ADC_COMMON_INSTANCE(ADC1), LL_ADC_CLOCK_FREQ_MODE_HIGH);
	LL_ADC_DisableIT_EOC(ADC1);
	LL_ADC_DisableIT_EOS(ADC1);
	LL_ADC_EnableInternalRegulator(ADC1);
	ADC_InitStruct.Clock = LL_ADC_CLOCK_SYNC_PCLK_DIV2;
	ADC_InitStruct.Resolution = LL_ADC_RESOLUTION_12B;
	ADC_InitStruct.DataAlignment = LL_ADC_DATA_ALIGN_RIGHT;
	ADC_InitStruct.LowPowerMode = LL_ADC_LP_MODE_NONE;
	LL_ADC_Init(ADC1, &ADC_InitStruct);
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */


}

void MX_DMA_Init(void) 
{

	/* Init with LL driver */
	/* DMA controller clock enable */
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

void ADC_DMA_ini(void)
{
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);  
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1);
	
	//---do i need irq?
	NVIC_SetPriority(DMA1_Channel1_IRQn, 0);
	NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	
	/**ADC GPIO Configuration  
	PB0   ------> ADC_IN8
	PB1   ------> ADC_IN9 
	*/
	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	GPIO_InitStruct.Pin = LL_GPIO_PIN_0;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);
	
	
	ADC1->CR |= ADC_CR_ADVREGEN; //enable voltage reg on ADC
	//continous conversion | 10 bit resolution | DMA circular mode | DMA enabled
	ADC1->CFGR1 |= ADC_CFGR1_CONT | (1<<ADC_CFGR1_RES_Pos) | ADC_CFGR1_DMACFG | ADC_CFGR1_DMAEN ;  //add dma here 
	//pclk / 2 
	ADC1->CFGR2 |= (1<<ADC_CFGR2_CKMODE_Pos);
	
	//adc channel 8 enable  = PORTb 0
	ADC1->CHSELR = ADC_CHSELR_CHSEL8 | ADC_CHSELR_CHSEL9;
	 
	
	
	DMA1_Channel1->CCR |= (1 << DMA_CCR_MSIZE_Pos) | (1 << DMA_CCR_PSIZE_Pos) | DMA_CCR_CIRC | DMA_CCR_MINC;
	DMA1_Channel1->CNDTR = 2;
	DMA1_Channel1->CPAR =(uint32_t) &ADC1->DR;
	DMA1_Channel1->CMAR = (uint32_t) posVal;
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	
	
	
	ADC1->ISR |= ADC_ISR_ADRDY; //write 1 to clear READY bit 
	ADC1->CR |= ADC_CR_ADEN; //Enable ADC when its enabled it will again set the READY bit
	if ((ADC1->CFGR1 & ADC_CFGR1_AUTOFF) == 0)
	{
		while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (3) */
		{
			/* For robust implementation, add here time-out management */
		}
	}
	
}
void DMA1_Channel1_IRQHandler(void)
{
	
	volatile int x = 0;
}
void drawAnimation(uint16_t x, uint16_t y)
{
	uint8_t del = 1;
	for (int i = 10; i >0; i--)
	{
		
		ST7735_DrawCircle(x, y, i, ST7735_WHITE);
		//delayMS(del);
    }
	
	for (int i = 1; i <11; i++)
	{
		
		ST7735_DrawCircle(x, y, i, ST7735_BLACK);
		//delayMS(del);
	}
	
		
}