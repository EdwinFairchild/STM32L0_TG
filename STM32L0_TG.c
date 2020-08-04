#include <stm32l0xx_ll_bus.h>
#include <stm32l0xx_ll_gpio.h>
#include <stm32l0xx_ll_utils.h>
#include <stm32l0xx_ll_spi.h>
#include <stm32l0xx_ll_usart.h>
#include <stm32l0xx_ll_adc.h>

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
	uint8_t  xv, yv = 0;
	//---------------------| ADC
	//ADc calibrate
	MX_ADC_Init();

	LL_ADC_REG_StartConversion(ADC1);
	delayMS(15);
	ST7735_printMsg(10, 10, "%d", LL_ADC_REG_ReadConversionData12(ADC1));
	//rand() % (max_number + 1 - minimum_number) + minimum_number
	uint8_t num , numPrev = 0;
	for (;;)
	{
		//ST7735_FillScreen(ST7735_BLACK);
		LL_ADC_REG_StartConversion(ADC1);
		delayMS(10);
		num = map(LL_ADC_REG_ReadConversionData12(ADC1), 0, 4095, 0, 160);
		ST7735_printMsg(10, 10, " %d  ", num);
		if (num != numPrev) //if at a new spot
		{
			
			ST7735_DrawCircle(numPrev, 50,10, ST7735_BLACK);
			numPrev = num;
			ST7735_DrawCircle(num, 50, 10, ST7735_YELLOW);
		}
		
		
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

	/* USER CODE BEGIN ADC_Init 0 */

	/* USER CODE END ADC_Init 0 */

	LL_ADC_REG_InitTypeDef ADC_REG_InitStruct = { 0 };
	LL_ADC_InitTypeDef ADC_InitStruct = { 0 };

	LL_GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	/* Peripheral clock enable */
	LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_ADC1);
  
	LL_IOP_GRP1_EnableClock(LL_IOP_GRP1_PERIPH_GPIOB);
	/**ADC GPIO Configuration  
	PB1   ------> ADC_IN9 
	*/
	GPIO_InitStruct.Pin = LL_GPIO_PIN_1;
	GPIO_InitStruct.Mode = LL_GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
	LL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* USER CODE BEGIN ADC_Init 1 */

	/* USER CODE END ADC_Init 1 */
	/** Configure Regular Channel 
	*/
	LL_ADC_REG_SetSequencerChAdd(ADC1, LL_ADC_CHANNEL_9);
	/** Common config 
	*/
	ADC_REG_InitStruct.TriggerSource = LL_ADC_REG_TRIG_SOFTWARE;
	ADC_REG_InitStruct.SequencerDiscont = LL_ADC_REG_SEQ_DISCONT_DISABLE;
	ADC_REG_InitStruct.ContinuousMode = LL_ADC_REG_CONV_SINGLE;
	ADC_REG_InitStruct.DMATransfer = LL_ADC_REG_DMA_TRANSFER_NONE;
	ADC_REG_InitStruct.Overrun = LL_ADC_REG_OVR_DATA_PRESERVED;
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
	LL_ADC_Enable(ADC1);
	/* USER CODE BEGIN ADC_Init 2 */

	/* USER CODE END ADC_Init 2 */

}