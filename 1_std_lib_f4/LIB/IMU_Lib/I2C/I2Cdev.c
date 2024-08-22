/*
	*
  ******************************************************************************
  * @file    myiic.c
  * @author  Sercan ERAT
  * @version V1.0.0
  ******************************************************************************
	*
*/

/*

SCL -> PB8
SDA -> PB9

*/

#include "I2Cdev.h"
int DMA_Tog = 0;
int read_finish(void);
void reset_finish(void);

void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){
	
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)); 
	I2C_GenerateSTART(I2Cx, ENABLE);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	I2C_Send7bitAddress(I2Cx, address, direction);
	   
	if(direction == I2C_Direction_Transmitter){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	}
	else if(direction == I2C_Direction_Receiver){
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));
	}
}

void I2C_write(I2C_TypeDef* I2Cx, uint8_t data) {
	
	I2C_SendData(I2Cx, data);
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	
	uint8_t data;
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	data = I2C_ReceiveData(I2Cx);
	return data;
}

uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	
	uint8_t data; 
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	data = I2C_ReceiveData(I2Cx);
	return data;
}

void I2C_stop(I2C_TypeDef* I2Cx){
	
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C_readByte(uint8_t slave_address, uint8_t readAddr, uint8_t *data){
	if (DMA_Tog == 1)
	{
		I2C_ReadDMA(slave_address, readAddr, data, 1);
		while(read_finish()==0){;}
		reset_finish();
	}
	else
	{
		I2C_start(I2C2, slave_address, I2C_Direction_Transmitter);
		I2C_write(I2C2, readAddr); 
		I2C_stop(I2C2);
		I2C_start(I2C2, slave_address, I2C_Direction_Receiver); 
		*data = I2C_read_nack(I2C2);
	}		
}

void I2C_writeByte(uint8_t slave_address, uint8_t writeAddr, uint8_t data){
	I2C_start(I2C2, slave_address, I2C_Direction_Transmitter); 
	I2C_write(I2C2, writeAddr);
  I2C_write(I2C2, data);
  I2C_stop(I2C2);
}

void I2C_readBytes(uint8_t slave_address, uint8_t readAddr, uint8_t length, uint8_t *data){
	if (DMA_Tog == 1)
	{
		I2C_ReadDMA(slave_address, readAddr, data, length);
	}
	else
	{
		I2C_start(I2C2, slave_address, I2C_Direction_Transmitter); 	
		I2C_write(I2C2, readAddr); 																	
		I2C_stop(I2C2); 																						
		I2C_start(I2C2, slave_address, I2C_Direction_Receiver); 		
		while(length){
			if(length==1)
				*data = I2C_read_nack(I2C2);
			else
				*data = I2C_read_ack(I2C2);																
		
			data++;
			length--;
	}
	}
}

void I2C_writeBytes(uint8_t slave_address, uint8_t writeAddr, uint8_t length, uint8_t *data){
	int i=0;
	I2C_start(I2C2, slave_address, I2C_Direction_Transmitter);
	I2C_write(I2C2, writeAddr);
	for(i=0; i<length; i++){	
		I2C_write(I2C2, data[i]);
  }
  I2C_stop(I2C2);
}

void I2C_readBit(uint8_t slave_address, uint8_t regAddr, uint8_t bitNum, uint8_t *data){
	
  uint8_t tmp;
  I2C_readByte(slave_address, regAddr, &tmp);  
  *data = tmp & (1 << bitNum);
}

void I2C_readBits(uint8_t slave_address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data){
	
  // 01101001 read byte
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  //    010   masked
  //   -> 010 shifted
  uint8_t mask,tmp;
  I2C_readByte(slave_address, regAddr, &tmp); 
  mask = ((1 << length) - 1) << (bitStart - length + 1);
  tmp &= mask;
  tmp >>= (bitStart - length + 1);
	*data = tmp;
}

void I2C_writeBit(uint8_t slave_address, uint8_t regAddr, uint8_t bitNum, uint8_t data){
  
	uint8_t tmp;
  I2C_readByte(slave_address, regAddr, &tmp);  
  tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
  I2C_writeByte(slave_address,regAddr,tmp); 
}

void I2C_writeBits(uint8_t slave_address, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data){
	
  //      010 value to write
  // 76543210 bit numbers
  //    xxx   args: bitStart=4, length=3
  // 00011100 mask byte
  // 10101111 original value (sample)
  // 10100011 original & ~mask
  // 10101011 masked | value
  uint8_t tmp,mask;
  I2C_readByte(slave_address, regAddr, &tmp);
  mask = ((1 << length) - 1) << (bitStart - length + 1);
  data <<= (bitStart - length + 1); 
  data &= mask; 
  tmp &= ~(mask); 
  tmp |= data; 
  I2C_writeByte(slave_address, regAddr, tmp);	
}

void I2C_writeWord(uint8_t slave_address, uint8_t writeAddr, uint16_t data){
	I2C_start(I2C2, slave_address, I2C_Direction_Transmitter); 
	I2C_write(I2C2, writeAddr);         
  I2C_write(I2C2, (data >> 8));      // send MSB
	I2C_write(I2C2, (data << 8));			 // send LSB
  I2C_stop(I2C2);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PB10_ALT (1<<21)
#define PB11_ALT (1<<23)
#define I2C_AF4 (0x04)
#define Channel_7 (7<<25)

void I2C2_Init(void)
{
	RCC->AHB1ENR|=RCC_AHB1ENR_GPIOBEN|RCC_AHB1ENR_GPIODEN; //enable gpiob clock
	RCC->APB1ENR|=RCC_APB1ENR_I2C2EN; //enable I2C2 clock
	GPIOB->MODER|=PB10_ALT|PB11_ALT; //set PB10 and PB11 to alternative function
	GPIOB->AFR[1]|=(I2C_AF4<<8)|(I2C_AF4<<12);
	GPIOB->OTYPER|=(0x1UL<<10U)|(0x1UL<<11U);
	GPIOD->MODER|=(1<<24)|(1<<28);
	GPIOD->OTYPER&=~((1<<12)|(1<<14));
	GPIOD->OSPEEDR|=(1<<25)|(1<<29);
	GPIOD->PUPDR&=~((1<<24)|(1<<25)|(1<<28)|(1<<29));
	I2C2->CR1=I2C_CR1_SWRST;//reset i2c
	I2C2->CR1&=~I2C_CR1_SWRST;// release reset i2c	
	I2C2->CR1 &=~ I2C_CR1_NOSTRETCH;//disable clock strech
	I2C2->CR1 &= ~I2C_CR1_ENGC;
	I2C2->CR2 |= I2C_CR2_LAST;
	I2C2->CR2 |= I2C_CR2_DMAEN;
	I2C2->CR2|=16;//set clock source to 16MHz
	I2C2->CCR=80;  //based on calculation
	I2C2->TRISE=17; //output max rise 
	I2C2->CR1 |=I2C_CR1_PE;
}

void I2C2_RX_DMA_Init(void)
{
	DMA_Tog = 1;
	RCC->AHB1ENR|=RCC_AHB1ENR_DMA1EN;
	DMA1_Stream2->CR=0x00;//reset everything
	while((DMA1_Stream2->CR)&DMA_SxCR_EN){;}
	DMA1_Stream2->CR|=Channel_7|DMA_SxCR_MINC|DMA_SxCR_TCIE;
	NVIC_EnableIRQ(DMA1_Stream2_IRQn);
}

/**
 * @brief   DMA data receive
 * @note    I2C2_RX -> DMA1, Stream 5, Channel 3
 * @param   pBuffer, size
 * @retval  None
 */
static void DMA_Receive(const uint8_t * pBuffer, uint8_t size)
{
	
  /* Check null pointers */
  if(NULL != pBuffer)
  {
    DMA1_Stream2->CR&=~DMA_SxCR_EN;
		while((DMA1_Stream2->CR)&DMA_SxCR_EN){;}

    /* Set memory address */
    DMA1_Stream2->M0AR = (uint32_t)pBuffer;
		DMA1_Stream2->PAR=(uint32_t)&I2C2->DR;
    /* Set number of data items */
    DMA1_Stream2->NDTR = size;

    /* Clear all interrupt flags */
    DMA1->HIFCR = ( DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2
        | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2);

    /* Enable DMA1_Stream2 */
    DMA1_Stream2->CR |= DMA_SxCR_EN;
  }
  else
  {
    /* Null pointers, do nothing */
  }
}		

		
/**
 * @brief   Read register data
 * @note
 * @param   SensorAddr, ReadAddr, pReadBuffer, NumByteToRead
 * @retval  None
 */		
uint32_t time_err;
int I2C_ReadDMA(uint8_t SensorAddr, uint8_t ReadAddr, uint8_t * pReadBuffer, uint16_t NumByteToRead)
{
	//wait until the bus is free
	while(I2C2->SR2&I2C_SR2_BUSY){;}
	
	/* Generate START */
	I2C2->CR1 |= I2C_CR1_START;
	/* Wait SB flag is set */
	while(!(I2C2->SR1&I2C_SR1_SB)){;}

	/* Read SR1 */
	(void)I2C2->SR1;

	/* Send slave address with write */
	I2C2->DR=(SensorAddr|0);

	/* Wait ADDR flag is set */
	while(((I2C2->SR1)&I2C_SR1_ADDR)==0){;}
	/* Read SR1 */
	(void)I2C2->SR1;

	/* Read SR2 */
	(void)I2C2->SR2;

	/* Wait TXE flag is set */
	while(I2C_SR1_TXE != (I2C_SR1_TXE & I2C2->SR1))
	{
		/* Do nothing */
	}

	if(2 <= NumByteToRead)
	{
		/* Acknowledge enable */
		I2C2->CR1 |= I2C_CR1_ACK;

		/* Send register address to read with increment */
		I2C2->DR =  (ReadAddr);
	}
	else
	{
		/* Acknowledge disable */
		I2C2->CR1 &= ~I2C_CR1_ACK;

		/* Send register address to read (single) */
		I2C2->DR =  ReadAddr;
		
	}
	time_err = 0x100;
	/* Wait BTF flag is set */
	while(!(I2C_SR1_BTF & I2C2->SR1))
	{
		time_err--;
		if (time_err == 0)
		{
			I2C_AcknowledgeConfig(I2C2, DISABLE);
			I2C_GenerateSTOP(I2C2, ENABLE);
			DMA1->HIFCR = ( DMA_LIFCR_CDMEIF2 | DMA_LIFCR_CTEIF2
        | DMA_LIFCR_CHTIF2 | DMA_LIFCR_CTCIF2);
			DMA_Cmd(DMA1_Stream2, DISABLE);
			while(DMA_GetCmdStatus(DMA1_Stream2) != DISABLE){}
			return -1;
		}
		/* Do nothing */
	}
	/* Generate ReSTART */
	I2C2->CR1 |= I2C_CR1_START;

	/* Wait SB flag is set */
	while(I2C_SR1_SB != (I2C_SR1_SB & I2C2->SR1))
	{
		/* Do nothing */
	}

	/* Read SR1 */
	(void)I2C2->SR1;

	/* Send slave address with read */
	I2C2->DR =  (SensorAddr | (uint8_t)0x01);

	/* Wait ADDR flag is set */
	while(((I2C2->SR1)&I2C_SR1_ADDR)==0){;}
	
	/* Start DMA */
	DMA_Receive(pReadBuffer, NumByteToRead);

	/* Read SR1 */
	(void)I2C2->SR1;

	/* Read SR2 */
	(void)I2C2->SR2;
	
	return 0;
}

/////////////////////////////////////////////////////////////////////////////////////////////////

int read_finish(void)
{
	return finished;
}
	
void reset_finish(void)
{
	finished=0;
}

void DMA1_Stream2_IRQHandler(void)
{
	if((DMA1->LISR)&DMA_LISR_TCIF2)
	{
		finished=1;
		I2C_AcknowledgeConfig(I2C2, DISABLE);
		I2C2->CR1 |= I2C_CR1_STOP;
		DMA1->LIFCR=DMA_LIFCR_CTCIF2;
	}
}
