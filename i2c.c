/*
	furkan jadid
	8 Juli 2013, Daun Biru Engineering

	I2C

	banya dicopy dari :
	http://eliaselectronics.com/stm32f4-tutorials/stm32f4-i2c-master-tutorial/
*/

#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>

#include "./include/mainku.h"

//#define SLAVE_ADDRESS 0x3D // the slave address (example)

void init_I2C1(void){

	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7; // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB

	// Connect I2C1 pins to AF
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource6, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_I2C1); // SDA

	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

/* ini dicopy dari versi diatas */
void init_I2C3(void)
{
	debug_entry;
	
	GPIO_InitTypeDef GPIO_SCL;
	GPIO_InitTypeDef GPIO_SDA;

	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3, ENABLE);
	// enable clock for SCL and SDA pins
	// SCL di PA8
	// SDA di PC9
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// SCL, PA8
	GPIO_SCL.GPIO_Pin = GPIO_Pin_8; 			// we are going to use PB6 and PB7
	GPIO_SCL.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_SCL.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_SCL.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_SCL.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOA, &GPIO_SCL);					// init GPIOB

	// SDA, PC9
	GPIO_SDA.GPIO_Pin = GPIO_Pin_9; 			// we are going to use PB6 and PB7
	GPIO_SDA.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_SDA.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_SDA.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_SDA.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOC, &GPIO_SDA);	

	// Connect I2C3 pins to AF
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_I2C3);	// SCL
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_I2C3); // SDA

	I2C_Cmd(I2C3, ENABLE);
	
	// configure I2C3
	I2C_InitStruct.I2C_ClockSpeed = 50000; 		// 100kHz, masimum LTC6904 adalah 100 kHz, minimum 10 kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	//I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;	
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C3, &I2C_InitStruct);				// init I2C1

	// enable I2C3
	//I2C_Cmd(I2C3, ENABLE);

	debug_leave;
}

/* 	ini dicopy dari versi diatas 
	untuk CILIWUNG
*/
void init_I2C2(void)
{
	debug_entry;
	
	GPIO_InitTypeDef GPIO_SCL;
	//GPIO_InitTypeDef GPIO_SDA;

	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
	// enable clock for SCL and SDA pins
	// SCL di PH4
	// SDA di PH5
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH, ENABLE);
	//RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// SCL, PH4 & PH5
	GPIO_SCL.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5; 			// we are going to use PB6 and PB7
	GPIO_SCL.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_SCL.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_SCL.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_SCL.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOH, &GPIO_SCL);					// init GPIOB

	/*
	// SDA, PC9
	GPIO_SDA.GPIO_Pin = GPIO_Pin_9; 			// we are going to use PB6 and PB7
	GPIO_SDA.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_SDA.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_SDA.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_SDA.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOC, &GPIO_SDA);	
	*/
	
	// Connect I2C3 pins to AF
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource4, GPIO_AF_I2C2);	// SCL
	GPIO_PinAFConfig(GPIOH, GPIO_PinSource5, GPIO_AF_I2C2); // SDA

	I2C_Cmd(I2C2, ENABLE);
	
	// configure I2C3
	I2C_InitStruct.I2C_ClockSpeed = 50000; 		// 100kHz, masimum LTC6904 adalah 100 kHz, minimum 10 kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	//I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;	
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C2, &I2C_InitStruct);				// init I2C1

	// enable I2C3
	//I2C_Cmd(I2C3, ENABLE);

	debug_leave;
}

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */
int I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction)
{
	//debug_entry;
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY));
	printf("1.");
	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);
	printf("2.");
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT));
	printf("3.");
	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);
	printf("4.");
	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	 int loop = 1024*1024;
	if(direction == I2C_Direction_Transmitter)
	{
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
		{
			loop--;
			togle_led();
			
			if (!loop) 
			{
				printf("I2C3 : 5 failed\n");
				I2C_GenerateSTOP(I2Cx, ENABLE);

				return -1;
				//break;
			}
		}
		printf("5.");
	}
	else if(direction == I2C_Direction_Receiver)
	{
		loop = 1024*1024;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED))
		{
			loop--;
			togle_led();
			
			if (!loop)
			{
				printf("I2C3 : 6 failed\n");
				I2C_GenerateSTOP(I2Cx, ENABLE);
				//break;

				return -1;
			}
		}
		printf("6.");
	}
	//debug_leave;

	return 0;
}

void i2c_write_buf(uint8_t address, uint8_t* buf, int32_t len)
{
  while(I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));
   
  /* Send I2C1 START condition */
  I2C_GenerateSTART(I2C1, ENABLE);
   
  /* Test on I2C1 EV5 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));
     
  /* Send slave Address for write */
  I2C_Send7bitAddress(I2C1, address, I2C_Direction_Transmitter);
   
  /* Test on I2C1 EV6 and clear it */
  while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)); //I2C stucks here!!!
   
  while(len--)
  {
    /* Send I2C1 slave register address */
    I2C_SendData(I2C1, *buf++);
     
    /* Test on I2C1 EV8_2 and clear it */
    while(!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
  }
   
  /* Send I2C1 STOP Condition */
  I2C_GenerateSTOP(I2C1, ENABLE); 
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx)
{
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx)
{
	// disabe acknowledge of received data
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) );
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}
