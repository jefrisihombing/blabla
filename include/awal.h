#ifndef H_AWAL_H
#define H_AWAL_H

void RCC_Configuration(void);
void GPIO_Configuration(void);
void USART3_Configuration(void);
void Delay(__IO uint32_t nCount);


void TIM5_Config(void);
void TIM3_Config(void);

void TIM5_Jalan();
void TIM3_Jalan();



/* i2c.c */
int I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction);
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data);
void I2C_stop(I2C_TypeDef* I2Cx);

int init_clock_khz(void);
void set_clock_khz(int khz);

#endif
