/**
 * @file soft_i2c.c
 * @brief 基于STM32 HAL库的软件I2C实现
 */

#include "soft_i2c.h"

/**
 * @brief 设置SDA引脚为输入模式
 * @param hi2c 软件I2C句柄
 */
static void SDA_IN(SoftI2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hi2c->sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(hi2c->sda_port, &GPIO_InitStruct);
}

/**
 * @brief 设置SDA引脚为输出模式
 * @param hi2c 软件I2C句柄
 */
static void SDA_OUT(SoftI2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = hi2c->sda_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(hi2c->sda_port, &GPIO_InitStruct);
}

/**
 * @brief 读取SDA引脚状态
 * @param hi2c 软件I2C句柄
 * @return 引脚状态（0或1）
 */
static uint8_t READ_SDA(SoftI2C_HandleTypeDef* hi2c) {
    return HAL_GPIO_ReadPin(hi2c->sda_port, hi2c->sda_pin);
}

/**
 * @brief 设置SCL引脚状态
 * @param hi2c 软件I2C句柄
 * @param state 引脚状态（0或1）
 */
static void SET_SCL(SoftI2C_HandleTypeDef* hi2c, uint8_t state) {
    HAL_GPIO_WritePin(hi2c->scl_port, hi2c->scl_pin, 
                     (GPIO_PinState)state);
}

/**
 * @brief 设置SDA引脚状态
 * @param hi2c 软件I2C句柄
 * @param state 引脚状态（0或1）
 */
static void SET_SDA(SoftI2C_HandleTypeDef* hi2c, uint8_t state) {
    HAL_GPIO_WritePin(hi2c->sda_port, hi2c->sda_pin, 
                     (GPIO_PinState)state);
}

uint8_t SoftI2C_Init(SoftI2C_HandleTypeDef* hi2c, 
                  GPIO_TypeDef* scl_port, uint16_t scl_pin,
                  GPIO_TypeDef* sda_port, uint16_t sda_pin,
                  uint32_t frequency) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint8_t status = 1;  // 默认成功

    // 保存引脚信息
    hi2c->scl_port = scl_port;
    hi2c->scl_pin = scl_pin;
    hi2c->sda_port = sda_port;
    hi2c->sda_pin = sda_pin;
    
    // 计算延时（微秒）
    if (frequency > 0) {
        // 标准模式100kHz：延时约5us
        // 快速模式400kHz：延时约1.25us
        hi2c->delay_us = 500000 / frequency; // 近似计算
    } else {
        hi2c->delay_us = 5; // 默认100kHz
    }
    
    // 使能GPIO时钟
    if (scl_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (scl_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (scl_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (scl_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else status = 0;
    
    if (sda_port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    else if (sda_port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    else if (sda_port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
    else if (sda_port == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
    else status = 0;
    
    // 配置SCL引脚
	if(status == 1)
	{
    GPIO_InitStruct.Pin = scl_pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(scl_port, &GPIO_InitStruct);
    
    // 配置SDA引脚
    GPIO_InitStruct.Pin = sda_pin;
    HAL_GPIO_Init(sda_port, &GPIO_InitStruct);
    
    // 初始化I2C总线
    SET_SCL(hi2c, 1);
    SET_SDA(hi2c, 1);
	}
	
	return status;
}

void SoftI2C_Start(SoftI2C_HandleTypeDef* hi2c) {
    SDA_OUT(hi2c);
    SET_SDA(hi2c, 1);
    SET_SCL(hi2c, 1);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SDA(hi2c, 0); // START: 当SCL为高时，SDA由高变低
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SCL(hi2c, 0); // 钳住I2C总线，准备发送或接收数据
}

void SoftI2C_Stop(SoftI2C_HandleTypeDef* hi2c) {
    SDA_OUT(hi2c);
    SET_SCL(hi2c, 0);
    SET_SDA(hi2c, 0);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SCL(hi2c, 1); // STOP: 当SCL为高时，SDA由低变高
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SDA(hi2c, 1); // 发送I2C总线结束信号
}

uint8_t SoftI2C_WaitAck(SoftI2C_HandleTypeDef* hi2c) {
    uint8_t ucErrTime = 0;
    
    SDA_IN(hi2c);
    SET_SDA(hi2c, 1);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SCL(hi2c, 1);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    while (READ_SDA(hi2c)) {
        ucErrTime++;
        if (ucErrTime > 250) {
            SoftI2C_Stop(hi2c);
            return 0;
        }
    }
    
    SET_SCL(hi2c, 0); // 时钟输出0
    return 1;
}

void SoftI2C_SendAck(SoftI2C_HandleTypeDef* hi2c) {
    SDA_OUT(hi2c);
    SET_SCL(hi2c, 0);
    SET_SDA(hi2c, 0);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SCL(hi2c, 1);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SCL(hi2c, 0);
}

void SoftI2C_SendNack(SoftI2C_HandleTypeDef* hi2c) {
    SDA_OUT(hi2c);
    SET_SCL(hi2c, 0);
    SET_SDA(hi2c, 1);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SCL(hi2c, 1);
    SoftI2C_DelayUs(hi2c->delay_us);
    
    SET_SCL(hi2c, 0);
}

uint8_t SoftI2C_SendByte(SoftI2C_HandleTypeDef* hi2c, uint8_t data) {
    uint8_t t;
    
    SDA_OUT(hi2c);
    SET_SCL(hi2c, 0); // 拉低时钟开始数据传输
    
    for (t = 0; t < 8; t++) {
        SET_SDA(hi2c, (data & 0x80) >> 7);
        data <<= 1;
        SoftI2C_DelayUs(hi2c->delay_us);
        
        SET_SCL(hi2c, 1);
        SoftI2C_DelayUs(hi2c->delay_us);
        
        SET_SCL(hi2c, 0);
        SoftI2C_DelayUs(hi2c->delay_us);
    }
    
    return SoftI2C_WaitAck(hi2c);
}

uint8_t SoftI2C_ReceiveByte(SoftI2C_HandleTypeDef* hi2c, uint8_t ack) {
    uint8_t i, receive = 0;
    
    SDA_IN(hi2c); // SDA设置为输入
    
    for (i = 0; i < 8; i++) {
        SET_SCL(hi2c, 0);
        SoftI2C_DelayUs(hi2c->delay_us);
        
        SET_SCL(hi2c, 1);
        receive <<= 1;
        
        if (READ_SDA(hi2c)) receive++;
        SoftI2C_DelayUs(hi2c->delay_us);
    }
    
    if (!ack) {
        SoftI2C_SendNack(hi2c); // 发送NACK
    } else {
        SoftI2C_SendAck(hi2c); // 发送ACK
    }
    
    return receive;
}

uint8_t SoftI2C_Read(SoftI2C_HandleTypeDef* hi2c, uint8_t dev_addr, 
                    uint8_t reg_addr, uint8_t* data, uint16_t len) {
    SoftI2C_Start(hi2c);
    
    // 发送设备地址+写命令
    if (!SoftI2C_SendByte(hi2c, dev_addr << 1 | 0)) {
        return 0;
    }
    
    // 发送寄存器地址
    if (!SoftI2C_SendByte(hi2c, reg_addr)) {
        return 0;
    }
    
    SoftI2C_Start(hi2c);
    
    // 发送设备地址+读命令
    if (!SoftI2C_SendByte(hi2c, dev_addr << 1 | 1)) {
        return 0;
    }
    
    // 读取数据
    while (len) {
        if (len == 1) {
            *data = SoftI2C_ReceiveByte(hi2c, 0); // 最后一个数据，发送NACK
        } else {
            *data = SoftI2C_ReceiveByte(hi2c, 1); // 发送ACK
        }
        data++;
        len--;
    }
    
    SoftI2C_Stop(hi2c);
    return 1;
}

uint8_t SoftI2C_Write(SoftI2C_HandleTypeDef* hi2c, uint8_t dev_addr, 
                     uint8_t reg_addr, uint8_t* data, uint16_t len) {
    SoftI2C_Start(hi2c);
    
    // 发送设备地址+写命令
    if (!SoftI2C_SendByte(hi2c, dev_addr << 1 | 0)) {
        return 0;
    }
    
    // 发送寄存器地址
    if (!SoftI2C_SendByte(hi2c, reg_addr)) {
        return 0;
    }
    
    // 发送数据
    while (len--) {
        if (!SoftI2C_SendByte(hi2c, *data)) {
            return 0;
        }
        data++;
    }
    
    SoftI2C_Stop(hi2c);
    return 1;
}

// 微秒级延时函数实现
// 注意：不同STM32芯片的实现方式可能不同
// 这里提供一个基于SysTick的通用实现
void SoftI2C_DelayUs(uint32_t us) {
    uint32_t cycles = us * (SystemCoreClock / 1000000) / 3; // 系数可校准
    
    __asm {
        loop:
            SUBS cycles, cycles, #1
            NOP
            BNE loop
    }
}

/*
void SoftI2C_DelayUs(uint32_t delay) {
 uint32_t cycles = us * (SystemCoreClock / 1000000);
    __asm__ volatile (
        "1: subs %0, #1\n\t"  // 1 cycle
        "   bne 1b"           // 1 cycle (跳转时)
        : "+r" (cycles)
        :
        : "cc"
    );
}    
*/


