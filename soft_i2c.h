/**
 * @file soft_i2c.h
 * @brief 基于STM32 HAL库的软件I2C实现
 */

#ifndef __SOFT_I2C_H
#define __SOFT_I2C_H

#include "main.h"

#define SOFT_STANDARD 100000
#define SOFT_FAST 400000


/**
 * @brief 软件I2C句柄结构
 */
typedef struct {
    GPIO_TypeDef* scl_port;     // SCL引脚端口
    uint16_t scl_pin;           // SCL引脚号
    GPIO_TypeDef* sda_port;     // SDA引脚端口
    uint16_t sda_pin;           // SDA引脚号
    uint32_t delay_us;          // 时钟延时（微秒）
} SoftI2C_HandleTypeDef;

/**
 * @brief 初始化软件I2C
 * @param hi2c 软件I2C句柄
 * @param scl_port SCL引脚端口
 * @param scl_pin SCL引脚号
 * @param sda_port SDA引脚端口
 * @param sda_pin SDA引脚号
 * @param frequency 通信频率（Hz）
 */
uint8_t SoftI2C_Init(SoftI2C_HandleTypeDef* hi2c, 
                  GPIO_TypeDef* scl_port, uint16_t scl_pin,
                  GPIO_TypeDef* sda_port, uint16_t sda_pin,
                  uint32_t frequency);

/**
 * @brief 发送I2C开始信号
 * @param hi2c 软件I2C句柄
 */
void SoftI2C_Start(SoftI2C_HandleTypeDef* hi2c);

/**
 * @brief 发送I2C停止信号
 * @param hi2c 软件I2C句柄
 */
void SoftI2C_Stop(SoftI2C_HandleTypeDef* hi2c);

/**
 * @brief 发送ACK信号
 * @param hi2c 软件I2C句柄
 */
void SoftI2C_SendAck(SoftI2C_HandleTypeDef* hi2c);

/**
 * @brief 发送NACK信号
 * @param hi2c 软件I2C句柄
 */
void SoftI2C_SendNack(SoftI2C_HandleTypeDef* hi2c);

/**
 * @brief 等待ACK信号
 * @param hi2c 软件I2C句柄
 * @retval 1-收到ACK，0-未收到ACK
 */
uint8_t SoftI2C_WaitAck(SoftI2C_HandleTypeDef* hi2c);

/**
 * @brief 发送一个字节
 * @param hi2c 软件I2C句柄
 * @param data 要发送的字节
 * @retval 1-发送成功，0-发送失败
 */
uint8_t SoftI2C_SendByte(SoftI2C_HandleTypeDef* hi2c, uint8_t data);

/**
 * @brief 接收一个字节
 * @param hi2c 软件I2C句柄
 * @param ack 1-接收后发送ACK，0-接收后发送NACK
 * @return 接收到的字节
 */
uint8_t SoftI2C_ReceiveByte(SoftI2C_HandleTypeDef* hi2c, uint8_t ack);

/**
 * @brief 读取多个字节
 * @param hi2c 软件I2C句柄
 * @param dev_addr 设备地址
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @retval 1-成功，0-失败
 */
uint8_t SoftI2C_Read(SoftI2C_HandleTypeDef* hi2c, uint8_t dev_addr, 
                    uint8_t reg_addr, uint8_t* data, uint16_t len);

/**
 * @brief 写入多个字节
 * @param hi2c 软件I2C句柄
 * @param dev_addr 设备地址
 * @param reg_addr 寄存器地址
 * @param data 数据缓冲区
 * @param len 数据长度
 * @retval 1-成功，0-失败
 */
uint8_t SoftI2C_Write(SoftI2C_HandleTypeDef* hi2c, uint8_t dev_addr, 
                     uint8_t reg_addr, uint8_t* data, uint16_t len);

/**
 * @brief 微秒级延时函数
 * @param delay 延时时间（微秒）
 */
void SoftI2C_DelayUs(uint32_t delay);

#endif /* __SOFT_I2C_H */    

