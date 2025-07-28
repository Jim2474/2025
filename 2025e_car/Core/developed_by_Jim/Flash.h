#ifndef __FLASH_H
#define __FLASH_H

#include "stm32f4xx_hal.h"

// 片选引脚定义（根据CubeMX实际配置修改）
#define W25Q128_CS_GPIO_PORT   GPIOA
#define W25Q128_CS_PIN         GPIO_PIN_4

#define W25Q128_CS_LOW()   HAL_GPIO_WritePin(W25Q128_CS_GPIO_PORT, W25Q128_CS_PIN, GPIO_PIN_RESET)
#define W25Q128_CS_HIGH()  HAL_GPIO_WritePin(W25Q128_CS_GPIO_PORT, W25Q128_CS_PIN, GPIO_PIN_SET)

// W25Q128 指令集（根据用户提供的表格更正）
#define W25Q128_CMD_WRITE_ENABLE      0x06  // 写使能
#define W25Q128_CMD_WRITE_DISABLE     0x04  // 写禁止（未在表格中）
#define W25Q128_CMD_READ_STATUS1      0x05  // 读SR1
#define W25Q128_CMD_READ_STATUS2      0x35  // 读SR2（未在表格中）
#define W25Q128_CMD_READ_DATA         0x03  // 读数据
#define W25Q128_CMD_FAST_READ         0x0B  // 快速读（未在表格中）
#define W25Q128_CMD_PAGE_PROGRAM      0x02  // 页写
#define W25Q128_CMD_SECTOR_ERASE      0x20  // 扇区擦除
#define W25Q128_CMD_BLOCK_ERASE_32K   0x52  // 32K块擦除（未在表格中）
#define W25Q128_CMD_BLOCK_ERASE_64K   0xD8  // 64K块擦除（未在表格中）
#define W25Q128_CMD_CHIP_ERASE        0xC7  // 芯片擦除
#define W25Q128_CMD_READ_ID           0x90  // 读取芯片ID（表格中的指令）
#define W25Q128_CMD_JEDEC_ID          0x9F  // JEDEC ID（标准指令，虽然表格中没有）

#define W25Q128_PAGE_SIZE             256
#define W25Q128_SECTOR_SIZE           4096

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化W25Q128
 * @param hspi SPI句柄
 */
void W25Q128_Init(SPI_HandleTypeDef *hspi);

/**
 * @brief 读取JEDEC ID
 * @return 24位ID
 */
uint32_t W25Q128_ReadID(void);

/**
 * @brief 读取数据
 * @param addr 起始地址
 * @param buf  数据缓冲区
 * @param len  读取长度
 */
void W25Q128_Read(uint32_t addr, uint8_t *buf, uint32_t len);

/**
 * @brief 页编程（最多256字节，需先擦除）
 * @param addr 页起始地址
 * @param buf  数据缓冲区
 * @param len  写入长度（<=256）
 */
void W25Q128_PageProgram(uint32_t addr, const uint8_t *buf, uint16_t len);

/**
 * @brief 擦除一个扇区（4KB）
 * @param addr 扇区起始地址
 */
void W25Q128_SectorErase(uint32_t addr);

#ifdef __cplusplus
}
#endif

#endif

