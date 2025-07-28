#include "Flash.h"

static SPI_HandleTypeDef *W25Q128_hspi = NULL;

/**
 * @brief 初始化W25Q128
 * @param hspi SPI句柄
 */
void W25Q128_Init(SPI_HandleTypeDef *hspi)
{
    W25Q128_hspi = hspi;
    W25Q128_CS_HIGH();
}

/**
 * @brief SPI发送接收一个字节
 */
static uint8_t W25Q128_SPI_Transfer(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(W25Q128_hspi, &data, &rx, 1, HAL_MAX_DELAY);
    return rx;
}

/**
 * @brief 写使能
 */
static void W25Q128_WriteEnable(void)
{
    W25Q128_CS_LOW();
    W25Q128_SPI_Transfer(W25Q128_CMD_WRITE_ENABLE);
    W25Q128_CS_HIGH();
}

/**
 * @brief 等待忙
 */
static void W25Q128_WaitBusy(void)
{
    W25Q128_CS_LOW();
    W25Q128_SPI_Transfer(W25Q128_CMD_READ_STATUS1);
    while (W25Q128_SPI_Transfer(0xFF) & 0x01);
    W25Q128_CS_HIGH();
}

/**
 * @brief 读取JEDEC ID
 */
uint32_t W25Q128_ReadID(void)
{
    uint32_t id = 0;
    uint8_t byte1, byte2, byte3;
    
    W25Q128_CS_LOW();
    HAL_Delay(1);
    
    W25Q128_SPI_Transfer(W25Q128_CMD_JEDEC_ID);
    byte1 = W25Q128_SPI_Transfer(0xFF);
    byte2 = W25Q128_SPI_Transfer(0xFF);  
    byte3 = W25Q128_SPI_Transfer(0xFF);
    
    W25Q128_CS_HIGH();
    
    // 在OLED上分别显示三个字节，看看是否都是0
    // 如果都是0，说明SPI通信有问题
    // 如果是0xFF，说明Flash没有响应
    
    id = (byte1 << 16) | (byte2 << 8) | byte3;
    return id;
}

/**
 * @brief 读取数据
 */
void W25Q128_Read(uint32_t addr, uint8_t *buf, uint32_t len)
{
    W25Q128_CS_LOW();
    W25Q128_SPI_Transfer(W25Q128_CMD_READ_DATA);
    W25Q128_SPI_Transfer((addr >> 16) & 0xFF);
    W25Q128_SPI_Transfer((addr >> 8) & 0xFF);
    W25Q128_SPI_Transfer(addr & 0xFF);
    for (uint32_t i = 0; i < len; i++)
        buf[i] = W25Q128_SPI_Transfer(0xFF);
    W25Q128_CS_HIGH();
}

/**
 * @brief 页编程
 */
void W25Q128_PageProgram(uint32_t addr, const uint8_t *buf, uint16_t len)
{
    W25Q128_WriteEnable();
    W25Q128_CS_LOW();
    W25Q128_SPI_Transfer(W25Q128_CMD_PAGE_PROGRAM);
    W25Q128_SPI_Transfer((addr >> 16) & 0xFF);
    W25Q128_SPI_Transfer((addr >> 8) & 0xFF);
    W25Q128_SPI_Transfer(addr & 0xFF);
    for (uint16_t i = 0; i < len; i++)
        W25Q128_SPI_Transfer(buf[i]);
    W25Q128_CS_HIGH();
    W25Q128_WaitBusy();
}

/**
 * @brief 擦除扇区
 */
void W25Q128_SectorErase(uint32_t addr)
{
    W25Q128_WriteEnable();
    W25Q128_CS_LOW();
    W25Q128_SPI_Transfer(W25Q128_CMD_SECTOR_ERASE);
    W25Q128_SPI_Transfer((addr >> 16) & 0xFF);
    W25Q128_SPI_Transfer((addr >> 8) & 0xFF);
    W25Q128_SPI_Transfer(addr & 0xFF);
    W25Q128_CS_HIGH();
    W25Q128_WaitBusy();
}
