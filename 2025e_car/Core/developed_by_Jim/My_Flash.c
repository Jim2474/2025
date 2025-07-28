#include "My_Flash.h"
#define FMC_FLASH_BASE      0x08000000   // FLASH的起始地址
#define FMC_FLASH_END       0x08080000   // FLASH的结束地址

#define FLASH_WAITETIME     50000        //FLASH等待超时时间

//FLASH 扇区的起始地址
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	//扇区0起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	//扇区1起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	//扇区2起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//扇区3起始地址, 16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//扇区4起始地址, 64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//扇区5起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//扇区6起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//扇区7起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//扇区8起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//扇区9起始地址, 128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//扇区10起始地址,128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//扇区11起始地址,128 Kbytes 

//读取指定地址的字(32位数据) 
//faddr:读地址 
//返回值:对应数据.
static uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
    return *(uint32_t*)faddr; 
}

//获取某个地址所在的flash扇区
//addr:flash地址
//返回值:0~11,即addr所在的扇区
uint8_t STMFLASH_GetFlashSector(uint32_t addr)
{
    if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
    else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
    else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
    else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
    else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
    else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
    else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
    else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
    else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
    else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
    else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10;   
    return FLASH_SECTOR_11;	
}

/**
 *@功能：向内部Flash写入数据
 *@参数1：WriteAddress：数据要写入的目标地址（偏移地址）
 *@参数2：*data： 写入的数据首地址
 *@参数3：length：写入数据的个数
 */
void WriteFlashData(uint32_t WriteAddress, uint8_t *data, uint32_t length)
{
    FLASH_EraseInitTypeDef FlashEraseInit;
    HAL_StatusTypeDef FlashStatus=HAL_OK;
    uint32_t SectorError=0;
    uint32_t addrx=0;
    uint32_t endaddr=0;
    
    if( (WriteAddress < FMC_FLASH_BASE) || ( WriteAddress + length >= FMC_FLASH_END) || (length <= 0) )
    return;

    HAL_FLASH_Unlock();              //解锁
    addrx = WriteAddress;            //写入的起始地址
    endaddr = WriteAddress+length;   //写入的结束地址


        while(addrx<endaddr)  //扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
        {
             if(STMFLASH_ReadWord(addrx)!=0XFFFFFFFF)//有非0XFFFFFFFF的地方,要擦除这个扇区
            {   
                FlashEraseInit.TypeErase=FLASH_TYPEERASE_SECTORS;       //擦除类型，扇区擦除 
                FlashEraseInit.Sector=STMFLASH_GetFlashSector(addrx);   //要擦除的扇区
                FlashEraseInit.NbSectors=1;                             //一次只擦除一个扇区
                FlashEraseInit.VoltageRange=FLASH_VOLTAGE_RANGE_3;      //电压范围，VCC=2.7~3.6V之间!!
                if(HAL_FLASHEx_Erase(&FlashEraseInit,&SectorError)!=HAL_OK) 
                {
                    break;//发生错误了
                }
                }else addrx+=1;
                FLASH_WaitForLastOperation(FLASH_WAITETIME);                //等待上次操作完成
        }
    
    FlashStatus=FLASH_WaitForLastOperation(FLASH_WAITETIME);            //等待上次操作完成
    if(FlashStatus==HAL_OK)
    {
         while(WriteAddress<endaddr)//写数据
         {
            if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE,WriteAddress,*data)!=HAL_OK)//写入数据
            { 
                break;	//写入异常
            }
            WriteAddress+=1;
            data++;
        }  
    }
    HAL_FLASH_Lock();           //上锁
}

/**
 * @brief 存储一个32位数字到指定地址（参考字符串写入方法）
 * @param addr Flash地址
 * @param number 要存储的数字
 * @return 0:成功, 1:失败
 */
uint8_t SaveNumber(uint32_t addr, uint32_t number)
{
    uint8_t bytes[4];

    // 将32位数字分解为4个字节（小端序），和字符串写入方法一样
    bytes[0] = (uint8_t)(number & 0xFF);
    bytes[1] = (uint8_t)((number >> 8) & 0xFF);
    bytes[2] = (uint8_t)((number >> 16) & 0xFF);
    bytes[3] = (uint8_t)((number >> 24) & 0xFF);

    // 使用和字符串一样的写入方法
    WriteFlashData(addr, bytes, 4);

    return 0; // 成功
}

/**
 * @brief 从指定地址读取32位数字
 * @param addr Flash地址
 * @return 读取到的数字
 */
/**
 * @brief 从Flash读取一个数字（参考字符串读取方法，按字节读取）
 * @param addr Flash地址
 * @return 读取的数字
 */
uint32_t ReadNumber(uint32_t addr)
{
    uint32_t number = 0;
    uint8_t bytes[4];

    // 参考字符串读取方法，按字节读取4个字节
    for (uint8_t i = 0; i < 4; i++)
    {
        // 直接从内存地址读取字节，和字符串读取方法一样
        bytes[i] = *(uint8_t*)(addr + i);

        // 如果读取到0xFF，说明Flash为空
        if (bytes[i] == 0xFF)
        {
            bytes[i] = 0;
        }
    }

    // 将4个字节组合成32位数字（小端序）
    number = (uint32_t)bytes[0] |
             ((uint32_t)bytes[1] << 8) |
             ((uint32_t)bytes[2] << 16) |
             ((uint32_t)bytes[3] << 24);

    return number;
}

/**
 * @brief 存储数字数组
 * @param addr Flash地址
 * @param numbers 数字数组
 * @param count 数组长度
 * @return 0:成功, 1:失败
 */
uint8_t SaveNumberArray(uint32_t addr, uint32_t *numbers, uint8_t count)
{
    uint8_t status = 0;
    uint32_t length = count * sizeof(uint32_t);

    WriteFlashData(addr, (uint8_t*)numbers, length);
    status = 0; // Assuming 0 means success
    return status;
}

/**
 * @brief 读取数字数组（参考字符串读取方法，按字节读取）
 * @param addr Flash地址
 * @param numbers 存储读取数据的数组
 * @param count 要读取的数量
 */
void ReadNumberArray(uint32_t addr, uint32_t *numbers, uint8_t count)
{
    for (uint8_t i = 0; i < count; i++)
    {
        // 使用修正后的ReadNumber函数，按字节读取
        numbers[i] = ReadNumber(addr + i * 4);
    }
}


// 存储字符串到FLASH
void SaveStringToFlash(void)
{
    char* str = "Hello, Flash!";  // 要存储的字符串
    uint32_t len = strlen(str) + 1;  // 计算长度，+1是为了包含结束符'\0'
    
    // 写入字符串数据
    WriteFlashData(STRING_STORAGE_ADDR, (uint8_t*)str, len);
}

// 从FLASH读取字符串
void ReadStringFromFlash(char* buffer, uint32_t max_len)
{
    uint32_t addr = STRING_STORAGE_ADDR;
    uint32_t i = 0;
    
    // 逐个字节读取，直到遇到结束符或达到最大长度
    while (i < max_len - 1)
    {
        buffer[i] = (char)STMFLASH_ReadWord(addr);  // 读取一个字节
        
        if (buffer[i] == '\0')  // 遇到字符串结束符
            break;
            
        addr++;
        i++;
    }
    
    buffer[i] = '\0';  // 确保字符串以结束符结尾
}

//// 使用示例
//int main(void)
//{
//    char read_buf[100];  // 用于存放读取的字符串
//    
//    // 保存字符串到FLASH
//    SaveStringToFlash();
//    
//    // 从FLASH读取字符串
//    ReadStringFromFlash(read_buf, sizeof(read_buf));
//    
//    // 此时read_buf中应该包含"Hello, Flash!"
//    while(1)
//    {
//        // 其他程序逻辑
//    }
//}


// //flash ćľčŻ  示例代码
//     uint32_t test_data []={1,2,3};
//     uint32_t read_data[4];
    
//     // ĺ­ĺ¨ć°ćŽ
//     SaveNumberArray(0x08060000, test_data,3);
    
//     // čŻťĺć°ćŽ
//     ReadNumberArray(0x08060000, read_data, 3);  // 正确的调用方法
    
