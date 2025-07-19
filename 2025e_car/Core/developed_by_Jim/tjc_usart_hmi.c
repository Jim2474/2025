/**
使用注意事项:
    1.将tjc_usart_hmi.c和tjc_usart_hmi.h 分别导入工程
    2.在需要使用的函数所在的头文件中添加 #include "tjc_usart_hmi.h"
    3.使用前请将 HAL_UART_Transmit_IT() 这个函数改为你的单片机的串口发送单字节函数


*/

#include "tjc_usart_hmi.h"
#include "Mytimer.h"  // 包含延时函数

void set_image_aph(uint8_t pic_id, uint8_t aph_val)  //我封装的图片切换函数，第一个是火源编号1-6，第二个是0或者127，0就是不可见，127是可见
{
    char str[64];

    // 图片编号必须在1~6之间
    if (pic_id < 1 || pic_id > 6) return;

    // 构建字符串：main.pX.aph=127或0，再加结束符
    sprintf(str, "main.p%d.aph=%d\xFF\xFF\xFF", pic_id, aph_val);
    //OLED_ShowString(0, 0, str, 16, 1); // 显示在OLED屏幕上，调试用
    // 发送到串口屏
    tjc_send_string(str);
}
void display_coordinates(const char* txt_id, int x, int y,int z) //坐标显示函数
{
    char str[64];
    sprintf(str, "%s.txt=\"(%d,%d,%d)\"\xFF\xFF\xFF", txt_id, x, y,z);  //main.t1
    tjc_send_string(str);
}

/**
 * @brief 更新控件后重绘轨迹（专门用于控件更新后的轨迹恢复）
 */
void redraw_tracks_after_control_update(void)
{
    // 给串口屏充足时间处理控件更新
    HAL_Delay(10);  // 短暂延时，确保控件更新完成

    // 立即重绘所有轨迹
    redraw_all_tracks();
}
void draw_line_on_screen(int x1, int y1, int x2, int y2, int color) //画线指令
{
    char cmd[64];

    // 坐标系转换,飞机和串口屏的坐标系不同
    int screen_x1 = x1+150;
    int screen_x2 = x2+150;    
    int screen_y1 = 450-y1;
    int screen_y2 = 450-y2;
    
   if (delay_ms_nb(500)) 
   {
    sprintf(cmd, "line %d,%d,%d,%d,%d\xFF\xFF\xFF", screen_x1, screen_y1, screen_x2, screen_y2, color);
    tjc_send_string(cmd);  // 一次性通过已验证的串口函数发送
    }
   
}


typedef struct
{
    uint16_t Head;
    uint16_t Tail;
    uint16_t Length;
    uint8_t  Ring_data[RINGBUFFER_LEN];
}RingBuffer_t;

RingBuffer_t ringBuffer;	//创建一个ringBuffer的缓冲区
uint8_t RxBuffer[1];


/********************************************************
函数名：  		intToStr
日期：    	2024.09.18
功能：    	将整形转换为字符串
输入参数：		要转换的整形数据,输出的字符串数组
返回值： 		无
修改记录：
**********************************************************/
void intToStr(int num, char* str) {
    int i = 0;
    int isNegative = 0;

    // 处理负数
    if (num < 0) {
        isNegative = 1;
        num = -num;
    }

    // 提取每一位数字
    do {
        str[i++] = (num % 10) + '0';
        num /= 10;
    } while (num);

    // 如果是负数，添加负号
    if (isNegative) {
        str[i++] = '-';
    }

    // 添加字符串终止符
    str[i] = '\0';

    // 反转字符串
    int start = 0;
    int end = i - 1;
    while (start < end) {
        char temp = str[start];
        str[start] = str[end];
        str[end] = temp;
        start++;
        end--;
    }
    return ;
}



/********************************************************
函数名：  		uart_send_char
日期：    	2024.09.18
功能：    	串口发送单个字符
输入参数：		要发送的单个字符
返回值： 		无
修改记录：
**********************************************************/
void uart_send_char(char ch)
{
	uint8_t ch2 = (uint8_t)ch;
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
	//while(__HAL_UART_GET_FLAG(&TJC_UART, UART_FLAG_TXE) == RESET);	//等待发送完毕
	while(__HAL_UART_GET_FLAG(&TJC_UART, UART_FLAG_TC) == RESET);
    //发送单个字符
	//HAL_UART_Transmit_IT(&TJC_UART, &ch2, 1);
	HAL_UART_Transmit_DMA(&TJC_UART, &ch2, 1);
	return;
}


void uart_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_send_char(*str++);
    }
	return;
}

/********************************************************
函数名：  		tjc_send_string
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串
返回值： 		无
示例:			tjc_send_val("n0", "val", 100); 发出的数据就是 n0.val=100
修改记录：
**********************************************************/
void tjc_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

/********************************************************
函数名：  		tjc_send_txt
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串
返回值： 		无
示例:			tjc_send_txt("t0", "txt", "ABC"); 发出的数据就是t0.txt="ABC"
修改记录：
**********************************************************/
void tjc_send_txt(char* objname, char* attribute, char* txt)
{

    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_string("=\"");
    uart_send_string(txt);
    uart_send_char('\"');
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}


/********************************************************
函数名：  		tjc_send_val
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串
返回值： 		无
修改记录：
**********************************************************/
void tjc_send_val(char* objname, char* attribute, int val)
{
	//拼接字符串,比如n0.val=123
    uart_send_string(objname);
    uart_send_char('.');
    uart_send_string(attribute);
    uart_send_char('=');
    //C语言中整形的取值范围是：“-2147483648 ~ 2147483647”, 最长为-2147483648,加上结束符\0一共12个字符
    char txt[12]="";
    intToStr(val, txt);
    uart_send_string(txt);
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}

/********************************************************
函数名：  		tjc_send_nstring
日期：    	2024.09.18
功能：    	串口发送字符串和结束符
输入参数：		要发送的字符串,字符串长度
返回值： 		无
修改记录：
**********************************************************/
void tjc_send_nstring(char* str, unsigned char str_length)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    for (int var = 0; var < str_length; ++var)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_send_char(*str++);
    }
	uart_send_char(0xff);
	uart_send_char(0xff);
	uart_send_char(0xff);
	return;
}





/********************************************************
函数名：  	HAL_UART_RxCpltCallback
日期：    	2022.10.08
功能：    	串口接收中断,将接收到的数据写入环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == TJC_UART_INS)	// 判断是由哪个串口触发的中断
	{
		write1ByteToRingBuffer(RxBuffer[0]);
		HAL_UART_Receive_IT(&TJC_UART,RxBuffer,1);		// 重新使能串口2接收中断
	}
	return;
}



/********************************************************
函数名：  		initRingBuffer
日期：    	2022.10.08
功能：    	初始化环形缓冲区
输入参数：
返回值： 		void
修改记录：
**********************************************************/
void initRingBuffer(void)
{
	//初始化相关信息
	ringBuffer.Head = 0;
	ringBuffer.Tail = 0;
	ringBuffer.Length = 0;
	return;
}



/********************************************************
函数名：  		write1ByteToRingBuffer
日期：    	2022.10.08
功能：    	往环形缓冲区写入数据
输入参数：		要写入的1字节数据
返回值： 		void
修改记录：
**********************************************************/
void write1ByteToRingBuffer(uint8_t data)
{
	if(ringBuffer.Length >= RINGBUFFER_LEN) //判断缓冲区是否已满
	{
	return ;
	}
	ringBuffer.Ring_data[ringBuffer.Tail]=data;
	ringBuffer.Tail = (ringBuffer.Tail+1)%RINGBUFFER_LEN;//防止越界非法访问
	ringBuffer.Length++;
	return ;
}




/********************************************************
函数名：  		deleteRingBuffer
作者：
日期：    	2022.10.08
功能：    	删除串口缓冲区中相应长度的数据
输入参数：		要删除的长度
返回值： 		void
修改记录：
**********************************************************/
void deleteRingBuffer(uint16_t size)
{
	if(size >= ringBuffer.Length)
	{
	    initRingBuffer();
	    return;
	}
	for(int i = 0; i < size; i++)
	{
		ringBuffer.Head = (ringBuffer.Head+1)%RINGBUFFER_LEN;//防止越界非法访问
		ringBuffer.Length--;
		return;
	}

}



/********************************************************
函数名：  		read1ByteFromRingBuffer
作者：
日期：    	2022.10.08
功能：    	从串口缓冲区读取1字节数据
输入参数：		position:读取的位置
返回值： 		所在位置的数据(1字节)
修改记录：
**********************************************************/
uint8_t read1ByteFromRingBuffer(uint16_t position)
{
	uint16_t realPosition = (ringBuffer.Head + position) % RINGBUFFER_LEN;

	return ringBuffer.Ring_data[realPosition];
}




/********************************************************
函数名：  		getRingBufferLength
作者：
日期：    	2022.10.08
功能：    	获取串口缓冲区的数据数量
输入参数：
返回值： 		串口缓冲区的数据数量
修改记录：
**********************************************************/
uint16_t getRingBufferLength()
{
	return ringBuffer.Length;
}


/********************************************************
函数名：  		isRingBufferOverflow
作者：
日期：    	2022.10.08
功能：    	判断环形缓冲区是否已满
输入参数：
返回值： 		0:环形缓冲区已满 , 1:环形缓冲区未满
修改记录：
**********************************************************/
uint8_t isRingBufferOverflow()
{
	return ringBuffer.Length < RINGBUFFER_LEN;
}

// ==================== 飞机轨迹显示功能 ====================

// 飞机轨迹相关变量
static int track_points_x[100];  // 轨迹点X坐标数组
static int track_points_y[100];  // 轨迹点Y坐标数组
static uint8_t track_count = 0;  // 当前轨迹点数量
static int last_drone_x = -999;  // 上次飞机X坐标
static int last_drone_y = -999;  // 上次飞机Y坐标
static uint8_t last_fire_id = 0;  // 上次显示的火源ID

/**
 * @brief 坐标系转换：飞机坐标转屏幕坐标
 * @param drone_x 飞机X坐标
 * @param drone_y 飞机Y坐标
 * @param screen_x 转换后的屏幕X坐标指针
 * @param screen_y 转换后的屏幕Y坐标指针
 */
static void convert_to_screen_coords(float drone_x, float drone_y, int *screen_x, int *screen_y)
{
    *screen_x = (int)drone_x + 150;
    *screen_y = 450 - (int)drone_y;
}

/**
 * @brief 添加轨迹点
 * @param x 飞机X坐标
 * @param y 飞机Y坐标
 */
static void add_track_point(int x, int y)
{
    // 检查是否与上一个点相同，避免重复添加
    if (track_count > 0 &&
        track_points_x[track_count-1] == x &&
        track_points_y[track_count-1] == y) {
        return;
    }

    // 如果数组满了，移除最老的点
    if (track_count >= 100) {
        for (uint8_t i = 0; i < 99; i++) {
            track_points_x[i] = track_points_x[i+1];
            track_points_y[i] = track_points_y[i+1];
        }
        track_count = 99;
    }

    // 添加新点
    track_points_x[track_count] = x;
    track_points_y[track_count] = y;
    track_count++;
}

/**
 * @brief 重绘所有轨迹线
 */
static void redraw_all_tracks(void)
{
    char cmd[64];
    int screen_x1, screen_y1, screen_x2, screen_y2;

    // 给串口屏一点时间处理之前的控件更新命令
    HAL_Delay(5);  // 短暂延时，确保控件更新完成

    // 绘制轨迹线（蓝色）
    for (uint8_t i = 1; i < track_count; i++)
    {
        // 轨迹点已经是飞机坐标，需要转换为屏幕坐标
        screen_x1 = track_points_x[i-1] + 150;
        screen_y1 = 450 - track_points_y[i-1];
        screen_x2 = track_points_x[i] + 150;
        screen_y2 = 450 - track_points_y[i];

        sprintf(cmd, "line %d,%d,%d,%d,%d\xFF\xFF\xFF", screen_x1, screen_y1, screen_x2, screen_y2, 1023);  // 青色轨迹，更明显
        tjc_send_string(cmd);

        // 每条线之间稍微延时，避免串口屏处理不过来
        HAL_Delay(1);
    }

    // 绘制当前飞机位置（红色圆点）
    if (track_count > 0) {
        screen_x1 = track_points_x[track_count-1] + 150;
        screen_y1 = 450 - track_points_y[track_count-1];
        sprintf(cmd, "cirs %d,%d,5,%d\xFF\xFF\xFF", screen_x1, screen_y1, 63488);  // 红色圆点
        tjc_send_string(cmd);
    }
}

/**
 * @brief 更新飞机实时位置和轨迹显示
 * @param drone_x 飞机当前X坐标
 * @param drone_y 飞机当前Y坐标
 * @param fire_id 火源ID（1-6）
 */
void update_drone_display_with_fire(float drone_x, float drone_y, uint8_t fire_id)
{
    int current_x = (int)drone_x;
    int current_y = (int)drone_y;

    // 检查位置是否有变化
    if (current_x == last_drone_x && current_y == last_drone_y) {
        return;  // 位置没变化，不需要更新
    }

    // 添加新的轨迹点（在更新控件之前）
    add_track_point(current_x, current_y);

    // ========== 第一步：更新所有文本/控件 ==========
    uint8_t need_redraw = 0;  // 标记是否需要重绘轨迹

    // 更新文本显示（坐标信息）
    display_coordinates("main.t1", current_x, current_y, 0);
    need_redraw = 1;  // 文本更新会影响轨迹显示

    // 显示火源位置（通过透明度控制）- 只在火源ID变化时更新
    if (fire_id >= 1 && fire_id <= 6 && fire_id != last_fire_id)
     {
        // 隐藏上次的火源图片
        if (last_fire_id >= 1 && last_fire_id <= 6)
        {
            set_image_aph(last_fire_id, 0);  // 设置为不可见
        }
        // 显示当前火源
        set_image_aph(fire_id, 127);  // 设置为可见
        last_fire_id = fire_id;  // 更新上次火源ID
        need_redraw = 1;  // 火源更新会影响轨迹显示
    }

    // ========== 第二步：重绘所有轨迹线和图形 ==========
    // 每次控件更新后，立即重新绘制所有轨迹线，把被清除的线条"补回来"
    if (need_redraw) {
        redraw_tracks_after_control_update();
    }

    // 更新上次位置
    last_drone_x = current_x;
    last_drone_y = current_y;
}

/**
 * @brief 更新飞机实时位置和轨迹显示（兼容旧版本）
 * @param drone_x 飞机当前X坐标
 * @param drone_y 飞机当前Y坐标
 */
void update_drone_display(float drone_x, float drone_y)
{
    // 使用外部的drone_data获取fire_id
    extern drone_data_t drone_data;
    update_drone_display_with_fire(drone_x, drone_y, drone_data.fire_id);
}

/**
 * @brief 清除飞机轨迹
 */
void clear_drone_track(void)
{
    track_count = 0;
    last_drone_x = -999;
    last_drone_y = -999;
    last_fire_id = 0;  // 重置火源ID

    // 隐藏所有火源图片
    for (uint8_t i = 1; i <= 6; i++) {
        set_image_aph(i, 0);  // 设置为不可见
    }

    // 发送清屏命令
    tjc_send_string("cls 0\xFF\xFF\xFF");  // 清除背景色
}

/**
 * @brief 定时更新飞机显示（建议在10Hz任务中调用）
 */
void drone_display_task(void)
{
    // 使用外部的drone_data变量
    extern drone_data_t drone_data;  // 需要在头文件中声明

    // 直接更新显示，由调用者控制频率
    update_drone_display_with_fire(drone_data.drone_x, drone_data.drone_y, drone_data.fire_id);
}
