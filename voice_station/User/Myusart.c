// #include "Myusart.h"

// // 全局变量
// uint8_t direction;
// float distance;   
// float radius; //半径
// uint8_t shape;  //形状
// float height; //高度
// uint8_t receive_data[20];
// uint8_t RxBuffer[10];
// uint8_t data_index = 0;
// uint8_t current_state = 0;  // 0:等待帧头 1:等待命令 2:等待数据 3:等待帧尾
// extern uint8_t test;
// // 发送缓冲区
// uint8_t send_buffer[20];
// uint8_t send_length = 0;

// void Myusart_Init(void)
// {
//     HAL_UART_Receive_IT(&huart1, RxBuffer, 1);

// }

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// {
//     if(huart->Instance == USART1)
//     {
       
//         //HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        
//         //HAL_UART_Transmit_IT(&huart1, &test, 1);

//         uint8_t data = RxBuffer[0];
//         parse_data(data);
//         HAL_UART_Receive_IT(&huart1, RxBuffer, 1);
//     }
// }

// void parse_data(uint8_t data)
// {
//     static uint8_t cmd = 0;
//     static uint32_t last_time = 0;
//     uint32_t current_time = HAL_GetTick();
    
//     // 检查超时
//     if(current_time - last_time > 100 && current_state != 0)  // 100ms超时
//     {
//         current_state = 0;  // 超时重置状态机
//     }
//     last_time = current_time;
    
//     switch(current_state) 
//     {
//         case 0: // 等待帧头
//             if(data == 0xF4) 
//             {
//                 reset_received_data();
//                 current_state = 1;
//             }
//             break;
            
//         case 1: // 等待命令
//             if(data == 0x06)
//             {
//                 current_state = 2;
//             } 
//             else
//             {
//                 current_state = 0;
//             }
//             break;
            
//         case 2: // 等待数据
//             cmd = data;
//             current_state = 3;
//             break;
            
//         case 3: // 等待帧尾
//             if(data == 0xFB) 
//             {
//                 // 根据命令解析数据
//                 switch(cmd)
//                 {/*
//                     case 0x0C: // 一米高度
//                         height = 1;
//                         break;
//                     case 0x0D: // 零点五米高度
//                         height = 0.5;
//                         break;
//                     case 0x0E: // 前方零点五米/前方一米
//                         direction = 1; // 前方
//                         distance = 0.5;
//                         break;
//                     case 0x0F: // 前方一米
//                         direction = 1; // 前方
//                         distance = 1;
//                         break;
//                     case 0x10: // 后方零点五米
//                         direction = 2; // 后方
//                         distance = 0.5;
//                         break;
//                     case 0x11: // 后方一米
//                         direction = 2; // 后方
//                         distance = 1;
//                         break;
//                     case 0x12: // 直径为零点五米
//                         radius = 0.25; // 半径
//                         shape = 1; // 圆
//                         break;
//                     case 0x13: // 直径为一米
//                         radius = 0.5; // 半径
//                         shape = 1; // 圆
//                         break;
//                   default:
//                      break;
// 					 */
//                     // 高度命令
//                     case 0x03: // 零点五米高度
//                         height = 0.5;
//                         break;
//                     case 0x04: // 一米高度
//                         height = 1.0;
//                         break;
//                     case 0x05: // 一点五米高度
//                         height = 1.5;
//                         break;
//                     case 0x06: // 两米高度
//                         height = 2.0;
//                         break;
                        
//                     // 前方距离命令
//                     case 0x07: // 前方零点五米
//                         direction = 1; // 前方
//                         distance = 0.5;
//                         break;
//                     case 0x08: // 前方一米
//                         direction = 1; // 前方
//                         distance = 1.0;
//                         break;
//                     case 0x09: // 前方一点五米
//                         direction = 1; // 前方
//                         distance = 1.5;
//                         break;
//                     case 0x0A: // 前方两米
//                         direction = 1; // 前方
//                         distance = 2.0;
//                         break;
                        
//                     // 后方距离命令
//                     case 0x0B: // 后方零点五米
//                         direction = 2; // 后方
//                         distance = 0.5;
//                         break;
//                     case 0x0C: // 后方一米
//                         direction = 2; // 后方
//                         distance = 1.0;
//                         break;
//                     case 0x0D: // 后方一点五米
//                         direction = 2; // 后方
//                         distance = 1.5;
//                         break;
//                     case 0x0E: // 后方两米
//                         direction = 2; // 后方
//                         distance = 2.0;
//                         break;
                        
//                     // 左方距离命令
//                     case 0x0F: // 左方零点五米
//                         direction = 3; // 左方
//                         distance = 0.5;
//                         break;
//                     case 0x10: // 左方一米
//                         direction = 3; // 左方
//                         distance = 1.0;
//                         break;
//                     case 0x11: // 左方一点五米
//                         direction = 3; // 左方
//                         distance = 1.5;
//                         break;
//                     case 0x12: // 左方两米
//                         direction = 3; // 左方
//                         distance = 2.0;
//                         break;
                        
//                     // 右方距离命令
//                     case 0x13: // 右方零点五米
//                         direction = 4; // 右方
//                         distance = 0.5;
//                         break;
//                     case 0x14: // 右方一米
//                         direction = 4; // 右方
//                         distance = 1.0;
//                         break;
//                     case 0x15: // 右方一点五米
//                         direction = 4; // 右方
//                         distance = 1.5;
//                         break;
//                     case 0x16: // 右方两米
//                         direction = 4; // 右方
//                         distance = 2.0;
//                         break;
                        
//                     // 圆形命令 - 直径
//                     case 0x17: // 直径零点五米的圆
//                         radius = 0.25; // 半径 = 直径/2
//                         shape = 1; // 圆
//                         break;
//                     case 0x18: // 直径一米的圆
//                         radius = 0.5; // 半径 = 直径/2
//                         shape = 1; // 圆
//                         break;
//                     case 0x19: // 直径一点五米的圆
//                         radius = 0.75; // 半径 = 直径/2
//                         shape = 1; // 圆
//                         break;
//                     case 0x1A: // 直径两米的圆
//                         radius = 1.0; // 半径 = 直径/2
//                         shape = 1; // 圆
//                         break;
                        
//                     // 圆形命令 - 半径
//                     case 0x1B: // 半径零点五米的圆
//                         radius = 0.5;
//                         shape = 1; // 圆
//                         break;
//                     case 0x1C: // 半径一米的圆
//                         radius = 1.0;
//                         shape = 1; // 圆
//                         break;
//                     case 0x1D: // 半径一点五米的圆
//                         radius = 1.5;
//                         shape = 1; // 圆
//                         break;
//                     case 0x1E: // 半径两米的圆
//                         radius = 2.0;
//                         shape = 1; // 圆
//                         break;
                        
//                     // 正方形命令
//                     case 0x1F: // 边长零点五米的正方形
//                         radius = 0.5; // 使用radius存储边长
//                         shape = 2; // 正方形
//                         break;
//                     case 0x20: // 边长一米的正方形
//                         radius = 1.0;
//                         shape = 2; // 正方形
//                         break;
//                     case 0x21: // 边长一点五米的正方形
//                         radius = 1.5;
//                         shape = 2; // 正方形
//                         break;
//                     case 0x22: // 边长两米的正方形
//                         radius = 2.0;
//                         shape = 2; // 正方形
//                         break;
                        
//                     // 三角形命令
//                     case 0x23: // 边长零点五米的三角形
//                         radius = 0.5; // 使用radius存储边长
//                         shape = 3; // 三角形
//                         break;
//                     case 0x24: // 边长一米的三角形
//                         radius = 1.0;
//                         shape = 3; // 三角形
//                         break;
//                     case 0x25: // 边长一点五米的三角形
//                         radius = 1.5;
//                         shape = 3; // 三角形
//                         break;
//                     case 0x26: // 边长两米的三角形
//                         radius = 2.0;
//                         shape = 3; // 三角形
//                         break;
                        
//                     default:
//                         break;
//                 }
//                 // 解析完成后打包并发送数据
              
//                 packet_and_send(); 
//                 current_state = 0;  // 只在收到正确帧尾时重置
//             }    
//             else
//             {
//                 current_state = 0;  // 或者在收到错误数据时也重置
//             }
//             break;
//     }
// }

// void packet_and_send()
// {
//     uint8_t index = 0;
//     char float_str[4]; // 存储转换后的浮点数字符串
    
//     // 帧头 '!'
//     send_buffer[index++] = '!';
    
//     // 方向 
//     float dir_float = (float)direction; 
//     float_to_fixed_string(dir_float, float_str);
//     send_buffer[index++] = float_str[0]; 
//     send_buffer[index++] = float_str[1]; 
//     send_buffer[index++] = float_str[2]; 

    
//     // 距离 
//     float_to_fixed_string(distance, float_str);
//     send_buffer[index++] = float_str[0]; 
//     send_buffer[index++] = float_str[1]; 
//     send_buffer[index++] = float_str[2]; 
    
//     // 半径 
//     float_to_fixed_string(radius, float_str);
//     send_buffer[index++] = float_str[0]; 
//     send_buffer[index++] = float_str[1]; 
//     send_buffer[index++] = float_str[2]; 
    
//     // 形状 
//     float shape_float = (float)shape; 
//     float_to_fixed_string(shape_float, float_str);
//     send_buffer[index++] = float_str[0]; 
//     send_buffer[index++] = float_str[1]; 
//     send_buffer[index++] = float_str[2]; 
    
//     // 高度 
//     float_to_fixed_string(height, float_str);
//     send_buffer[index++] = float_str[0]; 
//     send_buffer[index++] = float_str[1]; 
//     send_buffer[index++] = float_str[2]; 
    
//     // 帧尾
//     send_buffer[index++] = '#';
//     send_length = index;
//     while(huart1.gState == HAL_UART_STATE_BUSY_TX);
    
//     // 中断发送数据
//     HAL_UART_Transmit_IT(&huart2, send_buffer, send_length);
// }

// void reset_received_data()
//  {
//     direction = 0;
//     distance = 0;
//     radius = 0;
//     shape = 0;
//     height = 0;
// }

// void float_to_fixed_string(float value, char* buffer) {
//     // 取整数部分
//     int whole = (int)value;
    
//     // 计算小数部分并四舍五入到两位
//     int decimal = (int)(value * 100 + 0.5) % 100;  // +0.5用于四舍五入
    
//     // 格式化：整数部分1位，小数部分固定2位
//     buffer[0] = whole + '0';  // 整数部分转为字符
//     buffer[1] = (decimal / 10) + '0';  // 小数第一位
//     buffer[2] = (decimal % 10) + '0';  // 小数第二位
//     buffer[3] = '\0';  // 字符串结束符
// }