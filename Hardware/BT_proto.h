/* 蓝牙/网页串口协议解析模块接口。主循环周期性调用 BT_Process()，它会从 Serial 环形缓冲区提取 [ ... ] 数据包并更新小车控制变量。 */
#ifndef __BT_PROTO_H
#define __BT_PROTO_H

#include <stdint.h>

void BT_Process(void);

#endif
