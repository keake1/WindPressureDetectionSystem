#ifndef __DIGITAL_TUBE_H__
#define __DIGITAL_TUBE_H__

/* 初始化三位动态数码管。 */
void DigitalTube_Init(void);

/* 设置待显示数值：输入单位为 ×10，保留一位小数。 */
void DigitalTube_SetValue(int value_x10);

/* 每 1ms 调用一次，完成一位动态扫描。 */
void DigitalTube_Scan1ms(void);

#endif /* __DIGITAL_TUBE_H__ */
