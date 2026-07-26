#include "digital_cube.h"


sbit P33_LED_DIG4 = P3^3;


unsigned char display_buf[4] = {0};

code unsigned char data_[] = {
	0x3F,  //"0"
	0x06,  //"1"
	0x5B,  //"2"
	0x4F,  //"3"
	0x66,  //"4"
	0x6D,  //"5"
	0x7D,  //"6"
	0x07,  //"7"
	0x7F,  //"8"
	0x6F,  //"9"
	0x77,  //"A"
	0x7C,  //"B"
	0x39,  //"C"
	0x5E,  //"D"
	0x79,  //"E"
	0x71,  //"F"
	0x76,  //"H"
	0x38,  //"L"
	0x37,  //"n"
	0x3E,  //"u"
	0x73,  //"P"
	0x5C,  //"o"
	0x40,  //"-"
	0x00   //��
};


void digital_cube_init(void)
{

     // --- 新增：配置 P3.3 为推挽输出 (也可以在 main 里配，这里重复配也没事) ---
    P3M0 |= 0x01 << 3;
    P3M1 &= ~(0x01 << 3);
    P33_LED_DIG4 = 0; // 默认熄灭 // DIG4
    // --- 结束 ---

    // ����P0.1Ϊ�������
    P0M0 |= 0x01 << 1;
    P0M1 &= ~(0x01 << 1);
    P01 = 0;  // DIG3

    // ����P2.6Ϊ�������
    P2M0 |= 0x01 << 6;
    P2M1 &= ~(0x01 << 6);
    P26 = 0;  // DIG2

    // ����P2.3Ϊ�������
    P2M0 |= 0x01 << 3;
    P2M1 &= ~(0x01 << 3);
    P23 = 0;  // DIG1

    // ����P2.4Ϊ�������
    P2M0 |= 0x01 << 4;
    P2M1 &= ~(0x01 << 4);
    P24 = 0;  // LEDA

    // ����P2.7Ϊ�������
    P2M0 |= 0x01 << 7;
    P2M1 &= ~(0x01 << 7);
    P27 = 0;  // LEDB

    // ����P3.6Ϊ�������
    P3M0 |= 0x01 << 6;
    P3M1 &= ~(0x01 << 6);
    P36 = 0;  // LEDC

    // ����P2.0Ϊ�������
    P2M0 |= 0x01 << 0;
    P2M1 &= ~(0x01 << 0);
    P20 = 0;  // LEDD

    // ����P2.1Ϊ�������
    P2M0 |= 0x01 << 1;
    P2M1 &= ~(0x01 << 1);
    P21 = 0;  // LEDE

    // ����P2.5Ϊ�������
    P2M0 |= 0x01 << 5;
    P2M1 &= ~(0x01 << 5);
    P25 = 0;  // LEDF

    // ����P3.5Ϊ�������
    P3M0 |= 0x01 << 5;
    P3M1 &= ~(0x01 << 5);
    P35 = 0;  // LEDG

   

    P37 = 0;

}

void digital_cube_ShowOne(unsigned char num)
{
    // ���� data[num] ��ÿһλ��������
    P24 = (data_[num] & 0x01) ;
    P27 = (data_[num] & 0x02) ;
    P36 = (data_[num] & 0x04) ;
    P20 = (data_[num] & 0x08) ;
    P21 = (data_[num] & 0x10) ;
    P25 = (data_[num] & 0x20) ;
    P35 = (data_[num] & 0x40) ;
}


void Digital_Tube_Show(unsigned char num, unsigned char loc)
{
	
	P01 = 1;
    P26 = 1;
    P23 = 1;
    P33_LED_DIG4 = 1; // 新增：关闭第4位
    switch (loc)
    {
    case 0:
        P33_LED_DIG4 = 1; // 选中
        P01 = 1;
		P26 = 1;
		P23 = 0;
        
        break;
    case 1:
        P01 = 1;
		P26 = 0;
		P23 = 1;
        P33_LED_DIG4 = 1; // 选中
        break;
    case 2:
        P01 = 0;
		P26 = 1;
		P23 = 1;
        P33_LED_DIG4 = 1; // 选中
        break;
    case 3: // 个位 (新增的第4位)
        P01 = 1;
		P26 = 1;
		P23 = 1;
        P33_LED_DIG4 = 0; // 选中
        break;    
    
    default:
        break;
    }
    digital_cube_ShowOne(num);
}

void Digital_Tube_ShowNum(int num)
{
	if (num >= 0) {
        // 拆分 4 位数
		display_buf[0] = (num / 1000) % 10; // 千位
        display_buf[1] = (num / 100) % 10;  // 百位
        display_buf[2] = (num / 10) % 10;   // 十位
        display_buf[3] = (num % 10);        // 个位
	}
	else {
        // 负数处理 (简单的逻辑：第一位显示负号，后面显示3位数字)
		display_buf[0] = 22; // "-" 号
        display_buf[1] = (-num / 100) % 10;
        display_buf[2] = (-num / 10) % 10;
        display_buf[3] = (-num % 10);
	}
		
}


void Digital_Tube_flash(void)
{
	
	static unsigned char p = 0;
	Digital_Tube_Show(display_buf[p], p);
	p++;
	if (p >= 4)
		p = 0;

}

