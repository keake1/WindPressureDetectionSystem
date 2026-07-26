;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.6.0 #16555 (Linux)
;--------------------------------------------------------
	.module sensor_modbus
	
	.optsdcc -mmcs51 --model-small
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _Uart2_Send
	.globl _Uart2_ReadByte
	.globl _Crc16_Append
	.globl _Crc16_Check
	.globl _Board_RedLedSet
	.globl _P37
	.globl _P36
	.globl _P35
	.globl _P34
	.globl _P33
	.globl _P32
	.globl _P31
	.globl _P30
	.globl _P27
	.globl _P26
	.globl _P25
	.globl _P24
	.globl _P23
	.globl _P22
	.globl _P21
	.globl _P20
	.globl _P17
	.globl _P16
	.globl _P15
	.globl _P14
	.globl _P13
	.globl _P12
	.globl _P11
	.globl _P10
	.globl _P07
	.globl _P06
	.globl _P05
	.globl _P04
	.globl _P03
	.globl _P02
	.globl _P01
	.globl _P00
	.globl _P4M0
	.globl _P4M1
	.globl _P3M0
	.globl _P3M1
	.globl _P2M0
	.globl _P2M1
	.globl _P0M0
	.globl _P0M1
	.globl _P1M0
	.globl _P1M1
	.globl _T2L
	.globl _T2H
	.globl _IP
	.globl _IE
	.globl _P_SW1
	.globl _S2BUF
	.globl _S2CON
	.globl _SBUF
	.globl _SCON
	.globl _AUXR
	.globl _TH1
	.globl _TH0
	.globl _TL1
	.globl _TL0
	.globl _TMOD
	.globl _TCON
	.globl _PCON
	.globl _DPH
	.globl _DPL
	.globl _SP
	.globl _P3
	.globl _P2
	.globl _P1
	.globl _P0
	.globl _SensorModbus_Process_PARM_2
	.globl _SensorModbus_Init
	.globl _SensorModbus_Process
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
_P0	=	0x0080
_P1	=	0x0090
_P2	=	0x00a0
_P3	=	0x00b0
_SP	=	0x0081
_DPL	=	0x0082
_DPH	=	0x0083
_PCON	=	0x0087
_TCON	=	0x0088
_TMOD	=	0x0089
_TL0	=	0x008a
_TL1	=	0x008b
_TH0	=	0x008c
_TH1	=	0x008d
_AUXR	=	0x008e
_SCON	=	0x0098
_SBUF	=	0x0099
_S2CON	=	0x009a
_S2BUF	=	0x009b
_P_SW1	=	0x00a2
_IE	=	0x00a8
_IP	=	0x00b8
_T2H	=	0x00d6
_T2L	=	0x00d7
_P1M1	=	0x0091
_P1M0	=	0x0092
_P0M1	=	0x0093
_P0M0	=	0x0094
_P2M1	=	0x0095
_P2M0	=	0x0096
_P3M1	=	0x00b1
_P3M0	=	0x00b2
_P4M1	=	0x00ba
_P4M0	=	0x00bb
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
_P00	=	0x0080
_P01	=	0x0081
_P02	=	0x0082
_P03	=	0x0083
_P04	=	0x0084
_P05	=	0x0085
_P06	=	0x0086
_P07	=	0x0087
_P10	=	0x0090
_P11	=	0x0091
_P12	=	0x0092
_P13	=	0x0093
_P14	=	0x0094
_P15	=	0x0095
_P16	=	0x0096
_P17	=	0x0097
_P20	=	0x00a0
_P21	=	0x00a1
_P22	=	0x00a2
_P23	=	0x00a3
_P24	=	0x00a4
_P25	=	0x00a5
_P26	=	0x00a6
_P27	=	0x00a7
_P30	=	0x00b0
_P31	=	0x00b1
_P32	=	0x00b2
_P33	=	0x00b3
_P34	=	0x00b4
_P35	=	0x00b5
_P36	=	0x00b6
_P37	=	0x00b7
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	.area REG_BANK_0	(REL,OVR,DATA)
	.ds 8
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	.area DSEG    (DATA)
_g_rx:
	.ds 24
_g_rx_len:
	.ds 1
_g_rx_overflow:
	.ds 1
_SensorModbus_ReplyPressure_PARM_2:
	.ds 2
_SensorModbus_ReplyPressure_tx_10000_23:
	.ds 8
_SensorModbus_HandleFrame_PARM_2:
	.ds 2
_SensorModbus_Process_PARM_2:
	.ds 2
_SensorModbus_Process_b_10000_37:
	.ds 1
;--------------------------------------------------------
; overlayable items in internal ram
;--------------------------------------------------------
;--------------------------------------------------------
; indirectly addressable internal ram data
;--------------------------------------------------------
	.area ISEG    (DATA)
;--------------------------------------------------------
; absolute internal ram data
;--------------------------------------------------------
	.area IABS    (ABS,DATA)
	.area IABS    (ABS,DATA)
;--------------------------------------------------------
; bit data
;--------------------------------------------------------
	.area BSEG    (BIT)
;--------------------------------------------------------
; paged external ram data
;--------------------------------------------------------
	.area PSEG    (PAG,XDATA)
;--------------------------------------------------------
; uninitialized external ram data
;--------------------------------------------------------
	.area XSEG    (XDATA)
;--------------------------------------------------------
; absolute external ram data
;--------------------------------------------------------
	.area XABS    (ABS,XDATA)
;--------------------------------------------------------
; initialized external ram data
;--------------------------------------------------------
	.area XISEG   (XDATA)
	.area HOME    (CODE)
	.area GSINIT0 (CODE)
	.area GSINIT1 (CODE)
	.area GSINIT2 (CODE)
	.area GSINIT3 (CODE)
	.area GSINIT4 (CODE)
	.area GSINIT5 (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area CSEG    (CODE)
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME    (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area GSINIT  (CODE)
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME    (CODE)
	.area HOME    (CODE)
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CSEG    (CODE)
;------------------------------------------------------------
;Allocation info for local variables in function 'SensorModbus_Init'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:40: void SensorModbus_Init(void)
;	-----------------------------------------
;	 function SensorModbus_Init
;	-----------------------------------------
_SensorModbus_Init:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:42: g_rx_len = 0U;
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:43: g_rx_overflow = 0U;
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:46: TMOD = (TMOD & 0xF0U) | 0x01U;
	clr	a
	mov	_g_rx_len,a
	mov	_g_rx_overflow,a
	mov	a,_TMOD
	anl	a,#0xf0
	orl	a,#0x01
	mov	_TMOD,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:47: AUXR &= (uint8_t)~0x80U;            /* T0x12 = 0：12T 模式 */
	anl	_AUXR,#0x7f
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:48: TCON &= (uint8_t)~(TCON_TR0 | TCON_TF0);
	anl	_TCON,#0xcf
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:49: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'SensorModbus_RestartGapTimer'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:52: static void SensorModbus_RestartGapTimer(void)
;	-----------------------------------------
;	 function SensorModbus_RestartGapTimer
;	-----------------------------------------
_SensorModbus_RestartGapTimer:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:54: TCON &= (uint8_t)~(TCON_TR0 | TCON_TF0);
	anl	_TCON,#0xcf
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:55: TH0 = MB_GAP_RELOAD_H;
	mov	_TH0,#0xf1
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:56: TL0 = MB_GAP_RELOAD_L;
	mov	_TL0,#0x9a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:57: TCON |= TCON_TR0;
	orl	_TCON,#0x10
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:58: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'SensorModbus_ReplyPressure'
;------------------------------------------------------------
;pressure      Allocated with name '_SensorModbus_ReplyPressure_PARM_2'
;addr          Allocated to registers r7 
;tx            Allocated with name '_SensorModbus_ReplyPressure_tx_10000_23'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:60: static void SensorModbus_ReplyPressure(uint8_t addr, uint16_t pressure)
;	-----------------------------------------
;	 function SensorModbus_ReplyPressure
;	-----------------------------------------
_SensorModbus_ReplyPressure:
	mov	r7, dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:64: tx[0] = addr;
	mov	_SensorModbus_ReplyPressure_tx_10000_23,r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:65: tx[1] = 0x03U;
	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0001),#0x03
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:66: tx[2] = 0x03U;                       /* 字节数：型号 + 2 字节数据 */
	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0002),#0x03
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:67: tx[3] = SENSOR_TYPE_WIND_PRESSURE;   /* 0x02 */
	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0003),#0x02
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:68: tx[4] = (uint8_t)(pressure >> 8);
	mov	r7,(_SensorModbus_ReplyPressure_PARM_2 + 1)
	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0004),r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:69: tx[5] = (uint8_t)pressure;
	mov	r7,_SensorModbus_ReplyPressure_PARM_2
	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0005),r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:70: Crc16_Append(tx, 6U);
	mov	_Crc16_Append_PARM_2,#0x06
	mov	dptr,#_SensorModbus_ReplyPressure_tx_10000_23
	mov	b, #0x40
	lcall	_Crc16_Append
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:71: Uart2_Send(tx, sizeof(tx));
	mov	_Uart2_Send_PARM_2,#0x08
	mov	dptr,#_SensorModbus_ReplyPressure_tx_10000_23
	mov	b, #0x40
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:72: }
	ljmp	_Uart2_Send
;------------------------------------------------------------
;Allocation info for local variables in function 'SensorModbus_HandleFrame'
;------------------------------------------------------------
;pressure      Allocated with name '_SensorModbus_HandleFrame_PARM_2'
;my_addr       Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:74: static void SensorModbus_HandleFrame(uint8_t my_addr, uint16_t pressure)
;	-----------------------------------------
;	 function SensorModbus_HandleFrame
;	-----------------------------------------
_SensorModbus_HandleFrame:
	mov	r7, dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:77: if (g_rx_len != 8U) {
	mov	a,#0x08
	cjne	a,_g_rx_len,00214$
	sjmp	00102$
00214$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:78: return;
	ret
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:81: if (g_rx[0] != my_addr) {
	mov	a,r7
	cjne	a,_g_rx,00215$
	sjmp	00104$
00215$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:82: return;
	ret
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:85: if (Crc16_Check(g_rx, 8U) == 0U) {
	mov	_Crc16_Check_PARM_2,#0x08
	mov	dptr,#_g_rx
	mov	b, #0x40
	push	ar7
	lcall	_Crc16_Check
	mov	a, dpl
	pop	ar7
	jnz	00106$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:86: return;
	ret
00106$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:96: if ((g_rx[1] == 0x03U) &&
	mov	r6,(_g_rx + 0x0001)
	cjne	r6,#0x03,00108$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:97: (g_rx[2] == 0x00U) &&
	mov	a,(_g_rx + 0x0002)
	jnz	00108$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:98: ((g_rx[3] == 0x00U) || (g_rx[3] == 0x01U)) &&
	mov	a,(_g_rx + 0x0003)
	mov	r5,a
	jz	00112$
	cjne	r5,#0x01,00108$
00112$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:99: (g_rx[4] == 0x00U) && (g_rx[5] == 0x01U)) {
	mov	a,(_g_rx + 0x0004)
	jnz	00108$
	mov	a,#0x01
	cjne	a,(_g_rx + 0x0005),00108$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:100: SensorModbus_ReplyPressure(my_addr, pressure);
	mov	_SensorModbus_ReplyPressure_PARM_2,_SensorModbus_HandleFrame_PARM_2
	mov	(_SensorModbus_ReplyPressure_PARM_2 + 1),(_SensorModbus_HandleFrame_PARM_2 + 1)
	mov	dpl, r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:101: return;
	ljmp	_SensorModbus_ReplyPressure
00108$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:105: if ((g_rx[1] == 0x06U) &&
	cjne	r6,#0x06,00119$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:106: (g_rx[2] == 0x00U) && (g_rx[3] == 0x04U) &&
	mov	a,(_g_rx + 0x0002)
	jnz	00119$
	mov	a,#0x04
	cjne	a,(_g_rx + 0x0003),00119$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:107: (g_rx[4] == 0x00U)) {
	mov	a,(_g_rx + 0x0004)
	jnz	00119$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:108: Board_RedLedSet((g_rx[5] != 0U) ? 1U : 0U);
	mov	a,(_g_rx + 0x0005)
	jz	00121$
	mov	a,#0x01
00121$:
	mov	r7,a
	mov	dpl,r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:110: }
	ljmp	_Board_RedLedSet
00119$:
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'SensorModbus_Process'
;------------------------------------------------------------
;pressure      Allocated with name '_SensorModbus_Process_PARM_2'
;my_addr       Allocated to registers r7 
;b             Allocated with name '_SensorModbus_Process_b_10000_37'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:112: void SensorModbus_Process(uint8_t my_addr, uint16_t pressure)
;	-----------------------------------------
;	 function SensorModbus_Process
;	-----------------------------------------
_SensorModbus_Process:
	mov	r7, dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:117: while (Uart2_ReadByte(&b) != 0U) {
00104$:
	mov	dptr,#_SensorModbus_Process_b_10000_37
	mov	b, #0x40
	push	ar7
	lcall	_Uart2_ReadByte
	mov	a, dpl
	pop	ar7
	jz	00106$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:118: if (g_rx_len < MB_RX_BUF_SIZE) {
	mov	a,#0x100 - 0x18
	add	a,_g_rx_len
	jc	00102$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:119: g_rx[g_rx_len++] = b;
	mov	r6,_g_rx_len
	inc	_g_rx_len
	mov	a,r6
	add	a, #_g_rx
	mov	r0,a
	mov	@r0,_SensorModbus_Process_b_10000_37
	sjmp	00103$
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:121: g_rx_overflow = 1U;          /* 超长帧：整帧作废 */
	mov	_g_rx_overflow,#0x01
00103$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:123: SensorModbus_RestartGapTimer();
	push	ar7
	lcall	_SensorModbus_RestartGapTimer
	pop	ar7
	sjmp	00104$
00106$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:127: if ((g_rx_len != 0U) && ((TCON & TCON_TF0) != 0U)) {
	mov	a,_g_rx_len
	jz	00112$
	mov	a,_TCON
	jnb	acc.5,00112$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:128: TCON &= (uint8_t)~(TCON_TR0 | TCON_TF0);
	anl	_TCON,#0xcf
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:129: if (g_rx_overflow == 0U) {
	mov	a,_g_rx_overflow
	jnz	00108$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:130: SensorModbus_HandleFrame(my_addr, pressure);
	mov	_SensorModbus_HandleFrame_PARM_2,_SensorModbus_Process_PARM_2
	mov	(_SensorModbus_HandleFrame_PARM_2 + 1),(_SensorModbus_Process_PARM_2 + 1)
	mov	dpl, r7
	lcall	_SensorModbus_HandleFrame
00108$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:132: g_rx_len = 0U;
	mov	_g_rx_len,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:133: g_rx_overflow = 0U;
	mov	_g_rx_overflow,#0x00
00112$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:135: }
	ret
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
