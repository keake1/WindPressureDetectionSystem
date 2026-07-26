;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.6.0 #16555 (Linux)
;--------------------------------------------------------
	.module uart
	
	.optsdcc -mmcs51 --model-small
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
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
	.globl _Uart2_Send_PARM_2
	.globl _Uart1_Send_PARM_2
	.globl _Uart_Init
	.globl _Uart1_SendByte
	.globl _Uart1_ReadByte
	.globl _Uart2_SendByte
	.globl _Uart2_ReadByte
	.globl _Uart1_Send
	.globl _Uart2_Send
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
_Uart1_Send_PARM_2:
	.ds 1
_Uart2_Send_PARM_2:
	.ds 1
;--------------------------------------------------------
; overlayable items in internal ram
;--------------------------------------------------------
	.area	OSEG    (OVR,DATA)
	.area	OSEG    (OVR,DATA)
	.area	OSEG    (OVR,DATA)
	.area	OSEG    (OVR,DATA)
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
;Allocation info for local variables in function 'Uart_Reload'
;------------------------------------------------------------
;baud          Allocated to registers 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:10: static uint16_t Uart_Reload(uint32_t baud)
;	-----------------------------------------
;	 function Uart_Reload
;	-----------------------------------------
_Uart_Reload:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
	mov	__divulong_PARM_2,dpl
	mov	(__divulong_PARM_2 + 1),dph
	mov	(__divulong_PARM_2 + 2),b
	mov	(__divulong_PARM_2 + 3),a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:12: return (uint16_t)(65536UL - (FOSC_HZ / 4UL / baud));
	mov	dptr,#0x3000
	mov	b, #0x2a
	clr	a
	lcall	__divulong
	mov	r4, dpl
	mov	r5, dph
	clr	c
	clr	a
	subb	a,r4
	mov	dpl,a
	clr	a
	subb	a,r5
	mov	dph,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:13: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Uart_Init'
;------------------------------------------------------------
;reload2       Allocated to registers r6 r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:15: void Uart_Init(void)
;	-----------------------------------------
;	 function Uart_Init
;	-----------------------------------------
_Uart_Init:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:31: uint16_t reload2 = Uart_Reload(CONTROLLER_BAUD);  // 9600 重装值
	mov	dptr,#0x2580
	clr	a
	mov	b,a
	lcall	_Uart_Reload
	mov	r6, dpl
	mov	r7, dph
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:33: P_SW1 &= 0x3FU; /* UART1 on P30/RXD and P31/TXD. */
	anl	_P_SW1,#0x3f
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:34: SCON = SCON_MODE1 | SCON_REN;
	mov	_SCON,#0x50
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:35: S2CON = S2CON_REN;
	mov	_S2CON,#0x10
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:38: TMOD = (TMOD & 0x0FU) | 0x20U;            // Timer 1, 8-bit auto-reload (mode 2)
	mov	a,_TMOD
	anl	a,#0x0f
	orl	a,#0x20
	mov	_TMOD,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:39: TH1 = (uint8_t)(256U - (FOSC_HZ / 32U / PRESSURE_BAUD));
	mov	_TH1,#0xfd
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:41: TL1 = TH1;
	mov	_TL1,_TH1
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:42: TCON |= 0x40U;                             // TR1 = 1：启动 Timer 1
	orl	_TCON,#0x40
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:45: T2H = (uint8_t)(reload2 >> 8);
	mov	_T2H,r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:46: T2L = (uint8_t)reload2;
	mov	_T2L,r6
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:48: AUXR = (AUXR & (uint8_t)~0x01U) | 0x54U;
	mov	a,#0xfe
	anl	a,_AUXR
	orl	a,#0x54
	mov	_AUXR,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:50: SCON &= (uint8_t)~(SCON_RI | SCON_TI);
	anl	_SCON,#0xfc
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:51: S2CON &= (uint8_t)~(S2CON_RI | S2CON_TI);
	anl	_S2CON,#0xfc
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:52: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Uart1_SendByte'
;------------------------------------------------------------
;value         Allocated to registers 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:54: void Uart1_SendByte(uint8_t value)
;	-----------------------------------------
;	 function Uart1_SendByte
;	-----------------------------------------
_Uart1_SendByte:
	mov	_SBUF,dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:57: while ((SCON & SCON_TI) == 0U) {
00101$:
	mov	a,_SCON
	jnb	acc.1,00101$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:59: SCON &= (uint8_t)~SCON_TI;
	anl	_SCON,#0xfd
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:60: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Uart1_ReadByte'
;------------------------------------------------------------
;value         Allocated to registers r5 r6 r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:62: uint8_t Uart1_ReadByte(uint8_t *value)
;	-----------------------------------------
;	 function Uart1_ReadByte
;	-----------------------------------------
_Uart1_ReadByte:
	mov	r5, dpl
	mov	r6, dph
	mov	r7, b
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:64: if ((SCON & SCON_RI) == 0U) {
	mov	a,_SCON
	jb	acc.0,00102$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:65: return 0U;
	mov	dpl, #0x00
	ret
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:68: *value = SBUF;
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	mov	a,_SBUF
	lcall	__gptrput
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:69: SCON &= (uint8_t)~SCON_RI;
	anl	_SCON,#0xfe
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:70: return 1U;
	mov	dpl, #0x01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:71: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Uart2_SendByte'
;------------------------------------------------------------
;value         Allocated to registers 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:73: void Uart2_SendByte(uint8_t value)
;	-----------------------------------------
;	 function Uart2_SendByte
;	-----------------------------------------
_Uart2_SendByte:
	mov	_S2BUF,dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:76: while ((S2CON & S2CON_TI) == 0U) {
00101$:
	mov	a,_S2CON
	jnb	acc.1,00101$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:78: S2CON &= (uint8_t)~S2CON_TI;
	anl	_S2CON,#0xfd
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:79: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Uart2_ReadByte'
;------------------------------------------------------------
;value         Allocated to registers r5 r6 r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:81: uint8_t Uart2_ReadByte(uint8_t *value)
;	-----------------------------------------
;	 function Uart2_ReadByte
;	-----------------------------------------
_Uart2_ReadByte:
	mov	r5, dpl
	mov	r6, dph
	mov	r7, b
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:83: if ((S2CON & S2CON_RI) == 0U) {
	mov	a,_S2CON
	jb	acc.0,00102$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:84: return 0U;
	mov	dpl, #0x00
	ret
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:87: S2CON &= (uint8_t)~S2CON_RI;
	anl	_S2CON,#0xfe
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:88: *value = S2BUF;
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	mov	a,_S2BUF
	lcall	__gptrput
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:89: return 1U;
	mov	dpl, #0x01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:90: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Uart1_Send'
;------------------------------------------------------------
;len           Allocated with name '_Uart1_Send_PARM_2'
;data          Allocated to registers r5 r6 r7 
;i             Allocated to registers r4 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:92: void Uart1_Send(const uint8_t *data, uint8_t len)
;	-----------------------------------------
;	 function Uart1_Send
;	-----------------------------------------
_Uart1_Send:
	mov	r5, dpl
	mov	r6, dph
	mov	r7, b
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:95: for (i = 0U; i < len; ++i) {
	mov	r4,#0x00
00103$:
	clr	c
	mov	a,r4
	subb	a,_Uart1_Send_PARM_2
	jnc	00105$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:96: Uart1_SendByte(data[i]);
	mov	a,r4
	add	a, r5
	mov	r1,a
	clr	a
	addc	a, r6
	mov	r2,a
	mov	ar3,r7
	mov	dpl,r1
	mov	dph,r2
	mov	b,r3
	lcall	__gptrget
	mov	dpl,a
	push	ar7
	push	ar6
	push	ar5
	push	ar4
	lcall	_Uart1_SendByte
	pop	ar4
	pop	ar5
	pop	ar6
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:95: for (i = 0U; i < len; ++i) {
	inc	r4
	sjmp	00103$
00105$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:98: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Uart2_Send'
;------------------------------------------------------------
;len           Allocated with name '_Uart2_Send_PARM_2'
;data          Allocated to registers r5 r6 r7 
;i             Allocated to registers r4 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:100: void Uart2_Send(const uint8_t *data, uint8_t len)
;	-----------------------------------------
;	 function Uart2_Send
;	-----------------------------------------
_Uart2_Send:
	mov	r5, dpl
	mov	r6, dph
	mov	r7, b
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:103: for (i = 0U; i < len; ++i) {
	mov	r4,#0x00
00103$:
	clr	c
	mov	a,r4
	subb	a,_Uart2_Send_PARM_2
	jnc	00105$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:104: Uart2_SendByte(data[i]);
	mov	a,r4
	add	a, r5
	mov	r1,a
	clr	a
	addc	a, r6
	mov	r2,a
	mov	ar3,r7
	mov	dpl,r1
	mov	dph,r2
	mov	b,r3
	lcall	__gptrget
	mov	dpl,a
	push	ar7
	push	ar6
	push	ar5
	push	ar4
	lcall	_Uart2_SendByte
	pop	ar4
	pop	ar5
	pop	ar6
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:103: for (i = 0U; i < len; ++i) {
	inc	r4
	sjmp	00103$
00105$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:106: }
	ret
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
