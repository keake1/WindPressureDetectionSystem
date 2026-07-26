;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.6.0 #16555 (Linux)
;--------------------------------------------------------
	.module board
	
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
	.globl _Board_Init
	.globl _Board_ReadAddress
	.globl _Board_RedLedSet
	.globl _Board_GreenLedToggle
	.globl _Board_DelayMs
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
;--------------------------------------------------------
; overlayable items in internal ram
;--------------------------------------------------------
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
_Board_RedLedSet_sloc0_1_0:
	.ds 1
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
;Allocation info for local variables in function 'Board_SetQuasiBidirectional'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:5: static void Board_SetQuasiBidirectional(void)
;	-----------------------------------------
;	 function Board_SetQuasiBidirectional
;	-----------------------------------------
_Board_SetQuasiBidirectional:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:7: P0M0 = 0x00U;
	mov	_P0M0,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:8: P0M1 = 0x00U;
	mov	_P0M1,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:9: P1M0 = 0x00U;
	mov	_P1M0,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:10: P1M1 = 0x00U;
	mov	_P1M1,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:11: P2M0 = 0x00U;
	mov	_P2M0,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:12: P2M1 = 0x00U;
	mov	_P2M1,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:13: P3M0 = 0x00U;
	mov	_P3M0,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:14: P3M1 = 0x00U;
	mov	_P3M1,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:20: P0M0 |= 0x02U;  /* P01 DIG3 */
	orl	_P0M0,#0x02
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:21: P2M0 |= 0xFFU;  /* P20..P27 segments, green LED, DIG1/DIG2 */
	mov	a,_P2M0
	mov	_P2M0,#0xff
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:22: P3M0 |= 0xF0U;  /* P34 red LED, P35/P36/P37 segments */
	orl	_P3M0,#0xf0
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:23: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Board_Init'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:25: void Board_Init(void)
;	-----------------------------------------
;	 function Board_Init
;	-----------------------------------------
_Board_Init:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:27: Board_SetQuasiBidirectional();
	lcall	_Board_SetQuasiBidirectional
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:29: P02 = 1;
;	assignBit
	setb	_P02
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:30: P03 = 1;
;	assignBit
	setb	_P03
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:31: P12 = 1;
;	assignBit
	setb	_P12
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:32: P13 = 1;
;	assignBit
	setb	_P13
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:33: P14 = 1;
;	assignBit
	setb	_P14
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:34: P15 = 1;
;	assignBit
	setb	_P15
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:36: Board_RedLedSet(0U);
	mov	dpl, #0x00
	lcall	_Board_RedLedSet
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:37: P22 = 1;
;	assignBit
	setb	_P22
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:38: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Board_ReadAddress'
;------------------------------------------------------------
;addr          Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:40: uint8_t Board_ReadAddress(void)
;	-----------------------------------------
;	 function Board_ReadAddress
;	-----------------------------------------
_Board_ReadAddress:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:42: uint8_t addr = 0U;
	mov	r7,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:44: if (P15 == 0) { addr |= 0x20U; }
	jb	_P15,00102$
	mov	r7,#0x20
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:45: if (P14 == 0) { addr |= 0x10U; }
	jb	_P14,00104$
	orl	ar7,#0x10
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:46: if (P13 == 0) { addr |= 0x08U; }
	jb	_P13,00106$
	orl	ar7,#0x08
00106$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:47: if (P12 == 0) { addr |= 0x04U; }
	jb	_P12,00108$
	orl	ar7,#0x04
00108$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:48: if (P03 == 0) { addr |= 0x02U; }
	jb	_P03,00110$
	orl	ar7,#0x02
00110$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:49: if (P02 == 0) { addr |= 0x01U; }
	jb	_P02,00112$
	orl	ar7,#0x01
00112$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:51: return addr;
	mov	dpl, r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:52: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Board_RedLedSet'
;------------------------------------------------------------
;on            Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:54: void Board_RedLedSet(uint8_t on)
;	-----------------------------------------
;	 function Board_RedLedSet
;	-----------------------------------------
_Board_RedLedSet:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:56: P34 = (on != 0U) ? 0 : 1;
	mov	a,dpl
	cjne	a,#0x01,00103$
00103$:
	cpl	c
	mov	_Board_RedLedSet_sloc0_1_0,c
	cpl	c
	mov	_P34,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:57: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Board_GreenLedToggle'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:59: void Board_GreenLedToggle(void)
;	-----------------------------------------
;	 function Board_GreenLedToggle
;	-----------------------------------------
_Board_GreenLedToggle:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:61: P22 = !P22;
	cpl	_P22
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:62: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Board_DelayMs'
;------------------------------------------------------------
;ms            Allocated to registers 
;i             Allocated to registers r4 r5 
;j             Allocated to registers r3 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:64: void Board_DelayMs(uint16_t ms)
;	-----------------------------------------
;	 function Board_DelayMs
;	-----------------------------------------
_Board_DelayMs:
	mov	r6, dpl
	mov	r7, dph
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:69: while (ms-- != 0U) {
00103$:
	mov	ar4,r6
	mov	ar5,r7
	dec	r6
	cjne	r6,#0xff,00158$
	dec	r7
00158$:
	mov	a,r4
	orl	a,r5
	jz	00111$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:70: for (i = 0U; i < 500U; ++i) {
	mov	r4,#0x00
	mov	r5,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:71: for (j = 0U; j < 2U; ++j) {
00120$:
	mov	r3,#0x02
00108$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:72: __asm nop __endasm;
	nop	
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:71: for (j = 0U; j < 2U; ++j) {
	djnz	r3,00108$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:70: for (i = 0U; i < 500U; ++i) {
	inc	r4
	cjne	r4,#0x00,00161$
	inc	r5
00161$:
	mov	ar2,r4
	mov	ar3,r5
	clr	c
	mov	a,r2
	subb	a,#0xf4
	mov	a,r3
	subb	a,#0x01
	jc	00120$
	sjmp	00103$
00111$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/board.c:76: }
	ret
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
