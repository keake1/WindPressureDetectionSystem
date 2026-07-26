;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.6.0 #16555 (Linux)
;--------------------------------------------------------
	.module display
	
	.optsdcc -mmcs51 --model-small
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _SensorModbus_Process
	.globl _Pressure_GetValue
	.globl _Pressure_ProcessRx
	.globl _Board_ReadAddress
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
	.globl _Display_ShowRawDigit_PARM_2
	.globl _Display_Init
	.globl _Display_SetValue
	.globl _Display_ScanOnce
	.globl _Display_ShowRawDigit
	.globl _Display_TestAllOn
	.globl _Display_TestPolarity
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
_g_digits:
	.ds 3
_g_scan_index:
	.ds 1
_Display_ShowRawDigit_PARM_2:
	.ds 1
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
;Allocation info for local variables in function 'Display_AllDigitsOff'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:15: static void Display_AllDigitsOff(void)
;	-----------------------------------------
;	 function Display_AllDigitsOff
;	-----------------------------------------
_Display_AllDigitsOff:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:17: P23 = 1;
;	assignBit
	setb	_P23
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:18: P26 = 1;
;	assignBit
	setb	_P26
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:19: P01 = 1;
;	assignBit
	setb	_P01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:20: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_SetSegments'
;------------------------------------------------------------
;pattern       Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:22: static void Display_SetSegments(uint8_t pattern)
;	-----------------------------------------
;	 function Display_SetSegments
;	-----------------------------------------
_Display_SetSegments:
	mov	r7, dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:24: P24 = (pattern & 0x01U) ? 1 : 0;
	mov	a,#0x01
	anl	a,r7
;	assignBit
	add	a,#0xff
	mov	_P24,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:25: P27 = (pattern & 0x02U) ? 1 : 0;
	mov	a,#0x02
	anl	a,r7
;	assignBit
	add	a,#0xff
	mov	_P27,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:26: P36 = (pattern & 0x04U) ? 1 : 0;
	mov	a,#0x04
	anl	a,r7
;	assignBit
	add	a,#0xff
	mov	_P36,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:27: P20 = (pattern & 0x08U) ? 1 : 0;
	mov	a,#0x08
	anl	a,r7
;	assignBit
	add	a,#0xff
	mov	_P20,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:28: P21 = (pattern & 0x10U) ? 1 : 0;
	mov	a,#0x10
	anl	a,r7
;	assignBit
	add	a,#0xff
	mov	_P21,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:29: P25 = (pattern & 0x20U) ? 1 : 0;
	mov	a,#0x20
	anl	a,r7
;	assignBit
	add	a,#0xff
	mov	_P25,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:30: P35 = (pattern & 0x40U) ? 1 : 0;
	anl	ar7,#0x40
;	assignBit
	mov	a,r7
	add	a,#0xff
	mov	_P35,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:31: P37 = 0;
;	assignBit
	clr	_P37
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:32: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_EnableDigit'
;------------------------------------------------------------
;index         Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:34: static void Display_EnableDigit(uint8_t index)
;	-----------------------------------------
;	 function Display_EnableDigit
;	-----------------------------------------
_Display_EnableDigit:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:36: if (index == 0U) {
	mov	a,dpl
	mov	r7,a
	jnz	00105$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:37: P23 = 0;
;	assignBit
	clr	_P23
	ret
00105$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:38: } else if (index == 1U) {
	cjne	r7,#0x01,00102$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:39: P26 = 0;
;	assignBit
	clr	_P26
	ret
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:41: P01 = 0;
;	assignBit
	clr	_P01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:43: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_WaitAndPollRx'
;------------------------------------------------------------
;count         Allocated to registers 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:45: static void Display_WaitAndPollRx(uint16_t count)
;	-----------------------------------------
;	 function Display_WaitAndPollRx
;	-----------------------------------------
_Display_WaitAndPollRx:
	mov	r6, dpl
	mov	r7, dph
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:47: while (count-- != 0U) {
00101$:
	mov	ar4,r6
	mov	ar5,r7
	dec	r6
	cjne	r6,#0xff,00121$
	dec	r7
00121$:
	mov	a,r4
	orl	a,r5
	jz	00104$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:48: Pressure_ProcessRx();
	push	ar7
	push	ar6
	lcall	_Pressure_ProcessRx
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:49: SensorModbus_Process(Board_ReadAddress(), Pressure_GetValue());
	lcall	_Board_ReadAddress
	mov	r5, dpl
	push	ar5
	lcall	_Pressure_GetValue
	mov	_SensorModbus_Process_PARM_2,dpl
	mov	(_SensorModbus_Process_PARM_2 + 1),dph
	pop	ar5
	mov	dpl, r5
	lcall	_SensorModbus_Process
	pop	ar6
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:50: __asm nop __endasm;
	nop	
	sjmp	00101$
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:52: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_Init'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:54: void Display_Init(void)
;	-----------------------------------------
;	 function Display_Init
;	-----------------------------------------
_Display_Init:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:56: g_digits[0] = 0U;
	mov	_g_digits,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:57: g_digits[1] = 0U;
	mov	(_g_digits + 0x0001),#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:58: g_digits[2] = 0U;
	mov	(_g_digits + 0x0002),#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:59: g_scan_index = 0U;
	mov	_g_scan_index,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:60: Display_AllDigitsOff();
	lcall	_Display_AllDigitsOff
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:61: Display_SetSegments(0U);
	mov	dpl, #0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:62: }
	ljmp	_Display_SetSegments
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_SetValue'
;------------------------------------------------------------
;value         Allocated to registers r6 r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:64: void Display_SetValue(uint16_t value)
;	-----------------------------------------
;	 function Display_SetValue
;	-----------------------------------------
_Display_SetValue:
	mov	r6, dpl
	mov	r7, dph
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:66: if (value > 999U) {
	mov	ar4,r6
	mov	ar5,r7
	clr	c
	mov	a,#0xe7
	subb	a,r4
	mov	a,#0x03
	subb	a,r5
	jnc	00102$
;	free result
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:67: value = 999U;
	mov	r6,#0xe7
	mov	r7,#0x03
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:70: g_digits[0] = (uint8_t)(value / 100U);
	mov	__divuint_PARM_2,#0x64
	mov	(__divuint_PARM_2 + 1),#0x00
	mov	dpl, r6
	mov	dph, r7
	push	ar7
	push	ar6
	lcall	__divuint
	mov	r4, dpl
	pop	ar6
	pop	ar7
	mov	_g_digits,r4
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:71: g_digits[1] = (uint8_t)((value / 10U) % 10U);
	mov	__divuint_PARM_2,#0x0a
	mov	(__divuint_PARM_2 + 1),#0x00
	mov	dpl, r6
	mov	dph, r7
	push	ar7
	push	ar6
	lcall	__divuint
	mov	__moduint_PARM_2,#0x0a
	mov	(__moduint_PARM_2 + 1),#0x00
	lcall	__moduint
	mov	r4, dpl
	pop	ar6
	pop	ar7
	mov	(_g_digits + 0x0001),r4
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:72: g_digits[2] = (uint8_t)(value % 10U);
	mov	__moduint_PARM_2,#0x0a
	mov	(__moduint_PARM_2 + 1),#0x00
	mov	dpl, r6
	mov	dph, r7
	lcall	__moduint
	mov	r6, dpl
	mov	(_g_digits + 0x0002),r6
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:73: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_ScanOnce'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:75: void Display_ScanOnce(void)
;	-----------------------------------------
;	 function Display_ScanOnce
;	-----------------------------------------
_Display_ScanOnce:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:77: Display_AllDigitsOff();
	lcall	_Display_AllDigitsOff
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:78: Display_SetSegments(0U);
	mov	dpl, #0x00
	lcall	_Display_SetSegments
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:79: Display_WaitAndPollRx(10U);
	mov	dptr,#0x000a
	lcall	_Display_WaitAndPollRx
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:81: Display_SetSegments(g_seg_table[g_digits[g_scan_index]]);
	mov	a,_g_scan_index
	add	a, #_g_digits
	mov	r1,a
	mov	a,@r1
	mov	dptr,#_g_seg_table
	movc	a,@a+dptr
	mov	dpl,a
	lcall	_Display_SetSegments
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:82: Display_WaitAndPollRx(10U);
	mov	dptr,#0x000a
	lcall	_Display_WaitAndPollRx
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:84: Display_EnableDigit(g_scan_index);
	mov	dpl, _g_scan_index
	lcall	_Display_EnableDigit
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:85: Display_WaitAndPollRx(600U);
	mov	dptr,#0x0258
	lcall	_Display_WaitAndPollRx
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:86: Display_AllDigitsOff();
	lcall	_Display_AllDigitsOff
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:88: g_scan_index++;
	inc	_g_scan_index
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:89: if (g_scan_index >= 3U) {
	mov	a,#0x100 - 0x03
	add	a,_g_scan_index
	jnc	00103$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:90: g_scan_index = 0U;
	mov	_g_scan_index,#0x00
00103$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:92: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_ShowRawDigit'
;------------------------------------------------------------
;value         Allocated with name '_Display_ShowRawDigit_PARM_2'
;digit         Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:94: void Display_ShowRawDigit(uint8_t digit, uint8_t value)
;	-----------------------------------------
;	 function Display_ShowRawDigit
;	-----------------------------------------
_Display_ShowRawDigit:
	mov	r7, dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:96: if (value > 9U) {
	mov	a,_Display_ShowRawDigit_PARM_2
	add	a,#0xff - 0x09
	jnc	00102$
;	free result
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:97: value = 9U;
	mov	_Display_ShowRawDigit_PARM_2,#0x09
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:100: Display_AllDigitsOff();
	push	ar7
	lcall	_Display_AllDigitsOff
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:101: Display_SetSegments(0U);
	mov	dpl, #0x00
	lcall	_Display_SetSegments
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:102: Display_SetSegments(g_seg_table[value]);
	mov	a,_Display_ShowRawDigit_PARM_2
	mov	dptr,#_g_seg_table
	movc	a,@a+dptr
	mov	dpl,a
	push	ar7
	lcall	_Display_SetSegments
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:103: Display_EnableDigit(digit);
	mov	dpl, r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:104: }
	ljmp	_Display_EnableDigit
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_TestAllOn'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:106: void Display_TestAllOn(void)
;	-----------------------------------------
;	 function Display_TestAllOn
;	-----------------------------------------
_Display_TestAllOn:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:108: P24 = 1;
;	assignBit
	setb	_P24
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:109: P27 = 1;
;	assignBit
	setb	_P27
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:110: P36 = 1;
;	assignBit
	setb	_P36
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:111: P20 = 1;
;	assignBit
	setb	_P20
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:112: P21 = 1;
;	assignBit
	setb	_P21
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:113: P25 = 1;
;	assignBit
	setb	_P25
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:114: P35 = 1;
;	assignBit
	setb	_P35
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:115: P37 = 1;
;	assignBit
	setb	_P37
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:117: P23 = 0;
;	assignBit
	clr	_P23
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:118: P26 = 0;
;	assignBit
	clr	_P26
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:119: P01 = 0;
;	assignBit
	clr	_P01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:120: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Display_TestPolarity'
;------------------------------------------------------------
;mode          Allocated to registers r7 
;seg_on        Allocated to registers r6 
;dig_on        Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:122: void Display_TestPolarity(uint8_t mode)
;	-----------------------------------------
;	 function Display_TestPolarity
;	-----------------------------------------
_Display_TestPolarity:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:124: uint8_t seg_on = (mode & 0x01U) ? 1U : 0U;
	mov	a,dpl
	mov	r7,a
	jnb	acc.0,00103$
	mov	r6,#0x01
	sjmp	00104$
00103$:
	mov	r6,#0x00
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:125: uint8_t dig_on = (mode & 0x02U) ? 1U : 0U;
	mov	a,r7
	jnb	acc.1,00105$
	mov	r7,#0x01
	sjmp	00106$
00105$:
	mov	r7,#0x00
00106$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:127: P24 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	clr	a
	rlc	a
;	assignBit
	mov	r6,a
	add	a,#0xff
	mov	_P24,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:128: P27 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	mov	_P27,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:129: P36 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	mov	_P36,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:130: P20 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	mov	_P20,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:131: P21 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	mov	_P21,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:132: P25 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	mov	_P25,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:133: P35 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	mov	_P35,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:134: P37 = seg_on;
;	assignBit
	mov	a,r6
	add	a,#0xff
	mov	_P37,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:136: P23 = dig_on;
;	assignBit
	mov	a,r7
	add	a,#0xff
	clr	a
	rlc	a
;	assignBit
	mov	r7,a
	add	a,#0xff
	mov	_P23,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:137: P26 = dig_on;
;	assignBit
	mov	a,r7
	add	a,#0xff
	mov	_P26,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:138: P01 = dig_on;
;	assignBit
	mov	a,r7
	add	a,#0xff
	mov	_P01,c
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:139: }
	ret
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area CONST   (CODE)
_g_seg_table:
	.db #0x3f	; 63
	.db #0x06	; 6
	.db #0x5b	; 91
	.db #0x4f	; 79	'O'
	.db #0x66	; 102	'f'
	.db #0x6d	; 109	'm'
	.db #0x7d	; 125
	.db #0x07	; 7
	.db #0x7f	; 127
	.db #0x6f	; 111	'o'
	.area CSEG    (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
