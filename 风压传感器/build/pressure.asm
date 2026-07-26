;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.6.0 #16555 (Linux)
;--------------------------------------------------------
	.module pressure
	
	.optsdcc -mmcs51 --model-small
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _Uart1_Send
	.globl _Uart2_ReadByte
	.globl _Uart1_ReadByte
	.globl _Crc16_Append
	.globl _Crc16_Check
	.globl _Pressure_OnByte
	.globl _Pressure_Init
	.globl _Pressure_GetValue
	.globl _Pressure_ProcessRx
	.globl _Pressure_ProcessControllerRx
	.globl _Pressure_PollBlocking
;--------------------------------------------------------
; special function registers
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
;--------------------------------------------------------
; special function bits
;--------------------------------------------------------
	.area RSEG    (ABS,DATA)
	.org 0x0000
;--------------------------------------------------------
; overlayable register banks
;--------------------------------------------------------
	.area REG_BANK_0	(REL,OVR,DATA)
	.ds 8
;--------------------------------------------------------
; internal ram data
;--------------------------------------------------------
	.area DSEG    (DATA)
_g_pressure:
	.ds 2
_g_frame_pos:
	.ds 1
_g_pressure_hi:
	.ds 1
_g_pressure_lo:
	.ds 1
_Pressure_ProcessRx_b_10000_37:
	.ds 1
_Pressure_ProcessControllerRx_b_10000_40:
	.ds 1
_Pressure_PollBlocking_req_10000_43:
	.ds 8
_Pressure_PollBlocking_resp_10000_43:
	.ds 7
_Pressure_PollBlocking_b_10000_43:
	.ds 1
;--------------------------------------------------------
; overlayable items in internal ram
;--------------------------------------------------------
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
;Allocation info for local variables in function 'Pressure_OnByte'
;------------------------------------------------------------
;b             Allocated to registers r7 
;pressure      Allocated to registers r6 r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:12: void Pressure_OnByte(uint8_t b)
;	-----------------------------------------
;	 function Pressure_OnByte
;	-----------------------------------------
_Pressure_OnByte:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
	mov	r7, dpl
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:14: if (b == 0xFEU) {
	cjne	r7,#0xfe,00102$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:15: g_frame_pos = 1U;
	mov	_g_frame_pos,#0x01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:16: return;
	ret
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:19: switch (g_frame_pos) {
	mov	a,_g_frame_pos
	add	a,#0xff - 0x04
	jc	00108$
;	free result
	mov	a,_g_frame_pos
	mov	b,#0x03
	mul	ab
	mov	dptr,#00148$
	jmp	@a+dptr
00148$:
	ljmp	00114$
	ljmp	00104$
	ljmp	00105$
	ljmp	00106$
	ljmp	00107$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:23: case 1:
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:24: g_pressure_hi = b;
	mov	_g_pressure_hi,r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:25: g_frame_pos = 2U;
	mov	_g_frame_pos,#0x02
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:26: break;
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:28: case 2:
	ret
00105$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:29: g_pressure_lo = b;
	mov	_g_pressure_lo,r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:30: g_frame_pos = 3U;
	mov	_g_frame_pos,#0x03
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:31: break;
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:33: case 3:
	ret
00106$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:34: g_frame_pos = 4U;
	mov	_g_frame_pos,#0x04
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:35: break;
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:37: case 4:
	ret
00107$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:38: g_frame_pos = 5U;
	mov	_g_frame_pos,#0x05
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:39: break;
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:41: default:
	ret
00108$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:42: if (b == 0xDCU) {
	cjne	r7,#0xdc,00112$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:43: uint16_t pressure = ((uint16_t)g_pressure_hi << 8) | g_pressure_lo;
	mov	r7,_g_pressure_hi
	mov	r6,#0x00
	mov	r4,_g_pressure_lo
	mov	r5,#0x00
	mov	a,r4
	orl	ar6,a
	mov	a,r5
	orl	ar7,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:44: if (pressure <= 999U) {
	mov	ar4,r6
	mov	ar5,r7
	clr	c
	mov	a,#0xe7
	subb	a,r4
	mov	a,#0x03
	subb	a,r5
	jc	00112$
;	free result
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:45: g_pressure = pressure;
	mov	_g_pressure,r6
	mov	(_g_pressure + 1),r7
00112$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:48: g_frame_pos = 0U;
	mov	_g_frame_pos,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:50: }
00114$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:51: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Pressure_Init'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:53: void Pressure_Init(void)
;	-----------------------------------------
;	 function Pressure_Init
;	-----------------------------------------
_Pressure_Init:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:55: g_pressure = 0U;
	clr	a
	mov	_g_pressure,a
	mov	(_g_pressure + 1),a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:56: g_frame_pos = 0U;
	mov	_g_frame_pos,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:57: g_pressure_hi = 0U;
	mov	_g_pressure_hi,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:58: g_pressure_lo = 0U;
	mov	_g_pressure_lo,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:59: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Pressure_GetValue'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:61: uint16_t Pressure_GetValue(void)
;	-----------------------------------------
;	 function Pressure_GetValue
;	-----------------------------------------
_Pressure_GetValue:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:63: return g_pressure;
	mov	dpl, _g_pressure
	mov	dph, (_g_pressure + 1)
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:64: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Pressure_ProcessRx'
;------------------------------------------------------------
;b             Allocated with name '_Pressure_ProcessRx_b_10000_37'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:66: void Pressure_ProcessRx(void)
;	-----------------------------------------
;	 function Pressure_ProcessRx
;	-----------------------------------------
_Pressure_ProcessRx:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:70: while (Uart1_ReadByte(&b) != 0U) {
00101$:
	mov	dptr,#_Pressure_ProcessRx_b_10000_37
	mov	b, #0x40
	lcall	_Uart1_ReadByte
	mov	a, dpl
	jz	00104$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:71: Pressure_OnByte(b);
	mov	dpl, _Pressure_ProcessRx_b_10000_37
	lcall	_Pressure_OnByte
	sjmp	00101$
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:73: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Pressure_ProcessControllerRx'
;------------------------------------------------------------
;b             Allocated with name '_Pressure_ProcessControllerRx_b_10000_40'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:75: void Pressure_ProcessControllerRx(void)
;	-----------------------------------------
;	 function Pressure_ProcessControllerRx
;	-----------------------------------------
_Pressure_ProcessControllerRx:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:79: while (Uart2_ReadByte(&b) != 0U) {
00101$:
	mov	dptr,#_Pressure_ProcessControllerRx_b_10000_40
	mov	b, #0x40
	lcall	_Uart2_ReadByte
	mov	a, dpl
	jz	00104$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:80: Pressure_OnByte(b);
	mov	dpl, _Pressure_ProcessControllerRx_b_10000_40
	lcall	_Pressure_OnByte
	sjmp	00101$
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:82: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Pressure_PollBlocking'
;------------------------------------------------------------
;req           Allocated with name '_Pressure_PollBlocking_req_10000_43'
;resp          Allocated with name '_Pressure_PollBlocking_resp_10000_43'
;i             Allocated to registers r7 
;wait          Allocated to registers r5 r6 
;b             Allocated with name '_Pressure_PollBlocking_b_10000_43'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:84: void Pressure_PollBlocking(void)
;	-----------------------------------------
;	 function Pressure_PollBlocking
;	-----------------------------------------
_Pressure_PollBlocking:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:92: req[0] = PRESSURE_MODULE_ADDR;
	mov	_Pressure_PollBlocking_req_10000_43,#0x01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:93: req[1] = 0x03U;
	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0001),#0x03
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:94: req[2] = 0x00U;
	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0002),#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:95: req[3] = 0x00U;
	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0003),#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:96: req[4] = 0x00U;
	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0004),#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:97: req[5] = 0x01U;
	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0005),#0x01
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:98: Crc16_Append(req, 6U);
	mov	_Crc16_Append_PARM_2,#0x06
	mov	dptr,#_Pressure_PollBlocking_req_10000_43
	mov	b, #0x40
	lcall	_Crc16_Append
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:100: Uart1_Send(req, sizeof(req));
	mov	_Uart1_Send_PARM_2,#0x08
	mov	dptr,#_Pressure_PollBlocking_req_10000_43
	mov	b, #0x40
	lcall	_Uart1_Send
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:102: for (wait = 0U; wait < 30000U; ++wait) {
	mov	r7,#0x00
	mov	r5,#0x00
	mov	r6,#0x00
00114$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:103: if (Uart1_ReadByte(&b) != 0U) {
	mov	dptr,#_Pressure_PollBlocking_b_10000_43
	mov	b, #0x40
	push	ar7
	push	ar6
	push	ar5
	lcall	_Uart1_ReadByte
	mov	a, dpl
	pop	ar5
	pop	ar6
	pop	ar7
	jz	00115$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:105: resp[i++] = b;
	mov	a,r7
	add	a, #_Pressure_PollBlocking_resp_10000_43
	mov	r1,a
	inc	r7
	mov	@r1,_Pressure_PollBlocking_b_10000_43
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:107: if (i >= sizeof(resp)) {
	cjne	r7,#0x07,00172$
00172$:
	jnc	00107$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:108: break;
00115$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:102: for (wait = 0U; wait < 30000U; ++wait) {
	inc	r5
	cjne	r5,#0x00,00174$
	inc	r6
00174$:
	mov	ar3,r5
	mov	ar4,r6
	clr	c
	mov	a,r3
	subb	a,#0x30
	mov	a,r4
	subb	a,#0x75
	jc	00114$
00107$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:113: if ((i == sizeof(resp)) &&
	cjne	r7,#0x07,00116$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:114: (resp[0] == PRESSURE_MODULE_ADDR) &&
	mov	a,#0x01
	cjne	a,_Pressure_PollBlocking_resp_10000_43,00116$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:115: (resp[1] == 0x03U) &&
	mov	a,#0x03
	cjne	a,(_Pressure_PollBlocking_resp_10000_43 + 0x0001),00116$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:116: (resp[2] == 0x02U) &&
	mov	a,#0x02
	cjne	a,(_Pressure_PollBlocking_resp_10000_43 + 0x0002),00116$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:117: (Crc16_Check(resp, sizeof(resp)) != 0U)) {
	mov	_Crc16_Check_PARM_2,#0x07
	mov	dptr,#_Pressure_PollBlocking_resp_10000_43
	mov	b, #0x40
	lcall	_Crc16_Check
	mov	a, dpl
	jz	00116$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:118: g_pressure = ((uint16_t)resp[3] << 8) | resp[4];
	mov	r7,(_Pressure_PollBlocking_resp_10000_43 + 0x0003)
	mov	r6,#0x00
	mov	r4,(_Pressure_PollBlocking_resp_10000_43 + 0x0004)
	mov	r5,#0x00
	mov	a,r4
	orl	a,r6
	mov	_g_pressure,a
	mov	a,r5
	orl	a,r7
	mov	(_g_pressure + 1),a
00116$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:120: }
	ret
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
