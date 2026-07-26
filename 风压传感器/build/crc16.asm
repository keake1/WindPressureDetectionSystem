;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.6.0 #16555 (Linux)
;--------------------------------------------------------
	.module crc16
	
	.optsdcc -mmcs51 --model-small
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _Crc16_Modbus_PARM_2
	.globl _Crc16_Append_PARM_2
	.globl _Crc16_Check_PARM_2
	.globl _Crc16_Modbus
	.globl _Crc16_Check
	.globl _Crc16_Append
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
_Crc16_Check_PARM_2:
	.ds 1
_Crc16_Check_crc_10000_14:
	.ds 2
_Crc16_Append_PARM_2:
	.ds 1
_Crc16_Append_crc_10000_18:
	.ds 2
;--------------------------------------------------------
; overlayable items in internal ram
;--------------------------------------------------------
	.area	OSEG    (OVR,DATA)
_Crc16_Modbus_PARM_2:
	.ds 1
_Crc16_Modbus_data_10000_4:
	.ds 3
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
;Allocation info for local variables in function 'Crc16_Modbus'
;------------------------------------------------------------
;len           Allocated with name '_Crc16_Modbus_PARM_2'
;data          Allocated with name '_Crc16_Modbus_data_10000_4'
;crc           Allocated to registers r3 r4 
;i             Allocated to registers r2 
;bit           Allocated to registers r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:3: uint16_t Crc16_Modbus(const uint8_t *data, uint8_t len)
;	-----------------------------------------
;	 function Crc16_Modbus
;	-----------------------------------------
_Crc16_Modbus:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
	mov	_Crc16_Modbus_data_10000_4,dpl
	mov	(_Crc16_Modbus_data_10000_4 + 1),dph
	mov	(_Crc16_Modbus_data_10000_4 + 2),b
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:5: uint16_t crc = 0xFFFFU;
	mov	r3,#0xff
	mov	r4,#0xff
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:9: for (i = 0U; i < len; ++i) {
	mov	r2,#0x00
00109$:
	clr	c
	mov	a,r2
	subb	a,_Crc16_Modbus_PARM_2
	jnc	00105$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:10: crc ^= data[i];
	mov	a,r2
	add	a, _Crc16_Modbus_data_10000_4
	mov	r0,a
	clr	a
	addc	a, (_Crc16_Modbus_data_10000_4 + 1)
	mov	r1,a
	mov	r7,(_Crc16_Modbus_data_10000_4 + 2)
	mov	dpl,r0
	mov	dph,r1
	mov	b,r7
	lcall	__gptrget
	mov	r7,#0x00
	xrl	ar3,a
	mov	a,r7
	xrl	ar4,a
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:11: for (bit = 0U; bit < 8U; ++bit) {
	mov	r7,#0x00
00106$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:12: if ((crc & 0x0001U) != 0U) {
	mov	a,r3
	jnb	acc.0,00102$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:13: crc = (uint16_t)((crc >> 1) ^ 0xA001U);
	mov	ar5,r3
	mov	a,r4
	clr	c
	rrc	a
	xch	a,r5
	rrc	a
	xch	a,r5
	mov	r6,a
	xrl	ar5,#0x01
	xrl	ar6,#0xa0
	mov	ar3,r5
	mov	ar4,r6
	sjmp	00107$
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:15: crc >>= 1;
	mov	a,r4
	clr	c
	rrc	a
	xch	a,r3
	rrc	a
	xch	a,r3
	mov	r4,a
00107$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:11: for (bit = 0U; bit < 8U; ++bit) {
	inc	r7
	cjne	r7,#0x08,00152$
00152$:
	jc	00106$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:9: for (i = 0U; i < len; ++i) {
	inc	r2
	sjmp	00109$
00105$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:20: return crc;
	mov	dpl, r3
	mov	dph, r4
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:21: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Crc16_Check'
;------------------------------------------------------------
;len           Allocated with name '_Crc16_Check_PARM_2'
;frame         Allocated to registers r5 r6 r7 
;crc           Allocated with name '_Crc16_Check_crc_10000_14'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:23: uint8_t Crc16_Check(const uint8_t *frame, uint8_t len)
;	-----------------------------------------
;	 function Crc16_Check
;	-----------------------------------------
_Crc16_Check:
	mov	r5, dpl
	mov	r6, dph
	mov	r7, b
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:27: if ((frame == 0) || (len < 3U)) {
	mov	a,r5
	orl	a,r6
	jz	00101$
	mov	a,#0x100 - 0x03
	add	a,_Crc16_Check_PARM_2
	jc	00102$
00101$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:28: return 0U;
	mov	dpl, #0x00
	ret
00102$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:31: crc = Crc16_Modbus(frame, (uint8_t)(len - 2U));
	mov	a,_Crc16_Check_PARM_2
	add	a,#0xfe
	mov	_Crc16_Modbus_PARM_2,a
	mov	dpl, r5
	mov	dph, r6
	mov	b, r7
	push	ar7
	push	ar6
	push	ar5
	lcall	_Crc16_Modbus
	mov	_Crc16_Check_crc_10000_14,dpl
	mov	(_Crc16_Check_crc_10000_14 + 1),dph
	pop	ar5
	pop	ar6
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:32: return ((frame[len - 2U] == (uint8_t)crc) &&
	mov	r1,_Crc16_Check_PARM_2
	mov	r2,#0x00
	mov	a,r1
	add	a,#0xfe
	mov	r0,a
	mov	a,r2
	addc	a,#0xff
	mov	r4,a
	mov	a,r0
	add	a, r5
	mov	r0,a
	mov	a,r4
	addc	a, r6
	mov	r4,a
	mov	ar3,r7
	mov	dpl,r0
	mov	dph,r4
	mov	b,r3
	lcall	__gptrget
	mov	r3,_Crc16_Check_crc_10000_14
	cjne	a,ar3,00106$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:33: (frame[len - 1U] == (uint8_t)(crc >> 8))) ? 1U : 0U;
	dec	r1
	cjne	r1,#0xff,00135$
	dec	r2
00135$:
	mov	a,r1
	add	a, r5
	mov	r5,a
	mov	a,r2
	addc	a, r6
	mov	r6,a
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	lcall	__gptrget
	mov	r7,(_Crc16_Check_crc_10000_14 + 1)
	cjne	a,ar7,00106$
	mov	r7,#0x01
	sjmp	00107$
00106$:
	mov	r7,#0x00
00107$:
	mov	dpl,r7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:34: }
	ret
;------------------------------------------------------------
;Allocation info for local variables in function 'Crc16_Append'
;------------------------------------------------------------
;payload_len   Allocated with name '_Crc16_Append_PARM_2'
;frame         Allocated to registers r5 r6 r7 
;crc           Allocated with name '_Crc16_Append_crc_10000_18'
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:36: void Crc16_Append(uint8_t *frame, uint8_t payload_len)
;	-----------------------------------------
;	 function Crc16_Append
;	-----------------------------------------
_Crc16_Append:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:38: uint16_t crc = Crc16_Modbus(frame, payload_len);
	mov	r5,dpl
	mov	r6,dph
	mov	r7,b
	mov	_Crc16_Modbus_PARM_2,_Crc16_Append_PARM_2
	push	ar7
	push	ar6
	push	ar5
	lcall	_Crc16_Modbus
	mov	_Crc16_Append_crc_10000_18,dpl
	mov	(_Crc16_Append_crc_10000_18 + 1),dph
	pop	ar5
	pop	ar6
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:39: frame[payload_len] = (uint8_t)crc;
	mov	a,_Crc16_Append_PARM_2
	add	a, r5
	mov	r0,a
	clr	a
	addc	a, r6
	mov	r1,a
	mov	ar2,r7
	mov	r4,_Crc16_Append_crc_10000_18
	mov	dpl,r0
	mov	dph,r1
	mov	b,r2
	mov	a,r4
	lcall	__gptrput
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:40: frame[payload_len + 1U] = (uint8_t)(crc >> 8);
	mov	r4,_Crc16_Append_PARM_2
	mov	r3,#0x00
	inc	r4
	cjne	r4,#0x00,00103$
	inc	r3
00103$:
	mov	a,r4
	add	a, r5
	mov	r5,a
	mov	a,r3
	addc	a, r6
	mov	r6,a
	mov	r4,(_Crc16_Append_crc_10000_18 + 1)
	mov	dpl,r5
	mov	dph,r6
	mov	b,r7
	mov	a,r4
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:41: }
	ljmp	__gptrput
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
