;--------------------------------------------------------
; File Created by SDCC : free open source ISO C Compiler
; Version 4.6.0 #16555 (Linux)
;--------------------------------------------------------
	.module main
	
	.optsdcc -mmcs51 --model-small
;--------------------------------------------------------
; Public variables in this module
;--------------------------------------------------------
	.globl _main
	.globl _Uart_Init
	.globl _SensorModbus_Process
	.globl _SensorModbus_Init
	.globl _Pressure_GetValue
	.globl _Pressure_ProcessRx
	.globl _Pressure_Init
	.globl _Display_ScanOnce
	.globl _Display_SetValue
	.globl _Display_Init
	.globl _Board_DelayMs
	.globl _Board_GreenLedToggle
	.globl _Board_ReadAddress
	.globl _Board_Init
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
;--------------------------------------------------------
; overlayable items in internal ram
;--------------------------------------------------------
;--------------------------------------------------------
; Stack segment in internal ram
;--------------------------------------------------------
	.area SSEG
__start__stack:
	.ds	1

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
; interrupt vector
;--------------------------------------------------------
	.area HOME    (CODE)
__interrupt_vect:
	ljmp	__sdcc_gsinit_startup
; restartable atomic support routines
	.ds	5
sdcc_atomic_exchange_rollback_start::
	nop
	nop
sdcc_atomic_exchange_pdata_impl:
	movx	a, @r0
	mov	r3, a
	mov	a, r2
	movx	@r0, a
	sjmp	sdcc_atomic_exchange_exit
	nop
	nop
sdcc_atomic_exchange_xdata_impl:
	movx	a, @dptr
	mov	r3, a
	mov	a, r2
	movx	@dptr, a
	sjmp	sdcc_atomic_exchange_exit
sdcc_atomic_compare_exchange_idata_impl:
	mov	a, @r0
	cjne	a, ar2, .+#5
	mov	a, r3
	mov	@r0, a
	ret
	nop
sdcc_atomic_compare_exchange_pdata_impl:
	movx	a, @r0
	cjne	a, ar2, .+#5
	mov	a, r3
	movx	@r0, a
	ret
	nop
sdcc_atomic_compare_exchange_xdata_impl:
	movx	a, @dptr
	cjne	a, ar2, .+#5
	mov	a, r3
	movx	@dptr, a
	ret
sdcc_atomic_exchange_rollback_end::

sdcc_atomic_exchange_gptr_impl::
	jnb	b.6, sdcc_atomic_exchange_xdata_impl
	mov	r0, dpl
	jb	b.5, sdcc_atomic_exchange_pdata_impl
sdcc_atomic_exchange_idata_impl:
	mov	a, r2
	xch	a, @r0
	mov	dpl, a
	ret
sdcc_atomic_exchange_exit:
	mov	dpl, r3
	ret
sdcc_atomic_compare_exchange_gptr_impl::
	jnb	b.6, sdcc_atomic_compare_exchange_xdata_impl
	mov	r0, dpl
	jb	b.5, sdcc_atomic_compare_exchange_pdata_impl
	sjmp	sdcc_atomic_compare_exchange_idata_impl
;--------------------------------------------------------
; global & static initialisations
;--------------------------------------------------------
	.area HOME    (CODE)
	.area GSINIT  (CODE)
	.area GSFINAL (CODE)
	.area GSINIT  (CODE)
	.globl __sdcc_gsinit_startup
	.globl __sdcc_program_startup
	.globl __start__stack
	.globl __mcs51_genXINIT
	.globl __mcs51_genXRAMCLEAR
	.globl __mcs51_genRAMCLEAR
	.area GSFINAL (CODE)
	ljmp	__sdcc_program_startup
;--------------------------------------------------------
; Home
;--------------------------------------------------------
	.area HOME    (CODE)
	.area HOME    (CODE)
__sdcc_program_startup:
	lcall	_main
__sdcc_program_exit:
	sjmp	.
;	return from main will return to caller
;--------------------------------------------------------
; code
;--------------------------------------------------------
	.area CSEG    (CODE)
;------------------------------------------------------------
;Allocation info for local variables in function 'main'
;------------------------------------------------------------
;heartbeat_tick Allocated to registers r6 r7 
;------------------------------------------------------------
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:8: void main(void)
;	-----------------------------------------
;	 function main
;	-----------------------------------------
_main:
	ar7 = 0x07
	ar6 = 0x06
	ar5 = 0x05
	ar4 = 0x04
	ar3 = 0x03
	ar2 = 0x02
	ar1 = 0x01
	ar0 = 0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:10: uint16_t heartbeat_tick = 0U;
	mov	r6,#0x00
	mov	r7,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:12: Board_Init();
	push	ar7
	push	ar6
	lcall	_Board_Init
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:13: Uart_Init();
	lcall	_Uart_Init
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:14: Pressure_Init();
	lcall	_Pressure_Init
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:15: SensorModbus_Init();
	lcall	_SensorModbus_Init
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:16: Display_Init();
	lcall	_Display_Init
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:17: Board_GreenLedToggle();
	lcall	_Board_GreenLedToggle
	pop	ar6
	pop	ar7
00104$:
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:24: Pressure_ProcessRx();
	push	ar7
	push	ar6
	lcall	_Pressure_ProcessRx
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:25: SensorModbus_Process(Board_ReadAddress(), Pressure_GetValue());
	lcall	_Board_ReadAddress
	mov	r5, dpl
	push	ar5
	lcall	_Pressure_GetValue
	mov	_SensorModbus_Process_PARM_2,dpl
	mov	(_SensorModbus_Process_PARM_2 + 1),dph
	pop	ar5
	mov	dpl, r5
	lcall	_SensorModbus_Process
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:26: Display_SetValue(Pressure_GetValue());
	lcall	_Pressure_GetValue
	lcall	_Display_SetValue
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:27: Display_ScanOnce();
	lcall	_Display_ScanOnce
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:30: Board_DelayMs(1U);
	mov	dptr,#0x0001
	lcall	_Board_DelayMs
	pop	ar6
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:32: heartbeat_tick+=1U;
	mov	ar4,r6
	mov	ar5,r7
	inc	r4
	cjne	r4,#0x00,00122$
	inc	r5
00122$:
	mov	ar6,r4
	mov	ar7,r5
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:33: if (heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
	mov	ar4,r6
	mov	ar5,r7
	clr	c
	mov	a,r4
	subb	a,#0xf4
	mov	a,r5
	subb	a,#0x01
	jc	00104$
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:34: heartbeat_tick = 0U;
	mov	r6,#0x00
	mov	r7,#0x00
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:35: Board_GreenLedToggle();
	push	ar7
	push	ar6
	lcall	_Board_GreenLedToggle
	pop	ar6
	pop	ar7
;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:38: }
	ljmp	00104$
	.area CSEG    (CODE)
	.area CONST   (CODE)
	.area XINIT   (CODE)
	.area CABS    (ABS,CODE)
