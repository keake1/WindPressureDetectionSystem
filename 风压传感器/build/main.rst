                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler
                                      3 ; Version 4.6.0 #16555 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module main
                                      6 	
                                      7 	.optsdcc -mmcs51 --model-small
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _main
                                     12 	.globl _Uart_Init
                                     13 	.globl _SensorModbus_Process
                                     14 	.globl _SensorModbus_Init
                                     15 	.globl _Pressure_GetValue
                                     16 	.globl _Pressure_ProcessRx
                                     17 	.globl _Pressure_Init
                                     18 	.globl _Display_ScanOnce
                                     19 	.globl _Display_SetValue
                                     20 	.globl _Display_Init
                                     21 	.globl _Board_DelayMs
                                     22 	.globl _Board_GreenLedToggle
                                     23 	.globl _Board_ReadAddress
                                     24 	.globl _Board_Init
                                     25 ;--------------------------------------------------------
                                     26 ; special function registers
                                     27 ;--------------------------------------------------------
                                     28 	.area RSEG    (ABS,DATA)
      000000                         29 	.org 0x0000
                                     30 ;--------------------------------------------------------
                                     31 ; special function bits
                                     32 ;--------------------------------------------------------
                                     33 	.area RSEG    (ABS,DATA)
      000000                         34 	.org 0x0000
                                     35 ;--------------------------------------------------------
                                     36 ; overlayable register banks
                                     37 ;--------------------------------------------------------
                                     38 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                         39 	.ds 8
                                     40 ;--------------------------------------------------------
                                     41 ; internal ram data
                                     42 ;--------------------------------------------------------
                                     43 	.area DSEG    (DATA)
                                     44 ;--------------------------------------------------------
                                     45 ; overlayable items in internal ram
                                     46 ;--------------------------------------------------------
                                     47 ;--------------------------------------------------------
                                     48 ; Stack segment in internal ram
                                     49 ;--------------------------------------------------------
                                     50 	.area SSEG
      000061                         51 __start__stack:
      000061                         52 	.ds	1
                                     53 
                                     54 ;--------------------------------------------------------
                                     55 ; indirectly addressable internal ram data
                                     56 ;--------------------------------------------------------
                                     57 	.area ISEG    (DATA)
                                     58 ;--------------------------------------------------------
                                     59 ; absolute internal ram data
                                     60 ;--------------------------------------------------------
                                     61 	.area IABS    (ABS,DATA)
                                     62 	.area IABS    (ABS,DATA)
                                     63 ;--------------------------------------------------------
                                     64 ; bit data
                                     65 ;--------------------------------------------------------
                                     66 	.area BSEG    (BIT)
                                     67 ;--------------------------------------------------------
                                     68 ; paged external ram data
                                     69 ;--------------------------------------------------------
                                     70 	.area PSEG    (PAG,XDATA)
                                     71 ;--------------------------------------------------------
                                     72 ; uninitialized external ram data
                                     73 ;--------------------------------------------------------
                                     74 	.area XSEG    (XDATA)
                                     75 ;--------------------------------------------------------
                                     76 ; absolute external ram data
                                     77 ;--------------------------------------------------------
                                     78 	.area XABS    (ABS,XDATA)
                                     79 ;--------------------------------------------------------
                                     80 ; initialized external ram data
                                     81 ;--------------------------------------------------------
                                     82 	.area XISEG   (XDATA)
                                     83 	.area HOME    (CODE)
                                     84 	.area GSINIT0 (CODE)
                                     85 	.area GSINIT1 (CODE)
                                     86 	.area GSINIT2 (CODE)
                                     87 	.area GSINIT3 (CODE)
                                     88 	.area GSINIT4 (CODE)
                                     89 	.area GSINIT5 (CODE)
                                     90 	.area GSINIT  (CODE)
                                     91 	.area GSFINAL (CODE)
                                     92 	.area CSEG    (CODE)
                                     93 ;--------------------------------------------------------
                                     94 ; interrupt vector
                                     95 ;--------------------------------------------------------
                                     96 	.area HOME    (CODE)
      000000                         97 __interrupt_vect:
      000000 02 00 4E         [24]   98 	ljmp	__sdcc_gsinit_startup
                                     99 ; restartable atomic support routines
      000003                        100 	.ds	5
      000008                        101 sdcc_atomic_exchange_rollback_start::
      000008 00               [12]  102 	nop
      000009 00               [12]  103 	nop
      00000A                        104 sdcc_atomic_exchange_pdata_impl:
      00000A E2               [24]  105 	movx	a, @r0
      00000B FB               [12]  106 	mov	r3, a
      00000C EA               [12]  107 	mov	a, r2
      00000D F2               [24]  108 	movx	@r0, a
      00000E 80 2C            [24]  109 	sjmp	sdcc_atomic_exchange_exit
      000010 00               [12]  110 	nop
      000011 00               [12]  111 	nop
      000012                        112 sdcc_atomic_exchange_xdata_impl:
      000012 E0               [24]  113 	movx	a, @dptr
      000013 FB               [12]  114 	mov	r3, a
      000014 EA               [12]  115 	mov	a, r2
      000015 F0               [24]  116 	movx	@dptr, a
      000016 80 24            [24]  117 	sjmp	sdcc_atomic_exchange_exit
      000018                        118 sdcc_atomic_compare_exchange_idata_impl:
      000018 E6               [12]  119 	mov	a, @r0
      000019 B5 02 02         [24]  120 	cjne	a, ar2, .+#5
      00001C EB               [12]  121 	mov	a, r3
      00001D F6               [12]  122 	mov	@r0, a
      00001E 22               [24]  123 	ret
      00001F 00               [12]  124 	nop
      000020                        125 sdcc_atomic_compare_exchange_pdata_impl:
      000020 E2               [24]  126 	movx	a, @r0
      000021 B5 02 02         [24]  127 	cjne	a, ar2, .+#5
      000024 EB               [12]  128 	mov	a, r3
      000025 F2               [24]  129 	movx	@r0, a
      000026 22               [24]  130 	ret
      000027 00               [12]  131 	nop
      000028                        132 sdcc_atomic_compare_exchange_xdata_impl:
      000028 E0               [24]  133 	movx	a, @dptr
      000029 B5 02 02         [24]  134 	cjne	a, ar2, .+#5
      00002C EB               [12]  135 	mov	a, r3
      00002D F0               [24]  136 	movx	@dptr, a
      00002E 22               [24]  137 	ret
      00002F                        138 sdcc_atomic_exchange_rollback_end::
                                    139 
      00002F                        140 sdcc_atomic_exchange_gptr_impl::
      00002F 30 F6 E0         [24]  141 	jnb	b.6, sdcc_atomic_exchange_xdata_impl
      000032 A8 82            [24]  142 	mov	r0, dpl
      000034 20 F5 D3         [24]  143 	jb	b.5, sdcc_atomic_exchange_pdata_impl
      000037                        144 sdcc_atomic_exchange_idata_impl:
      000037 EA               [12]  145 	mov	a, r2
      000038 C6               [12]  146 	xch	a, @r0
      000039 F5 82            [12]  147 	mov	dpl, a
      00003B 22               [24]  148 	ret
      00003C                        149 sdcc_atomic_exchange_exit:
      00003C 8B 82            [24]  150 	mov	dpl, r3
      00003E 22               [24]  151 	ret
      00003F                        152 sdcc_atomic_compare_exchange_gptr_impl::
      00003F 30 F6 E6         [24]  153 	jnb	b.6, sdcc_atomic_compare_exchange_xdata_impl
      000042 A8 82            [24]  154 	mov	r0, dpl
      000044 20 F5 D9         [24]  155 	jb	b.5, sdcc_atomic_compare_exchange_pdata_impl
      000047 80 CF            [24]  156 	sjmp	sdcc_atomic_compare_exchange_idata_impl
                                    157 ;--------------------------------------------------------
                                    158 ; global & static initialisations
                                    159 ;--------------------------------------------------------
                                    160 	.area HOME    (CODE)
                                    161 	.area GSINIT  (CODE)
                                    162 	.area GSFINAL (CODE)
                                    163 	.area GSINIT  (CODE)
                                    164 	.globl __sdcc_gsinit_startup
                                    165 	.globl __sdcc_program_startup
                                    166 	.globl __start__stack
                                    167 	.globl __mcs51_genXINIT
                                    168 	.globl __mcs51_genXRAMCLEAR
                                    169 	.globl __mcs51_genRAMCLEAR
                                    170 	.area GSFINAL (CODE)
      0000A7 02 00 49         [24]  171 	ljmp	__sdcc_program_startup
                                    172 ;--------------------------------------------------------
                                    173 ; Home
                                    174 ;--------------------------------------------------------
                                    175 	.area HOME    (CODE)
                                    176 	.area HOME    (CODE)
      000049                        177 __sdcc_program_startup:
      000049 12 00 AA         [24]  178 	lcall	_main
      00004C                        179 __sdcc_program_exit:
      00004C 80 FE            [24]  180 	sjmp	.
                                    181 ;	return from main will return to caller
                                    182 ;--------------------------------------------------------
                                    183 ; code
                                    184 ;--------------------------------------------------------
                                    185 	.area CSEG    (CODE)
                                    186 ;------------------------------------------------------------
                                    187 ;Allocation info for local variables in function 'main'
                                    188 ;------------------------------------------------------------
                                    189 ;heartbeat_tick Allocated to registers r6 r7 
                                    190 ;------------------------------------------------------------
                                    191 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:8: void main(void)
                                    192 ;	-----------------------------------------
                                    193 ;	 function main
                                    194 ;	-----------------------------------------
      0000AA                        195 _main:
                           000007   196 	ar7 = 0x07
                           000006   197 	ar6 = 0x06
                           000005   198 	ar5 = 0x05
                           000004   199 	ar4 = 0x04
                           000003   200 	ar3 = 0x03
                           000002   201 	ar2 = 0x02
                           000001   202 	ar1 = 0x01
                           000000   203 	ar0 = 0x00
                                    204 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:10: uint16_t heartbeat_tick = 0U;
      0000AA 7E 00            [12]  205 	mov	r6,#0x00
      0000AC 7F 00            [12]  206 	mov	r7,#0x00
                                    207 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:12: Board_Init();
      0000AE C0 07            [24]  208 	push	ar7
      0000B0 C0 06            [24]  209 	push	ar6
      0000B2 12 01 49         [24]  210 	lcall	_Board_Init
                                    211 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:13: Uart_Init();
      0000B5 12 07 2B         [24]  212 	lcall	_Uart_Init
                                    213 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:14: Pressure_Init();
      0000B8 12 05 26         [24]  214 	lcall	_Pressure_Init
                                    215 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:15: SensorModbus_Init();
      0000BB 12 05 FF         [24]  216 	lcall	_SensorModbus_Init
                                    217 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:16: Display_Init();
      0000BE 12 03 73         [24]  218 	lcall	_Display_Init
                                    219 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:17: Board_GreenLedToggle();
      0000C1 12 01 95         [24]  220 	lcall	_Board_GreenLedToggle
      0000C4 D0 06            [24]  221 	pop	ar6
      0000C6 D0 07            [24]  222 	pop	ar7
      0000C8                        223 00104$:
                                    224 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:24: Pressure_ProcessRx();
      0000C8 C0 07            [24]  225 	push	ar7
      0000CA C0 06            [24]  226 	push	ar6
      0000CC 12 05 39         [24]  227 	lcall	_Pressure_ProcessRx
                                    228 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:25: SensorModbus_Process(Board_ReadAddress(), Pressure_GetValue());
      0000CF 12 01 61         [24]  229 	lcall	_Board_ReadAddress
      0000D2 AD 82            [24]  230 	mov	r5, dpl
      0000D4 C0 05            [24]  231 	push	ar5
      0000D6 12 05 32         [24]  232 	lcall	_Pressure_GetValue
      0000D9 85 82 5E         [24]  233 	mov	_SensorModbus_Process_PARM_2,dpl
      0000DC 85 83 5F         [24]  234 	mov	(_SensorModbus_Process_PARM_2 + 1),dph
      0000DF D0 05            [24]  235 	pop	ar5
      0000E1 8D 82            [24]  236 	mov	dpl, r5
      0000E3 12 06 B5         [24]  237 	lcall	_SensorModbus_Process
                                    238 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:26: Display_SetValue(Pressure_GetValue());
      0000E6 12 05 32         [24]  239 	lcall	_Pressure_GetValue
      0000E9 12 03 88         [24]  240 	lcall	_Display_SetValue
                                    241 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:27: Display_ScanOnce();
      0000EC 12 03 EA         [24]  242 	lcall	_Display_ScanOnce
                                    243 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:30: Board_DelayMs(1U);
      0000EF 90 00 01         [24]  244 	mov	dptr,#0x0001
      0000F2 12 01 98         [24]  245 	lcall	_Board_DelayMs
      0000F5 D0 06            [24]  246 	pop	ar6
      0000F7 D0 07            [24]  247 	pop	ar7
                                    248 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:32: heartbeat_tick+=1U;
      0000F9 8E 04            [24]  249 	mov	ar4,r6
      0000FB 8F 05            [24]  250 	mov	ar5,r7
      0000FD 0C               [12]  251 	inc	r4
      0000FE BC 00 01         [24]  252 	cjne	r4,#0x00,00122$
      000101 0D               [12]  253 	inc	r5
      000102                        254 00122$:
      000102 8C 06            [24]  255 	mov	ar6,r4
      000104 8D 07            [24]  256 	mov	ar7,r5
                                    257 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:33: if (heartbeat_tick >= HEARTBEAT_INTERVAL_MS) {
      000106 8E 04            [24]  258 	mov	ar4,r6
      000108 8F 05            [24]  259 	mov	ar5,r7
      00010A C3               [12]  260 	clr	c
      00010B EC               [12]  261 	mov	a,r4
      00010C 94 F4            [12]  262 	subb	a,#0xf4
      00010E ED               [12]  263 	mov	a,r5
      00010F 94 01            [12]  264 	subb	a,#0x01
      000111 40 B5            [24]  265 	jc	00104$
                                    266 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:34: heartbeat_tick = 0U;
      000113 7E 00            [12]  267 	mov	r6,#0x00
      000115 7F 00            [12]  268 	mov	r7,#0x00
                                    269 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:35: Board_GreenLedToggle();
      000117 C0 07            [24]  270 	push	ar7
      000119 C0 06            [24]  271 	push	ar6
      00011B 12 01 95         [24]  272 	lcall	_Board_GreenLedToggle
      00011E D0 06            [24]  273 	pop	ar6
      000120 D0 07            [24]  274 	pop	ar7
                                    275 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/main.c:38: }
      000122 02 00 C8         [24]  276 	ljmp	00104$
                                    277 	.area CSEG    (CODE)
                                    278 	.area CONST   (CODE)
                                    279 	.area XINIT   (CODE)
                                    280 	.area CABS    (ABS,CODE)
