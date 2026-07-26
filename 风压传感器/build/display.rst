                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler
                                      3 ; Version 4.6.0 #16555 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module display
                                      6 	
                                      7 	.optsdcc -mmcs51 --model-small
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _SensorModbus_Process
                                     12 	.globl _Pressure_GetValue
                                     13 	.globl _Pressure_ProcessRx
                                     14 	.globl _Board_ReadAddress
                                     15 	.globl _P37
                                     16 	.globl _P36
                                     17 	.globl _P35
                                     18 	.globl _P34
                                     19 	.globl _P33
                                     20 	.globl _P32
                                     21 	.globl _P31
                                     22 	.globl _P30
                                     23 	.globl _P27
                                     24 	.globl _P26
                                     25 	.globl _P25
                                     26 	.globl _P24
                                     27 	.globl _P23
                                     28 	.globl _P22
                                     29 	.globl _P21
                                     30 	.globl _P20
                                     31 	.globl _P17
                                     32 	.globl _P16
                                     33 	.globl _P15
                                     34 	.globl _P14
                                     35 	.globl _P13
                                     36 	.globl _P12
                                     37 	.globl _P11
                                     38 	.globl _P10
                                     39 	.globl _P07
                                     40 	.globl _P06
                                     41 	.globl _P05
                                     42 	.globl _P04
                                     43 	.globl _P03
                                     44 	.globl _P02
                                     45 	.globl _P01
                                     46 	.globl _P00
                                     47 	.globl _P4M0
                                     48 	.globl _P4M1
                                     49 	.globl _P3M0
                                     50 	.globl _P3M1
                                     51 	.globl _P2M0
                                     52 	.globl _P2M1
                                     53 	.globl _P0M0
                                     54 	.globl _P0M1
                                     55 	.globl _P1M0
                                     56 	.globl _P1M1
                                     57 	.globl _T2L
                                     58 	.globl _T2H
                                     59 	.globl _IP
                                     60 	.globl _IE
                                     61 	.globl _P_SW1
                                     62 	.globl _S2BUF
                                     63 	.globl _S2CON
                                     64 	.globl _SBUF
                                     65 	.globl _SCON
                                     66 	.globl _AUXR
                                     67 	.globl _TH1
                                     68 	.globl _TH0
                                     69 	.globl _TL1
                                     70 	.globl _TL0
                                     71 	.globl _TMOD
                                     72 	.globl _TCON
                                     73 	.globl _PCON
                                     74 	.globl _DPH
                                     75 	.globl _DPL
                                     76 	.globl _SP
                                     77 	.globl _P3
                                     78 	.globl _P2
                                     79 	.globl _P1
                                     80 	.globl _P0
                                     81 	.globl _Display_ShowRawDigit_PARM_2
                                     82 	.globl _Display_Init
                                     83 	.globl _Display_SetValue
                                     84 	.globl _Display_ScanOnce
                                     85 	.globl _Display_ShowRawDigit
                                     86 	.globl _Display_TestAllOn
                                     87 	.globl _Display_TestPolarity
                                     88 ;--------------------------------------------------------
                                     89 ; special function registers
                                     90 ;--------------------------------------------------------
                                     91 	.area RSEG    (ABS,DATA)
      000000                         92 	.org 0x0000
                           000080    93 _P0	=	0x0080
                           000090    94 _P1	=	0x0090
                           0000A0    95 _P2	=	0x00a0
                           0000B0    96 _P3	=	0x00b0
                           000081    97 _SP	=	0x0081
                           000082    98 _DPL	=	0x0082
                           000083    99 _DPH	=	0x0083
                           000087   100 _PCON	=	0x0087
                           000088   101 _TCON	=	0x0088
                           000089   102 _TMOD	=	0x0089
                           00008A   103 _TL0	=	0x008a
                           00008B   104 _TL1	=	0x008b
                           00008C   105 _TH0	=	0x008c
                           00008D   106 _TH1	=	0x008d
                           00008E   107 _AUXR	=	0x008e
                           000098   108 _SCON	=	0x0098
                           000099   109 _SBUF	=	0x0099
                           00009A   110 _S2CON	=	0x009a
                           00009B   111 _S2BUF	=	0x009b
                           0000A2   112 _P_SW1	=	0x00a2
                           0000A8   113 _IE	=	0x00a8
                           0000B8   114 _IP	=	0x00b8
                           0000D6   115 _T2H	=	0x00d6
                           0000D7   116 _T2L	=	0x00d7
                           000091   117 _P1M1	=	0x0091
                           000092   118 _P1M0	=	0x0092
                           000093   119 _P0M1	=	0x0093
                           000094   120 _P0M0	=	0x0094
                           000095   121 _P2M1	=	0x0095
                           000096   122 _P2M0	=	0x0096
                           0000B1   123 _P3M1	=	0x00b1
                           0000B2   124 _P3M0	=	0x00b2
                           0000BA   125 _P4M1	=	0x00ba
                           0000BB   126 _P4M0	=	0x00bb
                                    127 ;--------------------------------------------------------
                                    128 ; special function bits
                                    129 ;--------------------------------------------------------
                                    130 	.area RSEG    (ABS,DATA)
      000000                        131 	.org 0x0000
                           000080   132 _P00	=	0x0080
                           000081   133 _P01	=	0x0081
                           000082   134 _P02	=	0x0082
                           000083   135 _P03	=	0x0083
                           000084   136 _P04	=	0x0084
                           000085   137 _P05	=	0x0085
                           000086   138 _P06	=	0x0086
                           000087   139 _P07	=	0x0087
                           000090   140 _P10	=	0x0090
                           000091   141 _P11	=	0x0091
                           000092   142 _P12	=	0x0092
                           000093   143 _P13	=	0x0093
                           000094   144 _P14	=	0x0094
                           000095   145 _P15	=	0x0095
                           000096   146 _P16	=	0x0096
                           000097   147 _P17	=	0x0097
                           0000A0   148 _P20	=	0x00a0
                           0000A1   149 _P21	=	0x00a1
                           0000A2   150 _P22	=	0x00a2
                           0000A3   151 _P23	=	0x00a3
                           0000A4   152 _P24	=	0x00a4
                           0000A5   153 _P25	=	0x00a5
                           0000A6   154 _P26	=	0x00a6
                           0000A7   155 _P27	=	0x00a7
                           0000B0   156 _P30	=	0x00b0
                           0000B1   157 _P31	=	0x00b1
                           0000B2   158 _P32	=	0x00b2
                           0000B3   159 _P33	=	0x00b3
                           0000B4   160 _P34	=	0x00b4
                           0000B5   161 _P35	=	0x00b5
                           0000B6   162 _P36	=	0x00b6
                           0000B7   163 _P37	=	0x00b7
                                    164 ;--------------------------------------------------------
                                    165 ; overlayable register banks
                                    166 ;--------------------------------------------------------
                                    167 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                        168 	.ds 8
                                    169 ;--------------------------------------------------------
                                    170 ; internal ram data
                                    171 ;--------------------------------------------------------
                                    172 	.area DSEG    (DATA)
      00000E                        173 _g_digits:
      00000E                        174 	.ds 3
      000011                        175 _g_scan_index:
      000011                        176 	.ds 1
      000012                        177 _Display_ShowRawDigit_PARM_2:
      000012                        178 	.ds 1
                                    179 ;--------------------------------------------------------
                                    180 ; overlayable items in internal ram
                                    181 ;--------------------------------------------------------
                                    182 	.area	OSEG    (OVR,DATA)
                                    183 	.area	OSEG    (OVR,DATA)
                                    184 	.area	OSEG    (OVR,DATA)
                                    185 ;--------------------------------------------------------
                                    186 ; indirectly addressable internal ram data
                                    187 ;--------------------------------------------------------
                                    188 	.area ISEG    (DATA)
                                    189 ;--------------------------------------------------------
                                    190 ; absolute internal ram data
                                    191 ;--------------------------------------------------------
                                    192 	.area IABS    (ABS,DATA)
                                    193 	.area IABS    (ABS,DATA)
                                    194 ;--------------------------------------------------------
                                    195 ; bit data
                                    196 ;--------------------------------------------------------
                                    197 	.area BSEG    (BIT)
                                    198 ;--------------------------------------------------------
                                    199 ; paged external ram data
                                    200 ;--------------------------------------------------------
                                    201 	.area PSEG    (PAG,XDATA)
                                    202 ;--------------------------------------------------------
                                    203 ; uninitialized external ram data
                                    204 ;--------------------------------------------------------
                                    205 	.area XSEG    (XDATA)
                                    206 ;--------------------------------------------------------
                                    207 ; absolute external ram data
                                    208 ;--------------------------------------------------------
                                    209 	.area XABS    (ABS,XDATA)
                                    210 ;--------------------------------------------------------
                                    211 ; initialized external ram data
                                    212 ;--------------------------------------------------------
                                    213 	.area XISEG   (XDATA)
                                    214 	.area HOME    (CODE)
                                    215 	.area GSINIT0 (CODE)
                                    216 	.area GSINIT1 (CODE)
                                    217 	.area GSINIT2 (CODE)
                                    218 	.area GSINIT3 (CODE)
                                    219 	.area GSINIT4 (CODE)
                                    220 	.area GSINIT5 (CODE)
                                    221 	.area GSINIT  (CODE)
                                    222 	.area GSFINAL (CODE)
                                    223 	.area CSEG    (CODE)
                                    224 ;--------------------------------------------------------
                                    225 ; global & static initialisations
                                    226 ;--------------------------------------------------------
                                    227 	.area HOME    (CODE)
                                    228 	.area GSINIT  (CODE)
                                    229 	.area GSFINAL (CODE)
                                    230 	.area GSINIT  (CODE)
                                    231 ;--------------------------------------------------------
                                    232 ; Home
                                    233 ;--------------------------------------------------------
                                    234 	.area HOME    (CODE)
                                    235 	.area HOME    (CODE)
                                    236 ;--------------------------------------------------------
                                    237 ; code
                                    238 ;--------------------------------------------------------
                                    239 	.area CSEG    (CODE)
                                    240 ;------------------------------------------------------------
                                    241 ;Allocation info for local variables in function 'Display_AllDigitsOff'
                                    242 ;------------------------------------------------------------
                                    243 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:15: static void Display_AllDigitsOff(void)
                                    244 ;	-----------------------------------------
                                    245 ;	 function Display_AllDigitsOff
                                    246 ;	-----------------------------------------
      0002ED                        247 _Display_AllDigitsOff:
                           000007   248 	ar7 = 0x07
                           000006   249 	ar6 = 0x06
                           000005   250 	ar5 = 0x05
                           000004   251 	ar4 = 0x04
                           000003   252 	ar3 = 0x03
                           000002   253 	ar2 = 0x02
                           000001   254 	ar1 = 0x01
                           000000   255 	ar0 = 0x00
                                    256 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:17: P23 = 1;
                                    257 ;	assignBit
      0002ED D2 A3            [12]  258 	setb	_P23
                                    259 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:18: P26 = 1;
                                    260 ;	assignBit
      0002EF D2 A6            [12]  261 	setb	_P26
                                    262 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:19: P01 = 1;
                                    263 ;	assignBit
      0002F1 D2 81            [12]  264 	setb	_P01
                                    265 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:20: }
      0002F3 22               [24]  266 	ret
                                    267 ;------------------------------------------------------------
                                    268 ;Allocation info for local variables in function 'Display_SetSegments'
                                    269 ;------------------------------------------------------------
                                    270 ;pattern       Allocated to registers r7 
                                    271 ;------------------------------------------------------------
                                    272 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:22: static void Display_SetSegments(uint8_t pattern)
                                    273 ;	-----------------------------------------
                                    274 ;	 function Display_SetSegments
                                    275 ;	-----------------------------------------
      0002F4                        276 _Display_SetSegments:
      0002F4 AF 82            [24]  277 	mov	r7, dpl
                                    278 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:24: P24 = (pattern & 0x01U) ? 1 : 0;
      0002F6 74 01            [12]  279 	mov	a,#0x01
      0002F8 5F               [12]  280 	anl	a,r7
                                    281 ;	assignBit
      0002F9 24 FF            [12]  282 	add	a,#0xff
      0002FB 92 A4            [24]  283 	mov	_P24,c
                                    284 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:25: P27 = (pattern & 0x02U) ? 1 : 0;
      0002FD 74 02            [12]  285 	mov	a,#0x02
      0002FF 5F               [12]  286 	anl	a,r7
                                    287 ;	assignBit
      000300 24 FF            [12]  288 	add	a,#0xff
      000302 92 A7            [24]  289 	mov	_P27,c
                                    290 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:26: P36 = (pattern & 0x04U) ? 1 : 0;
      000304 74 04            [12]  291 	mov	a,#0x04
      000306 5F               [12]  292 	anl	a,r7
                                    293 ;	assignBit
      000307 24 FF            [12]  294 	add	a,#0xff
      000309 92 B6            [24]  295 	mov	_P36,c
                                    296 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:27: P20 = (pattern & 0x08U) ? 1 : 0;
      00030B 74 08            [12]  297 	mov	a,#0x08
      00030D 5F               [12]  298 	anl	a,r7
                                    299 ;	assignBit
      00030E 24 FF            [12]  300 	add	a,#0xff
      000310 92 A0            [24]  301 	mov	_P20,c
                                    302 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:28: P21 = (pattern & 0x10U) ? 1 : 0;
      000312 74 10            [12]  303 	mov	a,#0x10
      000314 5F               [12]  304 	anl	a,r7
                                    305 ;	assignBit
      000315 24 FF            [12]  306 	add	a,#0xff
      000317 92 A1            [24]  307 	mov	_P21,c
                                    308 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:29: P25 = (pattern & 0x20U) ? 1 : 0;
      000319 74 20            [12]  309 	mov	a,#0x20
      00031B 5F               [12]  310 	anl	a,r7
                                    311 ;	assignBit
      00031C 24 FF            [12]  312 	add	a,#0xff
      00031E 92 A5            [24]  313 	mov	_P25,c
                                    314 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:30: P35 = (pattern & 0x40U) ? 1 : 0;
      000320 53 07 40         [24]  315 	anl	ar7,#0x40
                                    316 ;	assignBit
      000323 EF               [12]  317 	mov	a,r7
      000324 24 FF            [12]  318 	add	a,#0xff
      000326 92 B5            [24]  319 	mov	_P35,c
                                    320 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:31: P37 = 0;
                                    321 ;	assignBit
      000328 C2 B7            [12]  322 	clr	_P37
                                    323 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:32: }
      00032A 22               [24]  324 	ret
                                    325 ;------------------------------------------------------------
                                    326 ;Allocation info for local variables in function 'Display_EnableDigit'
                                    327 ;------------------------------------------------------------
                                    328 ;index         Allocated to registers r7 
                                    329 ;------------------------------------------------------------
                                    330 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:34: static void Display_EnableDigit(uint8_t index)
                                    331 ;	-----------------------------------------
                                    332 ;	 function Display_EnableDigit
                                    333 ;	-----------------------------------------
      00032B                        334 _Display_EnableDigit:
                                    335 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:36: if (index == 0U) {
      00032B E5 82            [12]  336 	mov	a,dpl
      00032D FF               [12]  337 	mov	r7,a
      00032E 70 03            [24]  338 	jnz	00105$
                                    339 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:37: P23 = 0;
                                    340 ;	assignBit
      000330 C2 A3            [12]  341 	clr	_P23
      000332 22               [24]  342 	ret
      000333                        343 00105$:
                                    344 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:38: } else if (index == 1U) {
      000333 BF 01 03         [24]  345 	cjne	r7,#0x01,00102$
                                    346 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:39: P26 = 0;
                                    347 ;	assignBit
      000336 C2 A6            [12]  348 	clr	_P26
      000338 22               [24]  349 	ret
      000339                        350 00102$:
                                    351 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:41: P01 = 0;
                                    352 ;	assignBit
      000339 C2 81            [12]  353 	clr	_P01
                                    354 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:43: }
      00033B 22               [24]  355 	ret
                                    356 ;------------------------------------------------------------
                                    357 ;Allocation info for local variables in function 'Display_WaitAndPollRx'
                                    358 ;------------------------------------------------------------
                                    359 ;count         Allocated to registers 
                                    360 ;------------------------------------------------------------
                                    361 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:45: static void Display_WaitAndPollRx(uint16_t count)
                                    362 ;	-----------------------------------------
                                    363 ;	 function Display_WaitAndPollRx
                                    364 ;	-----------------------------------------
      00033C                        365 _Display_WaitAndPollRx:
      00033C AE 82            [24]  366 	mov	r6, dpl
      00033E AF 83            [24]  367 	mov	r7, dph
                                    368 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:47: while (count-- != 0U) {
      000340                        369 00101$:
      000340 8E 04            [24]  370 	mov	ar4,r6
      000342 8F 05            [24]  371 	mov	ar5,r7
      000344 1E               [12]  372 	dec	r6
      000345 BE FF 01         [24]  373 	cjne	r6,#0xff,00121$
      000348 1F               [12]  374 	dec	r7
      000349                        375 00121$:
      000349 EC               [12]  376 	mov	a,r4
      00034A 4D               [12]  377 	orl	a,r5
      00034B 60 25            [24]  378 	jz	00104$
                                    379 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:48: Pressure_ProcessRx();
      00034D C0 07            [24]  380 	push	ar7
      00034F C0 06            [24]  381 	push	ar6
      000351 12 05 39         [24]  382 	lcall	_Pressure_ProcessRx
                                    383 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:49: SensorModbus_Process(Board_ReadAddress(), Pressure_GetValue());
      000354 12 01 61         [24]  384 	lcall	_Board_ReadAddress
      000357 AD 82            [24]  385 	mov	r5, dpl
      000359 C0 05            [24]  386 	push	ar5
      00035B 12 05 32         [24]  387 	lcall	_Pressure_GetValue
      00035E 85 82 5E         [24]  388 	mov	_SensorModbus_Process_PARM_2,dpl
      000361 85 83 5F         [24]  389 	mov	(_SensorModbus_Process_PARM_2 + 1),dph
      000364 D0 05            [24]  390 	pop	ar5
      000366 8D 82            [24]  391 	mov	dpl, r5
      000368 12 06 B5         [24]  392 	lcall	_SensorModbus_Process
      00036B D0 06            [24]  393 	pop	ar6
      00036D D0 07            [24]  394 	pop	ar7
                                    395 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:50: __asm nop __endasm;
      00036F 00               [12]  396 	nop	
      000370 80 CE            [24]  397 	sjmp	00101$
      000372                        398 00104$:
                                    399 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:52: }
      000372 22               [24]  400 	ret
                                    401 ;------------------------------------------------------------
                                    402 ;Allocation info for local variables in function 'Display_Init'
                                    403 ;------------------------------------------------------------
                                    404 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:54: void Display_Init(void)
                                    405 ;	-----------------------------------------
                                    406 ;	 function Display_Init
                                    407 ;	-----------------------------------------
      000373                        408 _Display_Init:
                                    409 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:56: g_digits[0] = 0U;
      000373 75 0E 00         [24]  410 	mov	_g_digits,#0x00
                                    411 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:57: g_digits[1] = 0U;
      000376 75 0F 00         [24]  412 	mov	(_g_digits + 0x0001),#0x00
                                    413 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:58: g_digits[2] = 0U;
      000379 75 10 00         [24]  414 	mov	(_g_digits + 0x0002),#0x00
                                    415 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:59: g_scan_index = 0U;
      00037C 75 11 00         [24]  416 	mov	_g_scan_index,#0x00
                                    417 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:60: Display_AllDigitsOff();
      00037F 12 02 ED         [24]  418 	lcall	_Display_AllDigitsOff
                                    419 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:61: Display_SetSegments(0U);
      000382 75 82 00         [24]  420 	mov	dpl, #0x00
                                    421 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:62: }
      000385 02 02 F4         [24]  422 	ljmp	_Display_SetSegments
                                    423 ;------------------------------------------------------------
                                    424 ;Allocation info for local variables in function 'Display_SetValue'
                                    425 ;------------------------------------------------------------
                                    426 ;value         Allocated to registers r6 r7 
                                    427 ;------------------------------------------------------------
                                    428 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:64: void Display_SetValue(uint16_t value)
                                    429 ;	-----------------------------------------
                                    430 ;	 function Display_SetValue
                                    431 ;	-----------------------------------------
      000388                        432 _Display_SetValue:
      000388 AE 82            [24]  433 	mov	r6, dpl
      00038A AF 83            [24]  434 	mov	r7, dph
                                    435 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:66: if (value > 999U) {
      00038C 8E 04            [24]  436 	mov	ar4,r6
      00038E 8F 05            [24]  437 	mov	ar5,r7
      000390 C3               [12]  438 	clr	c
      000391 74 E7            [12]  439 	mov	a,#0xe7
      000393 9C               [12]  440 	subb	a,r4
      000394 74 03            [12]  441 	mov	a,#0x03
      000396 9D               [12]  442 	subb	a,r5
      000397 50 04            [24]  443 	jnc	00102$
                                    444 ;	free result
                                    445 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:67: value = 999U;
      000399 7E E7            [12]  446 	mov	r6,#0xe7
      00039B 7F 03            [12]  447 	mov	r7,#0x03
      00039D                        448 00102$:
                                    449 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:70: g_digits[0] = (uint8_t)(value / 100U);
      00039D 75 15 64         [24]  450 	mov	__divuint_PARM_2,#0x64
      0003A0 75 16 00         [24]  451 	mov	(__divuint_PARM_2 + 1),#0x00
      0003A3 8E 82            [24]  452 	mov	dpl, r6
      0003A5 8F 83            [24]  453 	mov	dph, r7
      0003A7 C0 07            [24]  454 	push	ar7
      0003A9 C0 06            [24]  455 	push	ar6
      0003AB 12 08 2F         [24]  456 	lcall	__divuint
      0003AE AC 82            [24]  457 	mov	r4, dpl
      0003B0 D0 06            [24]  458 	pop	ar6
      0003B2 D0 07            [24]  459 	pop	ar7
      0003B4 8C 0E            [24]  460 	mov	_g_digits,r4
                                    461 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:71: g_digits[1] = (uint8_t)((value / 10U) % 10U);
      0003B6 75 15 0A         [24]  462 	mov	__divuint_PARM_2,#0x0a
      0003B9 75 16 00         [24]  463 	mov	(__divuint_PARM_2 + 1),#0x00
      0003BC 8E 82            [24]  464 	mov	dpl, r6
      0003BE 8F 83            [24]  465 	mov	dph, r7
      0003C0 C0 07            [24]  466 	push	ar7
      0003C2 C0 06            [24]  467 	push	ar6
      0003C4 12 08 2F         [24]  468 	lcall	__divuint
      0003C7 75 15 0A         [24]  469 	mov	__moduint_PARM_2,#0x0a
      0003CA 75 16 00         [24]  470 	mov	(__moduint_PARM_2 + 1),#0x00
      0003CD 12 08 D8         [24]  471 	lcall	__moduint
      0003D0 AC 82            [24]  472 	mov	r4, dpl
      0003D2 D0 06            [24]  473 	pop	ar6
      0003D4 D0 07            [24]  474 	pop	ar7
      0003D6 8C 0F            [24]  475 	mov	(_g_digits + 0x0001),r4
                                    476 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:72: g_digits[2] = (uint8_t)(value % 10U);
      0003D8 75 15 0A         [24]  477 	mov	__moduint_PARM_2,#0x0a
      0003DB 75 16 00         [24]  478 	mov	(__moduint_PARM_2 + 1),#0x00
      0003DE 8E 82            [24]  479 	mov	dpl, r6
      0003E0 8F 83            [24]  480 	mov	dph, r7
      0003E2 12 08 D8         [24]  481 	lcall	__moduint
      0003E5 AE 82            [24]  482 	mov	r6, dpl
      0003E7 8E 10            [24]  483 	mov	(_g_digits + 0x0002),r6
                                    484 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:73: }
      0003E9 22               [24]  485 	ret
                                    486 ;------------------------------------------------------------
                                    487 ;Allocation info for local variables in function 'Display_ScanOnce'
                                    488 ;------------------------------------------------------------
                                    489 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:75: void Display_ScanOnce(void)
                                    490 ;	-----------------------------------------
                                    491 ;	 function Display_ScanOnce
                                    492 ;	-----------------------------------------
      0003EA                        493 _Display_ScanOnce:
                                    494 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:77: Display_AllDigitsOff();
      0003EA 12 02 ED         [24]  495 	lcall	_Display_AllDigitsOff
                                    496 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:78: Display_SetSegments(0U);
      0003ED 75 82 00         [24]  497 	mov	dpl, #0x00
      0003F0 12 02 F4         [24]  498 	lcall	_Display_SetSegments
                                    499 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:79: Display_WaitAndPollRx(10U);
      0003F3 90 00 0A         [24]  500 	mov	dptr,#0x000a
      0003F6 12 03 3C         [24]  501 	lcall	_Display_WaitAndPollRx
                                    502 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:81: Display_SetSegments(g_seg_table[g_digits[g_scan_index]]);
      0003F9 E5 11            [12]  503 	mov	a,_g_scan_index
      0003FB 24 0E            [12]  504 	add	a, #_g_digits
      0003FD F9               [12]  505 	mov	r1,a
      0003FE E7               [12]  506 	mov	a,@r1
      0003FF 90 09 45         [24]  507 	mov	dptr,#_g_seg_table
      000402 93               [24]  508 	movc	a,@a+dptr
      000403 F5 82            [12]  509 	mov	dpl,a
      000405 12 02 F4         [24]  510 	lcall	_Display_SetSegments
                                    511 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:82: Display_WaitAndPollRx(10U);
      000408 90 00 0A         [24]  512 	mov	dptr,#0x000a
      00040B 12 03 3C         [24]  513 	lcall	_Display_WaitAndPollRx
                                    514 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:84: Display_EnableDigit(g_scan_index);
      00040E 85 11 82         [24]  515 	mov	dpl, _g_scan_index
      000411 12 03 2B         [24]  516 	lcall	_Display_EnableDigit
                                    517 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:85: Display_WaitAndPollRx(600U);
      000414 90 02 58         [24]  518 	mov	dptr,#0x0258
      000417 12 03 3C         [24]  519 	lcall	_Display_WaitAndPollRx
                                    520 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:86: Display_AllDigitsOff();
      00041A 12 02 ED         [24]  521 	lcall	_Display_AllDigitsOff
                                    522 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:88: g_scan_index++;
      00041D 05 11            [12]  523 	inc	_g_scan_index
                                    524 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:89: if (g_scan_index >= 3U) {
      00041F 74 FD            [12]  525 	mov	a,#0x100 - 0x03
      000421 25 11            [12]  526 	add	a,_g_scan_index
      000423 50 03            [24]  527 	jnc	00103$
                                    528 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:90: g_scan_index = 0U;
      000425 75 11 00         [24]  529 	mov	_g_scan_index,#0x00
      000428                        530 00103$:
                                    531 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:92: }
      000428 22               [24]  532 	ret
                                    533 ;------------------------------------------------------------
                                    534 ;Allocation info for local variables in function 'Display_ShowRawDigit'
                                    535 ;------------------------------------------------------------
                                    536 ;value         Allocated with name '_Display_ShowRawDigit_PARM_2'
                                    537 ;digit         Allocated to registers r7 
                                    538 ;------------------------------------------------------------
                                    539 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:94: void Display_ShowRawDigit(uint8_t digit, uint8_t value)
                                    540 ;	-----------------------------------------
                                    541 ;	 function Display_ShowRawDigit
                                    542 ;	-----------------------------------------
      000429                        543 _Display_ShowRawDigit:
      000429 AF 82            [24]  544 	mov	r7, dpl
                                    545 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:96: if (value > 9U) {
      00042B E5 12            [12]  546 	mov	a,_Display_ShowRawDigit_PARM_2
      00042D 24 F6            [12]  547 	add	a,#0xff - 0x09
      00042F 50 03            [24]  548 	jnc	00102$
                                    549 ;	free result
                                    550 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:97: value = 9U;
      000431 75 12 09         [24]  551 	mov	_Display_ShowRawDigit_PARM_2,#0x09
      000434                        552 00102$:
                                    553 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:100: Display_AllDigitsOff();
      000434 C0 07            [24]  554 	push	ar7
      000436 12 02 ED         [24]  555 	lcall	_Display_AllDigitsOff
                                    556 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:101: Display_SetSegments(0U);
      000439 75 82 00         [24]  557 	mov	dpl, #0x00
      00043C 12 02 F4         [24]  558 	lcall	_Display_SetSegments
      00043F D0 07            [24]  559 	pop	ar7
                                    560 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:102: Display_SetSegments(g_seg_table[value]);
      000441 E5 12            [12]  561 	mov	a,_Display_ShowRawDigit_PARM_2
      000443 90 09 45         [24]  562 	mov	dptr,#_g_seg_table
      000446 93               [24]  563 	movc	a,@a+dptr
      000447 F5 82            [12]  564 	mov	dpl,a
      000449 C0 07            [24]  565 	push	ar7
      00044B 12 02 F4         [24]  566 	lcall	_Display_SetSegments
      00044E D0 07            [24]  567 	pop	ar7
                                    568 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:103: Display_EnableDigit(digit);
      000450 8F 82            [24]  569 	mov	dpl, r7
                                    570 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:104: }
      000452 02 03 2B         [24]  571 	ljmp	_Display_EnableDigit
                                    572 ;------------------------------------------------------------
                                    573 ;Allocation info for local variables in function 'Display_TestAllOn'
                                    574 ;------------------------------------------------------------
                                    575 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:106: void Display_TestAllOn(void)
                                    576 ;	-----------------------------------------
                                    577 ;	 function Display_TestAllOn
                                    578 ;	-----------------------------------------
      000455                        579 _Display_TestAllOn:
                                    580 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:108: P24 = 1;
                                    581 ;	assignBit
      000455 D2 A4            [12]  582 	setb	_P24
                                    583 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:109: P27 = 1;
                                    584 ;	assignBit
      000457 D2 A7            [12]  585 	setb	_P27
                                    586 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:110: P36 = 1;
                                    587 ;	assignBit
      000459 D2 B6            [12]  588 	setb	_P36
                                    589 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:111: P20 = 1;
                                    590 ;	assignBit
      00045B D2 A0            [12]  591 	setb	_P20
                                    592 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:112: P21 = 1;
                                    593 ;	assignBit
      00045D D2 A1            [12]  594 	setb	_P21
                                    595 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:113: P25 = 1;
                                    596 ;	assignBit
      00045F D2 A5            [12]  597 	setb	_P25
                                    598 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:114: P35 = 1;
                                    599 ;	assignBit
      000461 D2 B5            [12]  600 	setb	_P35
                                    601 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:115: P37 = 1;
                                    602 ;	assignBit
      000463 D2 B7            [12]  603 	setb	_P37
                                    604 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:117: P23 = 0;
                                    605 ;	assignBit
      000465 C2 A3            [12]  606 	clr	_P23
                                    607 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:118: P26 = 0;
                                    608 ;	assignBit
      000467 C2 A6            [12]  609 	clr	_P26
                                    610 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:119: P01 = 0;
                                    611 ;	assignBit
      000469 C2 81            [12]  612 	clr	_P01
                                    613 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:120: }
      00046B 22               [24]  614 	ret
                                    615 ;------------------------------------------------------------
                                    616 ;Allocation info for local variables in function 'Display_TestPolarity'
                                    617 ;------------------------------------------------------------
                                    618 ;mode          Allocated to registers r7 
                                    619 ;seg_on        Allocated to registers r6 
                                    620 ;dig_on        Allocated to registers r7 
                                    621 ;------------------------------------------------------------
                                    622 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:122: void Display_TestPolarity(uint8_t mode)
                                    623 ;	-----------------------------------------
                                    624 ;	 function Display_TestPolarity
                                    625 ;	-----------------------------------------
      00046C                        626 _Display_TestPolarity:
                                    627 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:124: uint8_t seg_on = (mode & 0x01U) ? 1U : 0U;
      00046C E5 82            [12]  628 	mov	a,dpl
      00046E FF               [12]  629 	mov	r7,a
      00046F 30 E0 04         [24]  630 	jnb	acc.0,00103$
      000472 7E 01            [12]  631 	mov	r6,#0x01
      000474 80 02            [24]  632 	sjmp	00104$
      000476                        633 00103$:
      000476 7E 00            [12]  634 	mov	r6,#0x00
      000478                        635 00104$:
                                    636 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:125: uint8_t dig_on = (mode & 0x02U) ? 1U : 0U;
      000478 EF               [12]  637 	mov	a,r7
      000479 30 E1 04         [24]  638 	jnb	acc.1,00105$
      00047C 7F 01            [12]  639 	mov	r7,#0x01
      00047E 80 02            [24]  640 	sjmp	00106$
      000480                        641 00105$:
      000480 7F 00            [12]  642 	mov	r7,#0x00
      000482                        643 00106$:
                                    644 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:127: P24 = seg_on;
                                    645 ;	assignBit
      000482 EE               [12]  646 	mov	a,r6
      000483 24 FF            [12]  647 	add	a,#0xff
      000485 E4               [12]  648 	clr	a
      000486 33               [12]  649 	rlc	a
                                    650 ;	assignBit
      000487 FE               [12]  651 	mov	r6,a
      000488 24 FF            [12]  652 	add	a,#0xff
      00048A 92 A4            [24]  653 	mov	_P24,c
                                    654 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:128: P27 = seg_on;
                                    655 ;	assignBit
      00048C EE               [12]  656 	mov	a,r6
      00048D 24 FF            [12]  657 	add	a,#0xff
      00048F 92 A7            [24]  658 	mov	_P27,c
                                    659 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:129: P36 = seg_on;
                                    660 ;	assignBit
      000491 EE               [12]  661 	mov	a,r6
      000492 24 FF            [12]  662 	add	a,#0xff
      000494 92 B6            [24]  663 	mov	_P36,c
                                    664 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:130: P20 = seg_on;
                                    665 ;	assignBit
      000496 EE               [12]  666 	mov	a,r6
      000497 24 FF            [12]  667 	add	a,#0xff
      000499 92 A0            [24]  668 	mov	_P20,c
                                    669 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:131: P21 = seg_on;
                                    670 ;	assignBit
      00049B EE               [12]  671 	mov	a,r6
      00049C 24 FF            [12]  672 	add	a,#0xff
      00049E 92 A1            [24]  673 	mov	_P21,c
                                    674 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:132: P25 = seg_on;
                                    675 ;	assignBit
      0004A0 EE               [12]  676 	mov	a,r6
      0004A1 24 FF            [12]  677 	add	a,#0xff
      0004A3 92 A5            [24]  678 	mov	_P25,c
                                    679 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:133: P35 = seg_on;
                                    680 ;	assignBit
      0004A5 EE               [12]  681 	mov	a,r6
      0004A6 24 FF            [12]  682 	add	a,#0xff
      0004A8 92 B5            [24]  683 	mov	_P35,c
                                    684 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:134: P37 = seg_on;
                                    685 ;	assignBit
      0004AA EE               [12]  686 	mov	a,r6
      0004AB 24 FF            [12]  687 	add	a,#0xff
      0004AD 92 B7            [24]  688 	mov	_P37,c
                                    689 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:136: P23 = dig_on;
                                    690 ;	assignBit
      0004AF EF               [12]  691 	mov	a,r7
      0004B0 24 FF            [12]  692 	add	a,#0xff
      0004B2 E4               [12]  693 	clr	a
      0004B3 33               [12]  694 	rlc	a
                                    695 ;	assignBit
      0004B4 FF               [12]  696 	mov	r7,a
      0004B5 24 FF            [12]  697 	add	a,#0xff
      0004B7 92 A3            [24]  698 	mov	_P23,c
                                    699 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:137: P26 = dig_on;
                                    700 ;	assignBit
      0004B9 EF               [12]  701 	mov	a,r7
      0004BA 24 FF            [12]  702 	add	a,#0xff
      0004BC 92 A6            [24]  703 	mov	_P26,c
                                    704 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:138: P01 = dig_on;
                                    705 ;	assignBit
      0004BE EF               [12]  706 	mov	a,r7
      0004BF 24 FF            [12]  707 	add	a,#0xff
      0004C1 92 81            [24]  708 	mov	_P01,c
                                    709 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/display.c:139: }
      0004C3 22               [24]  710 	ret
                                    711 	.area CSEG    (CODE)
                                    712 	.area CONST   (CODE)
                                    713 	.area CONST   (CODE)
      000945                        714 _g_seg_table:
      000945 3F                     715 	.db #0x3f	; 63
      000946 06                     716 	.db #0x06	; 6
      000947 5B                     717 	.db #0x5b	; 91
      000948 4F                     718 	.db #0x4f	; 79	'O'
      000949 66                     719 	.db #0x66	; 102	'f'
      00094A 6D                     720 	.db #0x6d	; 109	'm'
      00094B 7D                     721 	.db #0x7d	; 125
      00094C 07                     722 	.db #0x07	; 7
      00094D 7F                     723 	.db #0x7f	; 127
      00094E 6F                     724 	.db #0x6f	; 111	'o'
                                    725 	.area CSEG    (CODE)
                                    726 	.area XINIT   (CODE)
                                    727 	.area CABS    (ABS,CODE)
