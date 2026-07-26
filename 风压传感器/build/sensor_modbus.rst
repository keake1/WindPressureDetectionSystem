                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler
                                      3 ; Version 4.6.0 #16555 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module sensor_modbus
                                      6 	
                                      7 	.optsdcc -mmcs51 --model-small
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _Uart2_Send
                                     12 	.globl _Uart2_ReadByte
                                     13 	.globl _Crc16_Append
                                     14 	.globl _Crc16_Check
                                     15 	.globl _Board_RedLedSet
                                     16 	.globl _P37
                                     17 	.globl _P36
                                     18 	.globl _P35
                                     19 	.globl _P34
                                     20 	.globl _P33
                                     21 	.globl _P32
                                     22 	.globl _P31
                                     23 	.globl _P30
                                     24 	.globl _P27
                                     25 	.globl _P26
                                     26 	.globl _P25
                                     27 	.globl _P24
                                     28 	.globl _P23
                                     29 	.globl _P22
                                     30 	.globl _P21
                                     31 	.globl _P20
                                     32 	.globl _P17
                                     33 	.globl _P16
                                     34 	.globl _P15
                                     35 	.globl _P14
                                     36 	.globl _P13
                                     37 	.globl _P12
                                     38 	.globl _P11
                                     39 	.globl _P10
                                     40 	.globl _P07
                                     41 	.globl _P06
                                     42 	.globl _P05
                                     43 	.globl _P04
                                     44 	.globl _P03
                                     45 	.globl _P02
                                     46 	.globl _P01
                                     47 	.globl _P00
                                     48 	.globl _P4M0
                                     49 	.globl _P4M1
                                     50 	.globl _P3M0
                                     51 	.globl _P3M1
                                     52 	.globl _P2M0
                                     53 	.globl _P2M1
                                     54 	.globl _P0M0
                                     55 	.globl _P0M1
                                     56 	.globl _P1M0
                                     57 	.globl _P1M1
                                     58 	.globl _T2L
                                     59 	.globl _T2H
                                     60 	.globl _IP
                                     61 	.globl _IE
                                     62 	.globl _P_SW1
                                     63 	.globl _S2BUF
                                     64 	.globl _S2CON
                                     65 	.globl _SBUF
                                     66 	.globl _SCON
                                     67 	.globl _AUXR
                                     68 	.globl _TH1
                                     69 	.globl _TH0
                                     70 	.globl _TL1
                                     71 	.globl _TL0
                                     72 	.globl _TMOD
                                     73 	.globl _TCON
                                     74 	.globl _PCON
                                     75 	.globl _DPH
                                     76 	.globl _DPL
                                     77 	.globl _SP
                                     78 	.globl _P3
                                     79 	.globl _P2
                                     80 	.globl _P1
                                     81 	.globl _P0
                                     82 	.globl _SensorModbus_Process_PARM_2
                                     83 	.globl _SensorModbus_Init
                                     84 	.globl _SensorModbus_Process
                                     85 ;--------------------------------------------------------
                                     86 ; special function registers
                                     87 ;--------------------------------------------------------
                                     88 	.area RSEG    (ABS,DATA)
      000000                         89 	.org 0x0000
                           000080    90 _P0	=	0x0080
                           000090    91 _P1	=	0x0090
                           0000A0    92 _P2	=	0x00a0
                           0000B0    93 _P3	=	0x00b0
                           000081    94 _SP	=	0x0081
                           000082    95 _DPL	=	0x0082
                           000083    96 _DPH	=	0x0083
                           000087    97 _PCON	=	0x0087
                           000088    98 _TCON	=	0x0088
                           000089    99 _TMOD	=	0x0089
                           00008A   100 _TL0	=	0x008a
                           00008B   101 _TL1	=	0x008b
                           00008C   102 _TH0	=	0x008c
                           00008D   103 _TH1	=	0x008d
                           00008E   104 _AUXR	=	0x008e
                           000098   105 _SCON	=	0x0098
                           000099   106 _SBUF	=	0x0099
                           00009A   107 _S2CON	=	0x009a
                           00009B   108 _S2BUF	=	0x009b
                           0000A2   109 _P_SW1	=	0x00a2
                           0000A8   110 _IE	=	0x00a8
                           0000B8   111 _IP	=	0x00b8
                           0000D6   112 _T2H	=	0x00d6
                           0000D7   113 _T2L	=	0x00d7
                           000091   114 _P1M1	=	0x0091
                           000092   115 _P1M0	=	0x0092
                           000093   116 _P0M1	=	0x0093
                           000094   117 _P0M0	=	0x0094
                           000095   118 _P2M1	=	0x0095
                           000096   119 _P2M0	=	0x0096
                           0000B1   120 _P3M1	=	0x00b1
                           0000B2   121 _P3M0	=	0x00b2
                           0000BA   122 _P4M1	=	0x00ba
                           0000BB   123 _P4M0	=	0x00bb
                                    124 ;--------------------------------------------------------
                                    125 ; special function bits
                                    126 ;--------------------------------------------------------
                                    127 	.area RSEG    (ABS,DATA)
      000000                        128 	.org 0x0000
                           000080   129 _P00	=	0x0080
                           000081   130 _P01	=	0x0081
                           000082   131 _P02	=	0x0082
                           000083   132 _P03	=	0x0083
                           000084   133 _P04	=	0x0084
                           000085   134 _P05	=	0x0085
                           000086   135 _P06	=	0x0086
                           000087   136 _P07	=	0x0087
                           000090   137 _P10	=	0x0090
                           000091   138 _P11	=	0x0091
                           000092   139 _P12	=	0x0092
                           000093   140 _P13	=	0x0093
                           000094   141 _P14	=	0x0094
                           000095   142 _P15	=	0x0095
                           000096   143 _P16	=	0x0096
                           000097   144 _P17	=	0x0097
                           0000A0   145 _P20	=	0x00a0
                           0000A1   146 _P21	=	0x00a1
                           0000A2   147 _P22	=	0x00a2
                           0000A3   148 _P23	=	0x00a3
                           0000A4   149 _P24	=	0x00a4
                           0000A5   150 _P25	=	0x00a5
                           0000A6   151 _P26	=	0x00a6
                           0000A7   152 _P27	=	0x00a7
                           0000B0   153 _P30	=	0x00b0
                           0000B1   154 _P31	=	0x00b1
                           0000B2   155 _P32	=	0x00b2
                           0000B3   156 _P33	=	0x00b3
                           0000B4   157 _P34	=	0x00b4
                           0000B5   158 _P35	=	0x00b5
                           0000B6   159 _P36	=	0x00b6
                           0000B7   160 _P37	=	0x00b7
                                    161 ;--------------------------------------------------------
                                    162 ; overlayable register banks
                                    163 ;--------------------------------------------------------
                                    164 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                        165 	.ds 8
                                    166 ;--------------------------------------------------------
                                    167 ; internal ram data
                                    168 ;--------------------------------------------------------
                                    169 	.area DSEG    (DATA)
      000038                        170 _g_rx:
      000038                        171 	.ds 24
      000050                        172 _g_rx_len:
      000050                        173 	.ds 1
      000051                        174 _g_rx_overflow:
      000051                        175 	.ds 1
      000052                        176 _SensorModbus_ReplyPressure_PARM_2:
      000052                        177 	.ds 2
      000054                        178 _SensorModbus_ReplyPressure_tx_10000_23:
      000054                        179 	.ds 8
      00005C                        180 _SensorModbus_HandleFrame_PARM_2:
      00005C                        181 	.ds 2
      00005E                        182 _SensorModbus_Process_PARM_2:
      00005E                        183 	.ds 2
      000060                        184 _SensorModbus_Process_b_10000_37:
      000060                        185 	.ds 1
                                    186 ;--------------------------------------------------------
                                    187 ; overlayable items in internal ram
                                    188 ;--------------------------------------------------------
                                    189 ;--------------------------------------------------------
                                    190 ; indirectly addressable internal ram data
                                    191 ;--------------------------------------------------------
                                    192 	.area ISEG    (DATA)
                                    193 ;--------------------------------------------------------
                                    194 ; absolute internal ram data
                                    195 ;--------------------------------------------------------
                                    196 	.area IABS    (ABS,DATA)
                                    197 	.area IABS    (ABS,DATA)
                                    198 ;--------------------------------------------------------
                                    199 ; bit data
                                    200 ;--------------------------------------------------------
                                    201 	.area BSEG    (BIT)
                                    202 ;--------------------------------------------------------
                                    203 ; paged external ram data
                                    204 ;--------------------------------------------------------
                                    205 	.area PSEG    (PAG,XDATA)
                                    206 ;--------------------------------------------------------
                                    207 ; uninitialized external ram data
                                    208 ;--------------------------------------------------------
                                    209 	.area XSEG    (XDATA)
                                    210 ;--------------------------------------------------------
                                    211 ; absolute external ram data
                                    212 ;--------------------------------------------------------
                                    213 	.area XABS    (ABS,XDATA)
                                    214 ;--------------------------------------------------------
                                    215 ; initialized external ram data
                                    216 ;--------------------------------------------------------
                                    217 	.area XISEG   (XDATA)
                                    218 	.area HOME    (CODE)
                                    219 	.area GSINIT0 (CODE)
                                    220 	.area GSINIT1 (CODE)
                                    221 	.area GSINIT2 (CODE)
                                    222 	.area GSINIT3 (CODE)
                                    223 	.area GSINIT4 (CODE)
                                    224 	.area GSINIT5 (CODE)
                                    225 	.area GSINIT  (CODE)
                                    226 	.area GSFINAL (CODE)
                                    227 	.area CSEG    (CODE)
                                    228 ;--------------------------------------------------------
                                    229 ; global & static initialisations
                                    230 ;--------------------------------------------------------
                                    231 	.area HOME    (CODE)
                                    232 	.area GSINIT  (CODE)
                                    233 	.area GSFINAL (CODE)
                                    234 	.area GSINIT  (CODE)
                                    235 ;--------------------------------------------------------
                                    236 ; Home
                                    237 ;--------------------------------------------------------
                                    238 	.area HOME    (CODE)
                                    239 	.area HOME    (CODE)
                                    240 ;--------------------------------------------------------
                                    241 ; code
                                    242 ;--------------------------------------------------------
                                    243 	.area CSEG    (CODE)
                                    244 ;------------------------------------------------------------
                                    245 ;Allocation info for local variables in function 'SensorModbus_Init'
                                    246 ;------------------------------------------------------------
                                    247 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:40: void SensorModbus_Init(void)
                                    248 ;	-----------------------------------------
                                    249 ;	 function SensorModbus_Init
                                    250 ;	-----------------------------------------
      0005FF                        251 _SensorModbus_Init:
                           000007   252 	ar7 = 0x07
                           000006   253 	ar6 = 0x06
                           000005   254 	ar5 = 0x05
                           000004   255 	ar4 = 0x04
                           000003   256 	ar3 = 0x03
                           000002   257 	ar2 = 0x02
                           000001   258 	ar1 = 0x01
                           000000   259 	ar0 = 0x00
                                    260 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:42: g_rx_len = 0U;
                                    261 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:43: g_rx_overflow = 0U;
                                    262 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:46: TMOD = (TMOD & 0xF0U) | 0x01U;
      0005FF E4               [12]  263 	clr	a
      000600 F5 50            [12]  264 	mov	_g_rx_len,a
      000602 F5 51            [12]  265 	mov	_g_rx_overflow,a
      000604 E5 89            [12]  266 	mov	a,_TMOD
      000606 54 F0            [12]  267 	anl	a,#0xf0
      000608 44 01            [12]  268 	orl	a,#0x01
      00060A F5 89            [12]  269 	mov	_TMOD,a
                                    270 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:47: AUXR &= (uint8_t)~0x80U;            /* T0x12 = 0：12T 模式 */
      00060C 53 8E 7F         [24]  271 	anl	_AUXR,#0x7f
                                    272 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:48: TCON &= (uint8_t)~(TCON_TR0 | TCON_TF0);
      00060F 53 88 CF         [24]  273 	anl	_TCON,#0xcf
                                    274 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:49: }
      000612 22               [24]  275 	ret
                                    276 ;------------------------------------------------------------
                                    277 ;Allocation info for local variables in function 'SensorModbus_RestartGapTimer'
                                    278 ;------------------------------------------------------------
                                    279 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:52: static void SensorModbus_RestartGapTimer(void)
                                    280 ;	-----------------------------------------
                                    281 ;	 function SensorModbus_RestartGapTimer
                                    282 ;	-----------------------------------------
      000613                        283 _SensorModbus_RestartGapTimer:
                                    284 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:54: TCON &= (uint8_t)~(TCON_TR0 | TCON_TF0);
      000613 53 88 CF         [24]  285 	anl	_TCON,#0xcf
                                    286 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:55: TH0 = MB_GAP_RELOAD_H;
      000616 75 8C F1         [24]  287 	mov	_TH0,#0xf1
                                    288 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:56: TL0 = MB_GAP_RELOAD_L;
      000619 75 8A 9A         [24]  289 	mov	_TL0,#0x9a
                                    290 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:57: TCON |= TCON_TR0;
      00061C 43 88 10         [24]  291 	orl	_TCON,#0x10
                                    292 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:58: }
      00061F 22               [24]  293 	ret
                                    294 ;------------------------------------------------------------
                                    295 ;Allocation info for local variables in function 'SensorModbus_ReplyPressure'
                                    296 ;------------------------------------------------------------
                                    297 ;pressure      Allocated with name '_SensorModbus_ReplyPressure_PARM_2'
                                    298 ;addr          Allocated to registers r7 
                                    299 ;tx            Allocated with name '_SensorModbus_ReplyPressure_tx_10000_23'
                                    300 ;------------------------------------------------------------
                                    301 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:60: static void SensorModbus_ReplyPressure(uint8_t addr, uint16_t pressure)
                                    302 ;	-----------------------------------------
                                    303 ;	 function SensorModbus_ReplyPressure
                                    304 ;	-----------------------------------------
      000620                        305 _SensorModbus_ReplyPressure:
      000620 AF 82            [24]  306 	mov	r7, dpl
                                    307 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:64: tx[0] = addr;
      000622 8F 54            [24]  308 	mov	_SensorModbus_ReplyPressure_tx_10000_23,r7
                                    309 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:65: tx[1] = 0x03U;
      000624 75 55 03         [24]  310 	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0001),#0x03
                                    311 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:66: tx[2] = 0x03U;                       /* 字节数：型号 + 2 字节数据 */
      000627 75 56 03         [24]  312 	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0002),#0x03
                                    313 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:67: tx[3] = SENSOR_TYPE_WIND_PRESSURE;   /* 0x02 */
      00062A 75 57 02         [24]  314 	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0003),#0x02
                                    315 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:68: tx[4] = (uint8_t)(pressure >> 8);
      00062D AF 53            [24]  316 	mov	r7,(_SensorModbus_ReplyPressure_PARM_2 + 1)
      00062F 8F 58            [24]  317 	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0004),r7
                                    318 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:69: tx[5] = (uint8_t)pressure;
      000631 AF 52            [24]  319 	mov	r7,_SensorModbus_ReplyPressure_PARM_2
      000633 8F 59            [24]  320 	mov	(_SensorModbus_ReplyPressure_tx_10000_23 + 0x0005),r7
                                    321 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:70: Crc16_Append(tx, 6U);
      000635 75 0B 06         [24]  322 	mov	_Crc16_Append_PARM_2,#0x06
      000638 90 00 54         [24]  323 	mov	dptr,#_SensorModbus_ReplyPressure_tx_10000_23
      00063B 75 F0 40         [24]  324 	mov	b, #0x40
      00063E 12 02 9F         [24]  325 	lcall	_Crc16_Append
                                    326 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:71: Uart2_Send(tx, sizeof(tx));
      000641 75 14 08         [24]  327 	mov	_Uart2_Send_PARM_2,#0x08
      000644 90 00 54         [24]  328 	mov	dptr,#_SensorModbus_ReplyPressure_tx_10000_23
      000647 75 F0 40         [24]  329 	mov	b, #0x40
                                    330 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:72: }
      00064A 02 07 F7         [24]  331 	ljmp	_Uart2_Send
                                    332 ;------------------------------------------------------------
                                    333 ;Allocation info for local variables in function 'SensorModbus_HandleFrame'
                                    334 ;------------------------------------------------------------
                                    335 ;pressure      Allocated with name '_SensorModbus_HandleFrame_PARM_2'
                                    336 ;my_addr       Allocated to registers r7 
                                    337 ;------------------------------------------------------------
                                    338 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:74: static void SensorModbus_HandleFrame(uint8_t my_addr, uint16_t pressure)
                                    339 ;	-----------------------------------------
                                    340 ;	 function SensorModbus_HandleFrame
                                    341 ;	-----------------------------------------
      00064D                        342 _SensorModbus_HandleFrame:
      00064D AF 82            [24]  343 	mov	r7, dpl
                                    344 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:77: if (g_rx_len != 8U) {
      00064F 74 08            [12]  345 	mov	a,#0x08
      000651 B5 50 02         [24]  346 	cjne	a,_g_rx_len,00214$
      000654 80 01            [24]  347 	sjmp	00102$
      000656                        348 00214$:
                                    349 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:78: return;
      000656 22               [24]  350 	ret
      000657                        351 00102$:
                                    352 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:81: if (g_rx[0] != my_addr) {
      000657 EF               [12]  353 	mov	a,r7
      000658 B5 38 02         [24]  354 	cjne	a,_g_rx,00215$
      00065B 80 01            [24]  355 	sjmp	00104$
      00065D                        356 00215$:
                                    357 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:82: return;
      00065D 22               [24]  358 	ret
      00065E                        359 00104$:
                                    360 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:85: if (Crc16_Check(g_rx, 8U) == 0U) {
      00065E 75 08 08         [24]  361 	mov	_Crc16_Check_PARM_2,#0x08
      000661 90 00 38         [24]  362 	mov	dptr,#_g_rx
      000664 75 F0 40         [24]  363 	mov	b, #0x40
      000667 C0 07            [24]  364 	push	ar7
      000669 12 02 26         [24]  365 	lcall	_Crc16_Check
      00066C E5 82            [12]  366 	mov	a, dpl
      00066E D0 07            [24]  367 	pop	ar7
      000670 70 01            [24]  368 	jnz	00106$
                                    369 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:86: return;
      000672 22               [24]  370 	ret
      000673                        371 00106$:
                                    372 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:96: if ((g_rx[1] == 0x03U) &&
      000673 AE 39            [24]  373 	mov	r6,(_g_rx + 0x0001)
      000675 BE 03 20         [24]  374 	cjne	r6,#0x03,00108$
                                    375 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:97: (g_rx[2] == 0x00U) &&
      000678 E5 3A            [12]  376 	mov	a,(_g_rx + 0x0002)
      00067A 70 1C            [24]  377 	jnz	00108$
                                    378 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:98: ((g_rx[3] == 0x00U) || (g_rx[3] == 0x01U)) &&
      00067C E5 3B            [12]  379 	mov	a,(_g_rx + 0x0003)
      00067E FD               [12]  380 	mov	r5,a
      00067F 60 03            [24]  381 	jz	00112$
      000681 BD 01 14         [24]  382 	cjne	r5,#0x01,00108$
      000684                        383 00112$:
                                    384 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:99: (g_rx[4] == 0x00U) && (g_rx[5] == 0x01U)) {
      000684 E5 3C            [12]  385 	mov	a,(_g_rx + 0x0004)
      000686 70 10            [24]  386 	jnz	00108$
      000688 74 01            [12]  387 	mov	a,#0x01
      00068A B5 3D 0B         [24]  388 	cjne	a,(_g_rx + 0x0005),00108$
                                    389 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:100: SensorModbus_ReplyPressure(my_addr, pressure);
      00068D 85 5C 52         [24]  390 	mov	_SensorModbus_ReplyPressure_PARM_2,_SensorModbus_HandleFrame_PARM_2
      000690 85 5D 53         [24]  391 	mov	(_SensorModbus_ReplyPressure_PARM_2 + 1),(_SensorModbus_HandleFrame_PARM_2 + 1)
      000693 8F 82            [24]  392 	mov	dpl, r7
                                    393 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:101: return;
      000695 02 06 20         [24]  394 	ljmp	_SensorModbus_ReplyPressure
      000698                        395 00108$:
                                    396 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:105: if ((g_rx[1] == 0x06U) &&
      000698 BE 06 19         [24]  397 	cjne	r6,#0x06,00119$
                                    398 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:106: (g_rx[2] == 0x00U) && (g_rx[3] == 0x04U) &&
      00069B E5 3A            [12]  399 	mov	a,(_g_rx + 0x0002)
      00069D 70 15            [24]  400 	jnz	00119$
      00069F 74 04            [12]  401 	mov	a,#0x04
      0006A1 B5 3B 10         [24]  402 	cjne	a,(_g_rx + 0x0003),00119$
                                    403 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:107: (g_rx[4] == 0x00U)) {
      0006A4 E5 3C            [12]  404 	mov	a,(_g_rx + 0x0004)
      0006A6 70 0C            [24]  405 	jnz	00119$
                                    406 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:108: Board_RedLedSet((g_rx[5] != 0U) ? 1U : 0U);
      0006A8 E5 3D            [12]  407 	mov	a,(_g_rx + 0x0005)
      0006AA 60 02            [24]  408 	jz	00121$
      0006AC 74 01            [12]  409 	mov	a,#0x01
      0006AE                        410 00121$:
      0006AE FF               [12]  411 	mov	r7,a
      0006AF 8F 82            [24]  412 	mov	dpl,r7
                                    413 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:110: }
      0006B1 02 01 89         [24]  414 	ljmp	_Board_RedLedSet
      0006B4                        415 00119$:
      0006B4 22               [24]  416 	ret
                                    417 ;------------------------------------------------------------
                                    418 ;Allocation info for local variables in function 'SensorModbus_Process'
                                    419 ;------------------------------------------------------------
                                    420 ;pressure      Allocated with name '_SensorModbus_Process_PARM_2'
                                    421 ;my_addr       Allocated to registers r7 
                                    422 ;b             Allocated with name '_SensorModbus_Process_b_10000_37'
                                    423 ;------------------------------------------------------------
                                    424 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:112: void SensorModbus_Process(uint8_t my_addr, uint16_t pressure)
                                    425 ;	-----------------------------------------
                                    426 ;	 function SensorModbus_Process
                                    427 ;	-----------------------------------------
      0006B5                        428 _SensorModbus_Process:
      0006B5 AF 82            [24]  429 	mov	r7, dpl
                                    430 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:117: while (Uart2_ReadByte(&b) != 0U) {
      0006B7                        431 00104$:
      0006B7 90 00 60         [24]  432 	mov	dptr,#_SensorModbus_Process_b_10000_37
      0006BA 75 F0 40         [24]  433 	mov	b, #0x40
      0006BD C0 07            [24]  434 	push	ar7
      0006BF 12 07 9E         [24]  435 	lcall	_Uart2_ReadByte
      0006C2 E5 82            [12]  436 	mov	a, dpl
      0006C4 D0 07            [24]  437 	pop	ar7
      0006C6 60 1E            [24]  438 	jz	00106$
                                    439 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:118: if (g_rx_len < MB_RX_BUF_SIZE) {
      0006C8 74 E8            [12]  440 	mov	a,#0x100 - 0x18
      0006CA 25 50            [12]  441 	add	a,_g_rx_len
      0006CC 40 0C            [24]  442 	jc	00102$
                                    443 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:119: g_rx[g_rx_len++] = b;
      0006CE AE 50            [24]  444 	mov	r6,_g_rx_len
      0006D0 05 50            [12]  445 	inc	_g_rx_len
      0006D2 EE               [12]  446 	mov	a,r6
      0006D3 24 38            [12]  447 	add	a, #_g_rx
      0006D5 F8               [12]  448 	mov	r0,a
      0006D6 A6 60            [24]  449 	mov	@r0,_SensorModbus_Process_b_10000_37
      0006D8 80 03            [24]  450 	sjmp	00103$
      0006DA                        451 00102$:
                                    452 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:121: g_rx_overflow = 1U;          /* 超长帧：整帧作废 */
      0006DA 75 51 01         [24]  453 	mov	_g_rx_overflow,#0x01
      0006DD                        454 00103$:
                                    455 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:123: SensorModbus_RestartGapTimer();
      0006DD C0 07            [24]  456 	push	ar7
      0006DF 12 06 13         [24]  457 	lcall	_SensorModbus_RestartGapTimer
      0006E2 D0 07            [24]  458 	pop	ar7
      0006E4 80 D1            [24]  459 	sjmp	00104$
      0006E6                        460 00106$:
                                    461 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:127: if ((g_rx_len != 0U) && ((TCON & TCON_TF0) != 0U)) {
      0006E6 E5 50            [12]  462 	mov	a,_g_rx_len
      0006E8 60 1D            [24]  463 	jz	00112$
      0006EA E5 88            [12]  464 	mov	a,_TCON
      0006EC 30 E5 18         [24]  465 	jnb	acc.5,00112$
                                    466 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:128: TCON &= (uint8_t)~(TCON_TR0 | TCON_TF0);
      0006EF 53 88 CF         [24]  467 	anl	_TCON,#0xcf
                                    468 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:129: if (g_rx_overflow == 0U) {
      0006F2 E5 51            [12]  469 	mov	a,_g_rx_overflow
      0006F4 70 0B            [24]  470 	jnz	00108$
                                    471 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:130: SensorModbus_HandleFrame(my_addr, pressure);
      0006F6 85 5E 5C         [24]  472 	mov	_SensorModbus_HandleFrame_PARM_2,_SensorModbus_Process_PARM_2
      0006F9 85 5F 5D         [24]  473 	mov	(_SensorModbus_HandleFrame_PARM_2 + 1),(_SensorModbus_Process_PARM_2 + 1)
      0006FC 8F 82            [24]  474 	mov	dpl, r7
      0006FE 12 06 4D         [24]  475 	lcall	_SensorModbus_HandleFrame
      000701                        476 00108$:
                                    477 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:132: g_rx_len = 0U;
      000701 75 50 00         [24]  478 	mov	_g_rx_len,#0x00
                                    479 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:133: g_rx_overflow = 0U;
      000704 75 51 00         [24]  480 	mov	_g_rx_overflow,#0x00
      000707                        481 00112$:
                                    482 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/sensor_modbus.c:135: }
      000707 22               [24]  483 	ret
                                    484 	.area CSEG    (CODE)
                                    485 	.area CONST   (CODE)
                                    486 	.area XINIT   (CODE)
                                    487 	.area CABS    (ABS,CODE)
