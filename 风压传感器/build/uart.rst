                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler
                                      3 ; Version 4.6.0 #16555 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module uart
                                      6 	
                                      7 	.optsdcc -mmcs51 --model-small
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _P37
                                     12 	.globl _P36
                                     13 	.globl _P35
                                     14 	.globl _P34
                                     15 	.globl _P33
                                     16 	.globl _P32
                                     17 	.globl _P31
                                     18 	.globl _P30
                                     19 	.globl _P27
                                     20 	.globl _P26
                                     21 	.globl _P25
                                     22 	.globl _P24
                                     23 	.globl _P23
                                     24 	.globl _P22
                                     25 	.globl _P21
                                     26 	.globl _P20
                                     27 	.globl _P17
                                     28 	.globl _P16
                                     29 	.globl _P15
                                     30 	.globl _P14
                                     31 	.globl _P13
                                     32 	.globl _P12
                                     33 	.globl _P11
                                     34 	.globl _P10
                                     35 	.globl _P07
                                     36 	.globl _P06
                                     37 	.globl _P05
                                     38 	.globl _P04
                                     39 	.globl _P03
                                     40 	.globl _P02
                                     41 	.globl _P01
                                     42 	.globl _P00
                                     43 	.globl _P4M0
                                     44 	.globl _P4M1
                                     45 	.globl _P3M0
                                     46 	.globl _P3M1
                                     47 	.globl _P2M0
                                     48 	.globl _P2M1
                                     49 	.globl _P0M0
                                     50 	.globl _P0M1
                                     51 	.globl _P1M0
                                     52 	.globl _P1M1
                                     53 	.globl _T2L
                                     54 	.globl _T2H
                                     55 	.globl _IP
                                     56 	.globl _IE
                                     57 	.globl _P_SW1
                                     58 	.globl _S2BUF
                                     59 	.globl _S2CON
                                     60 	.globl _SBUF
                                     61 	.globl _SCON
                                     62 	.globl _AUXR
                                     63 	.globl _TH1
                                     64 	.globl _TH0
                                     65 	.globl _TL1
                                     66 	.globl _TL0
                                     67 	.globl _TMOD
                                     68 	.globl _TCON
                                     69 	.globl _PCON
                                     70 	.globl _DPH
                                     71 	.globl _DPL
                                     72 	.globl _SP
                                     73 	.globl _P3
                                     74 	.globl _P2
                                     75 	.globl _P1
                                     76 	.globl _P0
                                     77 	.globl _Uart2_Send_PARM_2
                                     78 	.globl _Uart1_Send_PARM_2
                                     79 	.globl _Uart_Init
                                     80 	.globl _Uart1_SendByte
                                     81 	.globl _Uart1_ReadByte
                                     82 	.globl _Uart2_SendByte
                                     83 	.globl _Uart2_ReadByte
                                     84 	.globl _Uart1_Send
                                     85 	.globl _Uart2_Send
                                     86 ;--------------------------------------------------------
                                     87 ; special function registers
                                     88 ;--------------------------------------------------------
                                     89 	.area RSEG    (ABS,DATA)
      000000                         90 	.org 0x0000
                           000080    91 _P0	=	0x0080
                           000090    92 _P1	=	0x0090
                           0000A0    93 _P2	=	0x00a0
                           0000B0    94 _P3	=	0x00b0
                           000081    95 _SP	=	0x0081
                           000082    96 _DPL	=	0x0082
                           000083    97 _DPH	=	0x0083
                           000087    98 _PCON	=	0x0087
                           000088    99 _TCON	=	0x0088
                           000089   100 _TMOD	=	0x0089
                           00008A   101 _TL0	=	0x008a
                           00008B   102 _TL1	=	0x008b
                           00008C   103 _TH0	=	0x008c
                           00008D   104 _TH1	=	0x008d
                           00008E   105 _AUXR	=	0x008e
                           000098   106 _SCON	=	0x0098
                           000099   107 _SBUF	=	0x0099
                           00009A   108 _S2CON	=	0x009a
                           00009B   109 _S2BUF	=	0x009b
                           0000A2   110 _P_SW1	=	0x00a2
                           0000A8   111 _IE	=	0x00a8
                           0000B8   112 _IP	=	0x00b8
                           0000D6   113 _T2H	=	0x00d6
                           0000D7   114 _T2L	=	0x00d7
                           000091   115 _P1M1	=	0x0091
                           000092   116 _P1M0	=	0x0092
                           000093   117 _P0M1	=	0x0093
                           000094   118 _P0M0	=	0x0094
                           000095   119 _P2M1	=	0x0095
                           000096   120 _P2M0	=	0x0096
                           0000B1   121 _P3M1	=	0x00b1
                           0000B2   122 _P3M0	=	0x00b2
                           0000BA   123 _P4M1	=	0x00ba
                           0000BB   124 _P4M0	=	0x00bb
                                    125 ;--------------------------------------------------------
                                    126 ; special function bits
                                    127 ;--------------------------------------------------------
                                    128 	.area RSEG    (ABS,DATA)
      000000                        129 	.org 0x0000
                           000080   130 _P00	=	0x0080
                           000081   131 _P01	=	0x0081
                           000082   132 _P02	=	0x0082
                           000083   133 _P03	=	0x0083
                           000084   134 _P04	=	0x0084
                           000085   135 _P05	=	0x0085
                           000086   136 _P06	=	0x0086
                           000087   137 _P07	=	0x0087
                           000090   138 _P10	=	0x0090
                           000091   139 _P11	=	0x0091
                           000092   140 _P12	=	0x0092
                           000093   141 _P13	=	0x0093
                           000094   142 _P14	=	0x0094
                           000095   143 _P15	=	0x0095
                           000096   144 _P16	=	0x0096
                           000097   145 _P17	=	0x0097
                           0000A0   146 _P20	=	0x00a0
                           0000A1   147 _P21	=	0x00a1
                           0000A2   148 _P22	=	0x00a2
                           0000A3   149 _P23	=	0x00a3
                           0000A4   150 _P24	=	0x00a4
                           0000A5   151 _P25	=	0x00a5
                           0000A6   152 _P26	=	0x00a6
                           0000A7   153 _P27	=	0x00a7
                           0000B0   154 _P30	=	0x00b0
                           0000B1   155 _P31	=	0x00b1
                           0000B2   156 _P32	=	0x00b2
                           0000B3   157 _P33	=	0x00b3
                           0000B4   158 _P34	=	0x00b4
                           0000B5   159 _P35	=	0x00b5
                           0000B6   160 _P36	=	0x00b6
                           0000B7   161 _P37	=	0x00b7
                                    162 ;--------------------------------------------------------
                                    163 ; overlayable register banks
                                    164 ;--------------------------------------------------------
                                    165 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                        166 	.ds 8
                                    167 ;--------------------------------------------------------
                                    168 ; internal ram data
                                    169 ;--------------------------------------------------------
                                    170 	.area DSEG    (DATA)
      000013                        171 _Uart1_Send_PARM_2:
      000013                        172 	.ds 1
      000014                        173 _Uart2_Send_PARM_2:
      000014                        174 	.ds 1
                                    175 ;--------------------------------------------------------
                                    176 ; overlayable items in internal ram
                                    177 ;--------------------------------------------------------
                                    178 	.area	OSEG    (OVR,DATA)
                                    179 	.area	OSEG    (OVR,DATA)
                                    180 	.area	OSEG    (OVR,DATA)
                                    181 	.area	OSEG    (OVR,DATA)
                                    182 ;--------------------------------------------------------
                                    183 ; indirectly addressable internal ram data
                                    184 ;--------------------------------------------------------
                                    185 	.area ISEG    (DATA)
                                    186 ;--------------------------------------------------------
                                    187 ; absolute internal ram data
                                    188 ;--------------------------------------------------------
                                    189 	.area IABS    (ABS,DATA)
                                    190 	.area IABS    (ABS,DATA)
                                    191 ;--------------------------------------------------------
                                    192 ; bit data
                                    193 ;--------------------------------------------------------
                                    194 	.area BSEG    (BIT)
                                    195 ;--------------------------------------------------------
                                    196 ; paged external ram data
                                    197 ;--------------------------------------------------------
                                    198 	.area PSEG    (PAG,XDATA)
                                    199 ;--------------------------------------------------------
                                    200 ; uninitialized external ram data
                                    201 ;--------------------------------------------------------
                                    202 	.area XSEG    (XDATA)
                                    203 ;--------------------------------------------------------
                                    204 ; absolute external ram data
                                    205 ;--------------------------------------------------------
                                    206 	.area XABS    (ABS,XDATA)
                                    207 ;--------------------------------------------------------
                                    208 ; initialized external ram data
                                    209 ;--------------------------------------------------------
                                    210 	.area XISEG   (XDATA)
                                    211 	.area HOME    (CODE)
                                    212 	.area GSINIT0 (CODE)
                                    213 	.area GSINIT1 (CODE)
                                    214 	.area GSINIT2 (CODE)
                                    215 	.area GSINIT3 (CODE)
                                    216 	.area GSINIT4 (CODE)
                                    217 	.area GSINIT5 (CODE)
                                    218 	.area GSINIT  (CODE)
                                    219 	.area GSFINAL (CODE)
                                    220 	.area CSEG    (CODE)
                                    221 ;--------------------------------------------------------
                                    222 ; global & static initialisations
                                    223 ;--------------------------------------------------------
                                    224 	.area HOME    (CODE)
                                    225 	.area GSINIT  (CODE)
                                    226 	.area GSFINAL (CODE)
                                    227 	.area GSINIT  (CODE)
                                    228 ;--------------------------------------------------------
                                    229 ; Home
                                    230 ;--------------------------------------------------------
                                    231 	.area HOME    (CODE)
                                    232 	.area HOME    (CODE)
                                    233 ;--------------------------------------------------------
                                    234 ; code
                                    235 ;--------------------------------------------------------
                                    236 	.area CSEG    (CODE)
                                    237 ;------------------------------------------------------------
                                    238 ;Allocation info for local variables in function 'Uart_Reload'
                                    239 ;------------------------------------------------------------
                                    240 ;baud          Allocated to registers 
                                    241 ;------------------------------------------------------------
                                    242 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:10: static uint16_t Uart_Reload(uint32_t baud)
                                    243 ;	-----------------------------------------
                                    244 ;	 function Uart_Reload
                                    245 ;	-----------------------------------------
      000708                        246 _Uart_Reload:
                           000007   247 	ar7 = 0x07
                           000006   248 	ar6 = 0x06
                           000005   249 	ar5 = 0x05
                           000004   250 	ar4 = 0x04
                           000003   251 	ar3 = 0x03
                           000002   252 	ar2 = 0x02
                           000001   253 	ar1 = 0x01
                           000000   254 	ar0 = 0x00
      000708 85 82 15         [24]  255 	mov	__divulong_PARM_2,dpl
      00070B 85 83 16         [24]  256 	mov	(__divulong_PARM_2 + 1),dph
      00070E 85 F0 17         [24]  257 	mov	(__divulong_PARM_2 + 2),b
      000711 F5 18            [12]  258 	mov	(__divulong_PARM_2 + 3),a
                                    259 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:12: return (uint16_t)(65536UL - (FOSC_HZ / 4UL / baud));
      000713 90 30 00         [24]  260 	mov	dptr,#0x3000
      000716 75 F0 2A         [24]  261 	mov	b, #0x2a
      000719 E4               [12]  262 	clr	a
      00071A 12 08 58         [24]  263 	lcall	__divulong
      00071D AC 82            [24]  264 	mov	r4, dpl
      00071F AD 83            [24]  265 	mov	r5, dph
      000721 C3               [12]  266 	clr	c
      000722 E4               [12]  267 	clr	a
      000723 9C               [12]  268 	subb	a,r4
      000724 F5 82            [12]  269 	mov	dpl,a
      000726 E4               [12]  270 	clr	a
      000727 9D               [12]  271 	subb	a,r5
      000728 F5 83            [12]  272 	mov	dph,a
                                    273 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:13: }
      00072A 22               [24]  274 	ret
                                    275 ;------------------------------------------------------------
                                    276 ;Allocation info for local variables in function 'Uart_Init'
                                    277 ;------------------------------------------------------------
                                    278 ;reload2       Allocated to registers r6 r7 
                                    279 ;------------------------------------------------------------
                                    280 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:15: void Uart_Init(void)
                                    281 ;	-----------------------------------------
                                    282 ;	 function Uart_Init
                                    283 ;	-----------------------------------------
      00072B                        284 _Uart_Init:
                                    285 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:31: uint16_t reload2 = Uart_Reload(CONTROLLER_BAUD);  // 9600 重装值
      00072B 90 25 80         [24]  286 	mov	dptr,#0x2580
      00072E E4               [12]  287 	clr	a
      00072F F5 F0            [12]  288 	mov	b,a
      000731 12 07 08         [24]  289 	lcall	_Uart_Reload
      000734 AE 82            [24]  290 	mov	r6, dpl
      000736 AF 83            [24]  291 	mov	r7, dph
                                    292 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:33: P_SW1 &= 0x3FU; /* UART1 on P30/RXD and P31/TXD. */
      000738 53 A2 3F         [24]  293 	anl	_P_SW1,#0x3f
                                    294 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:34: SCON = SCON_MODE1 | SCON_REN;
      00073B 75 98 50         [24]  295 	mov	_SCON,#0x50
                                    296 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:35: S2CON = S2CON_REN;
      00073E 75 9A 10         [24]  297 	mov	_S2CON,#0x10
                                    298 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:38: TMOD = (TMOD & 0x0FU) | 0x20U;            // Timer 1, 8-bit auto-reload (mode 2)
      000741 E5 89            [12]  299 	mov	a,_TMOD
      000743 54 0F            [12]  300 	anl	a,#0x0f
      000745 44 20            [12]  301 	orl	a,#0x20
      000747 F5 89            [12]  302 	mov	_TMOD,a
                                    303 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:39: TH1 = (uint8_t)(256U - (FOSC_HZ / 32U / PRESSURE_BAUD));
      000749 75 8D FD         [24]  304 	mov	_TH1,#0xfd
                                    305 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:41: TL1 = TH1;
      00074C 85 8D 8B         [24]  306 	mov	_TL1,_TH1
                                    307 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:42: TCON |= 0x40U;                             // TR1 = 1：启动 Timer 1
      00074F 43 88 40         [24]  308 	orl	_TCON,#0x40
                                    309 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:45: T2H = (uint8_t)(reload2 >> 8);
      000752 8F D6            [24]  310 	mov	_T2H,r7
                                    311 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:46: T2L = (uint8_t)reload2;
      000754 8E D7            [24]  312 	mov	_T2L,r6
                                    313 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:48: AUXR = (AUXR & (uint8_t)~0x01U) | 0x54U;
      000756 74 FE            [12]  314 	mov	a,#0xfe
      000758 55 8E            [12]  315 	anl	a,_AUXR
      00075A 44 54            [12]  316 	orl	a,#0x54
      00075C F5 8E            [12]  317 	mov	_AUXR,a
                                    318 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:50: SCON &= (uint8_t)~(SCON_RI | SCON_TI);
      00075E 53 98 FC         [24]  319 	anl	_SCON,#0xfc
                                    320 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:51: S2CON &= (uint8_t)~(S2CON_RI | S2CON_TI);
      000761 53 9A FC         [24]  321 	anl	_S2CON,#0xfc
                                    322 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:52: }
      000764 22               [24]  323 	ret
                                    324 ;------------------------------------------------------------
                                    325 ;Allocation info for local variables in function 'Uart1_SendByte'
                                    326 ;------------------------------------------------------------
                                    327 ;value         Allocated to registers 
                                    328 ;------------------------------------------------------------
                                    329 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:54: void Uart1_SendByte(uint8_t value)
                                    330 ;	-----------------------------------------
                                    331 ;	 function Uart1_SendByte
                                    332 ;	-----------------------------------------
      000765                        333 _Uart1_SendByte:
      000765 85 82 99         [24]  334 	mov	_SBUF,dpl
                                    335 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:57: while ((SCON & SCON_TI) == 0U) {
      000768                        336 00101$:
      000768 E5 98            [12]  337 	mov	a,_SCON
      00076A 30 E1 FB         [24]  338 	jnb	acc.1,00101$
                                    339 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:59: SCON &= (uint8_t)~SCON_TI;
      00076D 53 98 FD         [24]  340 	anl	_SCON,#0xfd
                                    341 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:60: }
      000770 22               [24]  342 	ret
                                    343 ;------------------------------------------------------------
                                    344 ;Allocation info for local variables in function 'Uart1_ReadByte'
                                    345 ;------------------------------------------------------------
                                    346 ;value         Allocated to registers r5 r6 r7 
                                    347 ;------------------------------------------------------------
                                    348 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:62: uint8_t Uart1_ReadByte(uint8_t *value)
                                    349 ;	-----------------------------------------
                                    350 ;	 function Uart1_ReadByte
                                    351 ;	-----------------------------------------
      000771                        352 _Uart1_ReadByte:
      000771 AD 82            [24]  353 	mov	r5, dpl
      000773 AE 83            [24]  354 	mov	r6, dph
      000775 AF F0            [24]  355 	mov	r7, b
                                    356 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:64: if ((SCON & SCON_RI) == 0U) {
      000777 E5 98            [12]  357 	mov	a,_SCON
      000779 20 E0 04         [24]  358 	jb	acc.0,00102$
                                    359 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:65: return 0U;
      00077C 75 82 00         [24]  360 	mov	dpl, #0x00
      00077F 22               [24]  361 	ret
      000780                        362 00102$:
                                    363 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:68: *value = SBUF;
      000780 8D 82            [24]  364 	mov	dpl,r5
      000782 8E 83            [24]  365 	mov	dph,r6
      000784 8F F0            [24]  366 	mov	b,r7
      000786 E5 99            [12]  367 	mov	a,_SBUF
      000788 12 08 BD         [24]  368 	lcall	__gptrput
                                    369 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:69: SCON &= (uint8_t)~SCON_RI;
      00078B 53 98 FE         [24]  370 	anl	_SCON,#0xfe
                                    371 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:70: return 1U;
      00078E 75 82 01         [24]  372 	mov	dpl, #0x01
                                    373 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:71: }
      000791 22               [24]  374 	ret
                                    375 ;------------------------------------------------------------
                                    376 ;Allocation info for local variables in function 'Uart2_SendByte'
                                    377 ;------------------------------------------------------------
                                    378 ;value         Allocated to registers 
                                    379 ;------------------------------------------------------------
                                    380 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:73: void Uart2_SendByte(uint8_t value)
                                    381 ;	-----------------------------------------
                                    382 ;	 function Uart2_SendByte
                                    383 ;	-----------------------------------------
      000792                        384 _Uart2_SendByte:
      000792 85 82 9B         [24]  385 	mov	_S2BUF,dpl
                                    386 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:76: while ((S2CON & S2CON_TI) == 0U) {
      000795                        387 00101$:
      000795 E5 9A            [12]  388 	mov	a,_S2CON
      000797 30 E1 FB         [24]  389 	jnb	acc.1,00101$
                                    390 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:78: S2CON &= (uint8_t)~S2CON_TI;
      00079A 53 9A FD         [24]  391 	anl	_S2CON,#0xfd
                                    392 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:79: }
      00079D 22               [24]  393 	ret
                                    394 ;------------------------------------------------------------
                                    395 ;Allocation info for local variables in function 'Uart2_ReadByte'
                                    396 ;------------------------------------------------------------
                                    397 ;value         Allocated to registers r5 r6 r7 
                                    398 ;------------------------------------------------------------
                                    399 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:81: uint8_t Uart2_ReadByte(uint8_t *value)
                                    400 ;	-----------------------------------------
                                    401 ;	 function Uart2_ReadByte
                                    402 ;	-----------------------------------------
      00079E                        403 _Uart2_ReadByte:
      00079E AD 82            [24]  404 	mov	r5, dpl
      0007A0 AE 83            [24]  405 	mov	r6, dph
      0007A2 AF F0            [24]  406 	mov	r7, b
                                    407 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:83: if ((S2CON & S2CON_RI) == 0U) {
      0007A4 E5 9A            [12]  408 	mov	a,_S2CON
      0007A6 20 E0 04         [24]  409 	jb	acc.0,00102$
                                    410 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:84: return 0U;
      0007A9 75 82 00         [24]  411 	mov	dpl, #0x00
      0007AC 22               [24]  412 	ret
      0007AD                        413 00102$:
                                    414 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:87: S2CON &= (uint8_t)~S2CON_RI;
      0007AD 53 9A FE         [24]  415 	anl	_S2CON,#0xfe
                                    416 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:88: *value = S2BUF;
      0007B0 8D 82            [24]  417 	mov	dpl,r5
      0007B2 8E 83            [24]  418 	mov	dph,r6
      0007B4 8F F0            [24]  419 	mov	b,r7
      0007B6 E5 9B            [12]  420 	mov	a,_S2BUF
      0007B8 12 08 BD         [24]  421 	lcall	__gptrput
                                    422 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:89: return 1U;
      0007BB 75 82 01         [24]  423 	mov	dpl, #0x01
                                    424 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:90: }
      0007BE 22               [24]  425 	ret
                                    426 ;------------------------------------------------------------
                                    427 ;Allocation info for local variables in function 'Uart1_Send'
                                    428 ;------------------------------------------------------------
                                    429 ;len           Allocated with name '_Uart1_Send_PARM_2'
                                    430 ;data          Allocated to registers r5 r6 r7 
                                    431 ;i             Allocated to registers r4 
                                    432 ;------------------------------------------------------------
                                    433 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:92: void Uart1_Send(const uint8_t *data, uint8_t len)
                                    434 ;	-----------------------------------------
                                    435 ;	 function Uart1_Send
                                    436 ;	-----------------------------------------
      0007BF                        437 _Uart1_Send:
      0007BF AD 82            [24]  438 	mov	r5, dpl
      0007C1 AE 83            [24]  439 	mov	r6, dph
      0007C3 AF F0            [24]  440 	mov	r7, b
                                    441 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:95: for (i = 0U; i < len; ++i) {
      0007C5 7C 00            [12]  442 	mov	r4,#0x00
      0007C7                        443 00103$:
      0007C7 C3               [12]  444 	clr	c
      0007C8 EC               [12]  445 	mov	a,r4
      0007C9 95 13            [12]  446 	subb	a,_Uart1_Send_PARM_2
      0007CB 50 29            [24]  447 	jnc	00105$
                                    448 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:96: Uart1_SendByte(data[i]);
      0007CD EC               [12]  449 	mov	a,r4
      0007CE 2D               [12]  450 	add	a, r5
      0007CF F9               [12]  451 	mov	r1,a
      0007D0 E4               [12]  452 	clr	a
      0007D1 3E               [12]  453 	addc	a, r6
      0007D2 FA               [12]  454 	mov	r2,a
      0007D3 8F 03            [24]  455 	mov	ar3,r7
      0007D5 89 82            [24]  456 	mov	dpl,r1
      0007D7 8A 83            [24]  457 	mov	dph,r2
      0007D9 8B F0            [24]  458 	mov	b,r3
      0007DB 12 09 25         [24]  459 	lcall	__gptrget
      0007DE F5 82            [12]  460 	mov	dpl,a
      0007E0 C0 07            [24]  461 	push	ar7
      0007E2 C0 06            [24]  462 	push	ar6
      0007E4 C0 05            [24]  463 	push	ar5
      0007E6 C0 04            [24]  464 	push	ar4
      0007E8 12 07 65         [24]  465 	lcall	_Uart1_SendByte
      0007EB D0 04            [24]  466 	pop	ar4
      0007ED D0 05            [24]  467 	pop	ar5
      0007EF D0 06            [24]  468 	pop	ar6
      0007F1 D0 07            [24]  469 	pop	ar7
                                    470 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:95: for (i = 0U; i < len; ++i) {
      0007F3 0C               [12]  471 	inc	r4
      0007F4 80 D1            [24]  472 	sjmp	00103$
      0007F6                        473 00105$:
                                    474 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:98: }
      0007F6 22               [24]  475 	ret
                                    476 ;------------------------------------------------------------
                                    477 ;Allocation info for local variables in function 'Uart2_Send'
                                    478 ;------------------------------------------------------------
                                    479 ;len           Allocated with name '_Uart2_Send_PARM_2'
                                    480 ;data          Allocated to registers r5 r6 r7 
                                    481 ;i             Allocated to registers r4 
                                    482 ;------------------------------------------------------------
                                    483 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:100: void Uart2_Send(const uint8_t *data, uint8_t len)
                                    484 ;	-----------------------------------------
                                    485 ;	 function Uart2_Send
                                    486 ;	-----------------------------------------
      0007F7                        487 _Uart2_Send:
      0007F7 AD 82            [24]  488 	mov	r5, dpl
      0007F9 AE 83            [24]  489 	mov	r6, dph
      0007FB AF F0            [24]  490 	mov	r7, b
                                    491 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:103: for (i = 0U; i < len; ++i) {
      0007FD 7C 00            [12]  492 	mov	r4,#0x00
      0007FF                        493 00103$:
      0007FF C3               [12]  494 	clr	c
      000800 EC               [12]  495 	mov	a,r4
      000801 95 14            [12]  496 	subb	a,_Uart2_Send_PARM_2
      000803 50 29            [24]  497 	jnc	00105$
                                    498 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:104: Uart2_SendByte(data[i]);
      000805 EC               [12]  499 	mov	a,r4
      000806 2D               [12]  500 	add	a, r5
      000807 F9               [12]  501 	mov	r1,a
      000808 E4               [12]  502 	clr	a
      000809 3E               [12]  503 	addc	a, r6
      00080A FA               [12]  504 	mov	r2,a
      00080B 8F 03            [24]  505 	mov	ar3,r7
      00080D 89 82            [24]  506 	mov	dpl,r1
      00080F 8A 83            [24]  507 	mov	dph,r2
      000811 8B F0            [24]  508 	mov	b,r3
      000813 12 09 25         [24]  509 	lcall	__gptrget
      000816 F5 82            [12]  510 	mov	dpl,a
      000818 C0 07            [24]  511 	push	ar7
      00081A C0 06            [24]  512 	push	ar6
      00081C C0 05            [24]  513 	push	ar5
      00081E C0 04            [24]  514 	push	ar4
      000820 12 07 92         [24]  515 	lcall	_Uart2_SendByte
      000823 D0 04            [24]  516 	pop	ar4
      000825 D0 05            [24]  517 	pop	ar5
      000827 D0 06            [24]  518 	pop	ar6
      000829 D0 07            [24]  519 	pop	ar7
                                    520 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:103: for (i = 0U; i < len; ++i) {
      00082B 0C               [12]  521 	inc	r4
      00082C 80 D1            [24]  522 	sjmp	00103$
      00082E                        523 00105$:
                                    524 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/uart.c:106: }
      00082E 22               [24]  525 	ret
                                    526 	.area CSEG    (CODE)
                                    527 	.area CONST   (CODE)
                                    528 	.area XINIT   (CODE)
                                    529 	.area CABS    (ABS,CODE)
