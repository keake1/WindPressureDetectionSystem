                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler
                                      3 ; Version 4.6.0 #16555 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module pressure
                                      6 	
                                      7 	.optsdcc -mmcs51 --model-small
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _Uart1_Send
                                     12 	.globl _Uart2_ReadByte
                                     13 	.globl _Uart1_ReadByte
                                     14 	.globl _Crc16_Append
                                     15 	.globl _Crc16_Check
                                     16 	.globl _Pressure_OnByte
                                     17 	.globl _Pressure_Init
                                     18 	.globl _Pressure_GetValue
                                     19 	.globl _Pressure_ProcessRx
                                     20 	.globl _Pressure_ProcessControllerRx
                                     21 	.globl _Pressure_PollBlocking
                                     22 ;--------------------------------------------------------
                                     23 ; special function registers
                                     24 ;--------------------------------------------------------
                                     25 	.area RSEG    (ABS,DATA)
      000000                         26 	.org 0x0000
                                     27 ;--------------------------------------------------------
                                     28 ; special function bits
                                     29 ;--------------------------------------------------------
                                     30 	.area RSEG    (ABS,DATA)
      000000                         31 	.org 0x0000
                                     32 ;--------------------------------------------------------
                                     33 ; overlayable register banks
                                     34 ;--------------------------------------------------------
                                     35 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                         36 	.ds 8
                                     37 ;--------------------------------------------------------
                                     38 ; internal ram data
                                     39 ;--------------------------------------------------------
                                     40 	.area DSEG    (DATA)
      000021                         41 _g_pressure:
      000021                         42 	.ds 2
      000023                         43 _g_frame_pos:
      000023                         44 	.ds 1
      000024                         45 _g_pressure_hi:
      000024                         46 	.ds 1
      000025                         47 _g_pressure_lo:
      000025                         48 	.ds 1
      000026                         49 _Pressure_ProcessRx_b_10000_37:
      000026                         50 	.ds 1
      000027                         51 _Pressure_ProcessControllerRx_b_10000_40:
      000027                         52 	.ds 1
      000028                         53 _Pressure_PollBlocking_req_10000_43:
      000028                         54 	.ds 8
      000030                         55 _Pressure_PollBlocking_resp_10000_43:
      000030                         56 	.ds 7
      000037                         57 _Pressure_PollBlocking_b_10000_43:
      000037                         58 	.ds 1
                                     59 ;--------------------------------------------------------
                                     60 ; overlayable items in internal ram
                                     61 ;--------------------------------------------------------
                                     62 	.area	OSEG    (OVR,DATA)
                                     63 ;--------------------------------------------------------
                                     64 ; indirectly addressable internal ram data
                                     65 ;--------------------------------------------------------
                                     66 	.area ISEG    (DATA)
                                     67 ;--------------------------------------------------------
                                     68 ; absolute internal ram data
                                     69 ;--------------------------------------------------------
                                     70 	.area IABS    (ABS,DATA)
                                     71 	.area IABS    (ABS,DATA)
                                     72 ;--------------------------------------------------------
                                     73 ; bit data
                                     74 ;--------------------------------------------------------
                                     75 	.area BSEG    (BIT)
                                     76 ;--------------------------------------------------------
                                     77 ; paged external ram data
                                     78 ;--------------------------------------------------------
                                     79 	.area PSEG    (PAG,XDATA)
                                     80 ;--------------------------------------------------------
                                     81 ; uninitialized external ram data
                                     82 ;--------------------------------------------------------
                                     83 	.area XSEG    (XDATA)
                                     84 ;--------------------------------------------------------
                                     85 ; absolute external ram data
                                     86 ;--------------------------------------------------------
                                     87 	.area XABS    (ABS,XDATA)
                                     88 ;--------------------------------------------------------
                                     89 ; initialized external ram data
                                     90 ;--------------------------------------------------------
                                     91 	.area XISEG   (XDATA)
                                     92 	.area HOME    (CODE)
                                     93 	.area GSINIT0 (CODE)
                                     94 	.area GSINIT1 (CODE)
                                     95 	.area GSINIT2 (CODE)
                                     96 	.area GSINIT3 (CODE)
                                     97 	.area GSINIT4 (CODE)
                                     98 	.area GSINIT5 (CODE)
                                     99 	.area GSINIT  (CODE)
                                    100 	.area GSFINAL (CODE)
                                    101 	.area CSEG    (CODE)
                                    102 ;--------------------------------------------------------
                                    103 ; global & static initialisations
                                    104 ;--------------------------------------------------------
                                    105 	.area HOME    (CODE)
                                    106 	.area GSINIT  (CODE)
                                    107 	.area GSFINAL (CODE)
                                    108 	.area GSINIT  (CODE)
                                    109 ;--------------------------------------------------------
                                    110 ; Home
                                    111 ;--------------------------------------------------------
                                    112 	.area HOME    (CODE)
                                    113 	.area HOME    (CODE)
                                    114 ;--------------------------------------------------------
                                    115 ; code
                                    116 ;--------------------------------------------------------
                                    117 	.area CSEG    (CODE)
                                    118 ;------------------------------------------------------------
                                    119 ;Allocation info for local variables in function 'Pressure_OnByte'
                                    120 ;------------------------------------------------------------
                                    121 ;b             Allocated to registers r7 
                                    122 ;pressure      Allocated to registers r6 r7 
                                    123 ;------------------------------------------------------------
                                    124 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:12: void Pressure_OnByte(uint8_t b)
                                    125 ;	-----------------------------------------
                                    126 ;	 function Pressure_OnByte
                                    127 ;	-----------------------------------------
      0004C4                        128 _Pressure_OnByte:
                           000007   129 	ar7 = 0x07
                           000006   130 	ar6 = 0x06
                           000005   131 	ar5 = 0x05
                           000004   132 	ar4 = 0x04
                           000003   133 	ar3 = 0x03
                           000002   134 	ar2 = 0x02
                           000001   135 	ar1 = 0x01
                           000000   136 	ar0 = 0x00
      0004C4 AF 82            [24]  137 	mov	r7, dpl
                                    138 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:14: if (b == 0xFEU) {
      0004C6 BF FE 04         [24]  139 	cjne	r7,#0xfe,00102$
                                    140 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:15: g_frame_pos = 1U;
      0004C9 75 23 01         [24]  141 	mov	_g_frame_pos,#0x01
                                    142 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:16: return;
      0004CC 22               [24]  143 	ret
      0004CD                        144 00102$:
                                    145 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:19: switch (g_frame_pos) {
      0004CD E5 23            [12]  146 	mov	a,_g_frame_pos
      0004CF 24 FB            [12]  147 	add	a,#0xff - 0x04
      0004D1 40 2D            [24]  148 	jc	00108$
                                    149 ;	free result
      0004D3 E5 23            [12]  150 	mov	a,_g_frame_pos
      0004D5 75 F0 03         [24]  151 	mov	b,#0x03
      0004D8 A4               [48]  152 	mul	ab
      0004D9 90 04 DD         [24]  153 	mov	dptr,#00148$
      0004DC 73               [24]  154 	jmp	@a+dptr
      0004DD                        155 00148$:
      0004DD 02 05 25         [24]  156 	ljmp	00114$
      0004E0 02 04 EC         [24]  157 	ljmp	00104$
      0004E3 02 04 F2         [24]  158 	ljmp	00105$
      0004E6 02 04 F8         [24]  159 	ljmp	00106$
      0004E9 02 04 FC         [24]  160 	ljmp	00107$
                                    161 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:23: case 1:
      0004EC                        162 00104$:
                                    163 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:24: g_pressure_hi = b;
      0004EC 8F 24            [24]  164 	mov	_g_pressure_hi,r7
                                    165 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:25: g_frame_pos = 2U;
      0004EE 75 23 02         [24]  166 	mov	_g_frame_pos,#0x02
                                    167 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:26: break;
                                    168 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:28: case 2:
      0004F1 22               [24]  169 	ret
      0004F2                        170 00105$:
                                    171 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:29: g_pressure_lo = b;
      0004F2 8F 25            [24]  172 	mov	_g_pressure_lo,r7
                                    173 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:30: g_frame_pos = 3U;
      0004F4 75 23 03         [24]  174 	mov	_g_frame_pos,#0x03
                                    175 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:31: break;
                                    176 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:33: case 3:
      0004F7 22               [24]  177 	ret
      0004F8                        178 00106$:
                                    179 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:34: g_frame_pos = 4U;
      0004F8 75 23 04         [24]  180 	mov	_g_frame_pos,#0x04
                                    181 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:35: break;
                                    182 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:37: case 4:
      0004FB 22               [24]  183 	ret
      0004FC                        184 00107$:
                                    185 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:38: g_frame_pos = 5U;
      0004FC 75 23 05         [24]  186 	mov	_g_frame_pos,#0x05
                                    187 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:39: break;
                                    188 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:41: default:
      0004FF 22               [24]  189 	ret
      000500                        190 00108$:
                                    191 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:42: if (b == 0xDCU) {
      000500 BF DC 1F         [24]  192 	cjne	r7,#0xdc,00112$
                                    193 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:43: uint16_t pressure = ((uint16_t)g_pressure_hi << 8) | g_pressure_lo;
      000503 AF 24            [24]  194 	mov	r7,_g_pressure_hi
      000505 7E 00            [12]  195 	mov	r6,#0x00
      000507 AC 25            [24]  196 	mov	r4,_g_pressure_lo
      000509 7D 00            [12]  197 	mov	r5,#0x00
      00050B EC               [12]  198 	mov	a,r4
      00050C 42 06            [12]  199 	orl	ar6,a
      00050E ED               [12]  200 	mov	a,r5
      00050F 42 07            [12]  201 	orl	ar7,a
                                    202 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:44: if (pressure <= 999U) {
      000511 8E 04            [24]  203 	mov	ar4,r6
      000513 8F 05            [24]  204 	mov	ar5,r7
      000515 C3               [12]  205 	clr	c
      000516 74 E7            [12]  206 	mov	a,#0xe7
      000518 9C               [12]  207 	subb	a,r4
      000519 74 03            [12]  208 	mov	a,#0x03
      00051B 9D               [12]  209 	subb	a,r5
      00051C 40 04            [24]  210 	jc	00112$
                                    211 ;	free result
                                    212 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:45: g_pressure = pressure;
      00051E 8E 21            [24]  213 	mov	_g_pressure,r6
      000520 8F 22            [24]  214 	mov	(_g_pressure + 1),r7
      000522                        215 00112$:
                                    216 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:48: g_frame_pos = 0U;
      000522 75 23 00         [24]  217 	mov	_g_frame_pos,#0x00
                                    218 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:50: }
      000525                        219 00114$:
                                    220 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:51: }
      000525 22               [24]  221 	ret
                                    222 ;------------------------------------------------------------
                                    223 ;Allocation info for local variables in function 'Pressure_Init'
                                    224 ;------------------------------------------------------------
                                    225 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:53: void Pressure_Init(void)
                                    226 ;	-----------------------------------------
                                    227 ;	 function Pressure_Init
                                    228 ;	-----------------------------------------
      000526                        229 _Pressure_Init:
                                    230 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:55: g_pressure = 0U;
      000526 E4               [12]  231 	clr	a
      000527 F5 21            [12]  232 	mov	_g_pressure,a
      000529 F5 22            [12]  233 	mov	(_g_pressure + 1),a
                                    234 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:56: g_frame_pos = 0U;
      00052B F5 23            [12]  235 	mov	_g_frame_pos,a
                                    236 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:57: g_pressure_hi = 0U;
      00052D F5 24            [12]  237 	mov	_g_pressure_hi,a
                                    238 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:58: g_pressure_lo = 0U;
      00052F F5 25            [12]  239 	mov	_g_pressure_lo,a
                                    240 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:59: }
      000531 22               [24]  241 	ret
                                    242 ;------------------------------------------------------------
                                    243 ;Allocation info for local variables in function 'Pressure_GetValue'
                                    244 ;------------------------------------------------------------
                                    245 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:61: uint16_t Pressure_GetValue(void)
                                    246 ;	-----------------------------------------
                                    247 ;	 function Pressure_GetValue
                                    248 ;	-----------------------------------------
      000532                        249 _Pressure_GetValue:
                                    250 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:63: return g_pressure;
      000532 85 21 82         [24]  251 	mov	dpl, _g_pressure
      000535 85 22 83         [24]  252 	mov	dph, (_g_pressure + 1)
                                    253 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:64: }
      000538 22               [24]  254 	ret
                                    255 ;------------------------------------------------------------
                                    256 ;Allocation info for local variables in function 'Pressure_ProcessRx'
                                    257 ;------------------------------------------------------------
                                    258 ;b             Allocated with name '_Pressure_ProcessRx_b_10000_37'
                                    259 ;------------------------------------------------------------
                                    260 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:66: void Pressure_ProcessRx(void)
                                    261 ;	-----------------------------------------
                                    262 ;	 function Pressure_ProcessRx
                                    263 ;	-----------------------------------------
      000539                        264 _Pressure_ProcessRx:
                                    265 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:70: while (Uart1_ReadByte(&b) != 0U) {
      000539                        266 00101$:
      000539 90 00 26         [24]  267 	mov	dptr,#_Pressure_ProcessRx_b_10000_37
      00053C 75 F0 40         [24]  268 	mov	b, #0x40
      00053F 12 07 71         [24]  269 	lcall	_Uart1_ReadByte
      000542 E5 82            [12]  270 	mov	a, dpl
      000544 60 08            [24]  271 	jz	00104$
                                    272 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:71: Pressure_OnByte(b);
      000546 85 26 82         [24]  273 	mov	dpl, _Pressure_ProcessRx_b_10000_37
      000549 12 04 C4         [24]  274 	lcall	_Pressure_OnByte
      00054C 80 EB            [24]  275 	sjmp	00101$
      00054E                        276 00104$:
                                    277 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:73: }
      00054E 22               [24]  278 	ret
                                    279 ;------------------------------------------------------------
                                    280 ;Allocation info for local variables in function 'Pressure_ProcessControllerRx'
                                    281 ;------------------------------------------------------------
                                    282 ;b             Allocated with name '_Pressure_ProcessControllerRx_b_10000_40'
                                    283 ;------------------------------------------------------------
                                    284 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:75: void Pressure_ProcessControllerRx(void)
                                    285 ;	-----------------------------------------
                                    286 ;	 function Pressure_ProcessControllerRx
                                    287 ;	-----------------------------------------
      00054F                        288 _Pressure_ProcessControllerRx:
                                    289 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:79: while (Uart2_ReadByte(&b) != 0U) {
      00054F                        290 00101$:
      00054F 90 00 27         [24]  291 	mov	dptr,#_Pressure_ProcessControllerRx_b_10000_40
      000552 75 F0 40         [24]  292 	mov	b, #0x40
      000555 12 07 9E         [24]  293 	lcall	_Uart2_ReadByte
      000558 E5 82            [12]  294 	mov	a, dpl
      00055A 60 08            [24]  295 	jz	00104$
                                    296 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:80: Pressure_OnByte(b);
      00055C 85 27 82         [24]  297 	mov	dpl, _Pressure_ProcessControllerRx_b_10000_40
      00055F 12 04 C4         [24]  298 	lcall	_Pressure_OnByte
      000562 80 EB            [24]  299 	sjmp	00101$
      000564                        300 00104$:
                                    301 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:82: }
      000564 22               [24]  302 	ret
                                    303 ;------------------------------------------------------------
                                    304 ;Allocation info for local variables in function 'Pressure_PollBlocking'
                                    305 ;------------------------------------------------------------
                                    306 ;req           Allocated with name '_Pressure_PollBlocking_req_10000_43'
                                    307 ;resp          Allocated with name '_Pressure_PollBlocking_resp_10000_43'
                                    308 ;i             Allocated to registers r7 
                                    309 ;wait          Allocated to registers r5 r6 
                                    310 ;b             Allocated with name '_Pressure_PollBlocking_b_10000_43'
                                    311 ;------------------------------------------------------------
                                    312 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:84: void Pressure_PollBlocking(void)
                                    313 ;	-----------------------------------------
                                    314 ;	 function Pressure_PollBlocking
                                    315 ;	-----------------------------------------
      000565                        316 _Pressure_PollBlocking:
                                    317 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:92: req[0] = PRESSURE_MODULE_ADDR;
      000565 75 28 01         [24]  318 	mov	_Pressure_PollBlocking_req_10000_43,#0x01
                                    319 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:93: req[1] = 0x03U;
      000568 75 29 03         [24]  320 	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0001),#0x03
                                    321 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:94: req[2] = 0x00U;
      00056B 75 2A 00         [24]  322 	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0002),#0x00
                                    323 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:95: req[3] = 0x00U;
      00056E 75 2B 00         [24]  324 	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0003),#0x00
                                    325 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:96: req[4] = 0x00U;
      000571 75 2C 00         [24]  326 	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0004),#0x00
                                    327 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:97: req[5] = 0x01U;
      000574 75 2D 01         [24]  328 	mov	(_Pressure_PollBlocking_req_10000_43 + 0x0005),#0x01
                                    329 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:98: Crc16_Append(req, 6U);
      000577 75 0B 06         [24]  330 	mov	_Crc16_Append_PARM_2,#0x06
      00057A 90 00 28         [24]  331 	mov	dptr,#_Pressure_PollBlocking_req_10000_43
      00057D 75 F0 40         [24]  332 	mov	b, #0x40
      000580 12 02 9F         [24]  333 	lcall	_Crc16_Append
                                    334 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:100: Uart1_Send(req, sizeof(req));
      000583 75 13 08         [24]  335 	mov	_Uart1_Send_PARM_2,#0x08
      000586 90 00 28         [24]  336 	mov	dptr,#_Pressure_PollBlocking_req_10000_43
      000589 75 F0 40         [24]  337 	mov	b, #0x40
      00058C 12 07 BF         [24]  338 	lcall	_Uart1_Send
                                    339 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:102: for (wait = 0U; wait < 30000U; ++wait) {
      00058F 7F 00            [12]  340 	mov	r7,#0x00
      000591 7D 00            [12]  341 	mov	r5,#0x00
      000593 7E 00            [12]  342 	mov	r6,#0x00
      000595                        343 00114$:
                                    344 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:103: if (Uart1_ReadByte(&b) != 0U) {
      000595 90 00 37         [24]  345 	mov	dptr,#_Pressure_PollBlocking_b_10000_43
      000598 75 F0 40         [24]  346 	mov	b, #0x40
      00059B C0 07            [24]  347 	push	ar7
      00059D C0 06            [24]  348 	push	ar6
      00059F C0 05            [24]  349 	push	ar5
      0005A1 12 07 71         [24]  350 	lcall	_Uart1_ReadByte
      0005A4 E5 82            [12]  351 	mov	a, dpl
      0005A6 D0 05            [24]  352 	pop	ar5
      0005A8 D0 06            [24]  353 	pop	ar6
      0005AA D0 07            [24]  354 	pop	ar7
      0005AC 60 0C            [24]  355 	jz	00115$
                                    356 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:105: resp[i++] = b;
      0005AE EF               [12]  357 	mov	a,r7
      0005AF 24 30            [12]  358 	add	a, #_Pressure_PollBlocking_resp_10000_43
      0005B1 F9               [12]  359 	mov	r1,a
      0005B2 0F               [12]  360 	inc	r7
      0005B3 A7 37            [24]  361 	mov	@r1,_Pressure_PollBlocking_b_10000_43
                                    362 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:107: if (i >= sizeof(resp)) {
      0005B5 BF 07 00         [24]  363 	cjne	r7,#0x07,00172$
      0005B8                        364 00172$:
      0005B8 50 12            [24]  365 	jnc	00107$
                                    366 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:108: break;
      0005BA                        367 00115$:
                                    368 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:102: for (wait = 0U; wait < 30000U; ++wait) {
      0005BA 0D               [12]  369 	inc	r5
      0005BB BD 00 01         [24]  370 	cjne	r5,#0x00,00174$
      0005BE 0E               [12]  371 	inc	r6
      0005BF                        372 00174$:
      0005BF 8D 03            [24]  373 	mov	ar3,r5
      0005C1 8E 04            [24]  374 	mov	ar4,r6
      0005C3 C3               [12]  375 	clr	c
      0005C4 EB               [12]  376 	mov	a,r3
      0005C5 94 30            [12]  377 	subb	a,#0x30
      0005C7 EC               [12]  378 	mov	a,r4
      0005C8 94 75            [12]  379 	subb	a,#0x75
      0005CA 40 C9            [24]  380 	jc	00114$
      0005CC                        381 00107$:
                                    382 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:113: if ((i == sizeof(resp)) &&
      0005CC BF 07 2F         [24]  383 	cjne	r7,#0x07,00116$
                                    384 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:114: (resp[0] == PRESSURE_MODULE_ADDR) &&
      0005CF 74 01            [12]  385 	mov	a,#0x01
      0005D1 B5 30 2A         [24]  386 	cjne	a,_Pressure_PollBlocking_resp_10000_43,00116$
                                    387 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:115: (resp[1] == 0x03U) &&
      0005D4 74 03            [12]  388 	mov	a,#0x03
      0005D6 B5 31 25         [24]  389 	cjne	a,(_Pressure_PollBlocking_resp_10000_43 + 0x0001),00116$
                                    390 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:116: (resp[2] == 0x02U) &&
      0005D9 74 02            [12]  391 	mov	a,#0x02
      0005DB B5 32 20         [24]  392 	cjne	a,(_Pressure_PollBlocking_resp_10000_43 + 0x0002),00116$
                                    393 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:117: (Crc16_Check(resp, sizeof(resp)) != 0U)) {
      0005DE 75 08 07         [24]  394 	mov	_Crc16_Check_PARM_2,#0x07
      0005E1 90 00 30         [24]  395 	mov	dptr,#_Pressure_PollBlocking_resp_10000_43
      0005E4 75 F0 40         [24]  396 	mov	b, #0x40
      0005E7 12 02 26         [24]  397 	lcall	_Crc16_Check
      0005EA E5 82            [12]  398 	mov	a, dpl
      0005EC 60 10            [24]  399 	jz	00116$
                                    400 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:118: g_pressure = ((uint16_t)resp[3] << 8) | resp[4];
      0005EE AF 33            [24]  401 	mov	r7,(_Pressure_PollBlocking_resp_10000_43 + 0x0003)
      0005F0 7E 00            [12]  402 	mov	r6,#0x00
      0005F2 AC 34            [24]  403 	mov	r4,(_Pressure_PollBlocking_resp_10000_43 + 0x0004)
      0005F4 7D 00            [12]  404 	mov	r5,#0x00
      0005F6 EC               [12]  405 	mov	a,r4
      0005F7 4E               [12]  406 	orl	a,r6
      0005F8 F5 21            [12]  407 	mov	_g_pressure,a
      0005FA ED               [12]  408 	mov	a,r5
      0005FB 4F               [12]  409 	orl	a,r7
      0005FC F5 22            [12]  410 	mov	(_g_pressure + 1),a
      0005FE                        411 00116$:
                                    412 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/pressure.c:120: }
      0005FE 22               [24]  413 	ret
                                    414 	.area CSEG    (CODE)
                                    415 	.area CONST   (CODE)
                                    416 	.area XINIT   (CODE)
                                    417 	.area CABS    (ABS,CODE)
