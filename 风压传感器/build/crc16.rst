                                      1 ;--------------------------------------------------------
                                      2 ; File Created by SDCC : free open source ISO C Compiler
                                      3 ; Version 4.6.0 #16555 (Linux)
                                      4 ;--------------------------------------------------------
                                      5 	.module crc16
                                      6 	
                                      7 	.optsdcc -mmcs51 --model-small
                                      8 ;--------------------------------------------------------
                                      9 ; Public variables in this module
                                     10 ;--------------------------------------------------------
                                     11 	.globl _Crc16_Modbus_PARM_2
                                     12 	.globl _Crc16_Append_PARM_2
                                     13 	.globl _Crc16_Check_PARM_2
                                     14 	.globl _Crc16_Modbus
                                     15 	.globl _Crc16_Check
                                     16 	.globl _Crc16_Append
                                     17 ;--------------------------------------------------------
                                     18 ; special function registers
                                     19 ;--------------------------------------------------------
                                     20 	.area RSEG    (ABS,DATA)
      000000                         21 	.org 0x0000
                                     22 ;--------------------------------------------------------
                                     23 ; special function bits
                                     24 ;--------------------------------------------------------
                                     25 	.area RSEG    (ABS,DATA)
      000000                         26 	.org 0x0000
                                     27 ;--------------------------------------------------------
                                     28 ; overlayable register banks
                                     29 ;--------------------------------------------------------
                                     30 	.area REG_BANK_0	(REL,OVR,DATA)
      000000                         31 	.ds 8
                                     32 ;--------------------------------------------------------
                                     33 ; internal ram data
                                     34 ;--------------------------------------------------------
                                     35 	.area DSEG    (DATA)
      000008                         36 _Crc16_Check_PARM_2:
      000008                         37 	.ds 1
      000009                         38 _Crc16_Check_crc_10000_14:
      000009                         39 	.ds 2
      00000B                         40 _Crc16_Append_PARM_2:
      00000B                         41 	.ds 1
      00000C                         42 _Crc16_Append_crc_10000_18:
      00000C                         43 	.ds 2
                                     44 ;--------------------------------------------------------
                                     45 ; overlayable items in internal ram
                                     46 ;--------------------------------------------------------
                                     47 	.area	OSEG    (OVR,DATA)
      000015                         48 _Crc16_Modbus_PARM_2:
      000015                         49 	.ds 1
      000016                         50 _Crc16_Modbus_data_10000_4:
      000016                         51 	.ds 3
                                     52 ;--------------------------------------------------------
                                     53 ; indirectly addressable internal ram data
                                     54 ;--------------------------------------------------------
                                     55 	.area ISEG    (DATA)
                                     56 ;--------------------------------------------------------
                                     57 ; absolute internal ram data
                                     58 ;--------------------------------------------------------
                                     59 	.area IABS    (ABS,DATA)
                                     60 	.area IABS    (ABS,DATA)
                                     61 ;--------------------------------------------------------
                                     62 ; bit data
                                     63 ;--------------------------------------------------------
                                     64 	.area BSEG    (BIT)
                                     65 ;--------------------------------------------------------
                                     66 ; paged external ram data
                                     67 ;--------------------------------------------------------
                                     68 	.area PSEG    (PAG,XDATA)
                                     69 ;--------------------------------------------------------
                                     70 ; uninitialized external ram data
                                     71 ;--------------------------------------------------------
                                     72 	.area XSEG    (XDATA)
                                     73 ;--------------------------------------------------------
                                     74 ; absolute external ram data
                                     75 ;--------------------------------------------------------
                                     76 	.area XABS    (ABS,XDATA)
                                     77 ;--------------------------------------------------------
                                     78 ; initialized external ram data
                                     79 ;--------------------------------------------------------
                                     80 	.area XISEG   (XDATA)
                                     81 	.area HOME    (CODE)
                                     82 	.area GSINIT0 (CODE)
                                     83 	.area GSINIT1 (CODE)
                                     84 	.area GSINIT2 (CODE)
                                     85 	.area GSINIT3 (CODE)
                                     86 	.area GSINIT4 (CODE)
                                     87 	.area GSINIT5 (CODE)
                                     88 	.area GSINIT  (CODE)
                                     89 	.area GSFINAL (CODE)
                                     90 	.area CSEG    (CODE)
                                     91 ;--------------------------------------------------------
                                     92 ; global & static initialisations
                                     93 ;--------------------------------------------------------
                                     94 	.area HOME    (CODE)
                                     95 	.area GSINIT  (CODE)
                                     96 	.area GSFINAL (CODE)
                                     97 	.area GSINIT  (CODE)
                                     98 ;--------------------------------------------------------
                                     99 ; Home
                                    100 ;--------------------------------------------------------
                                    101 	.area HOME    (CODE)
                                    102 	.area HOME    (CODE)
                                    103 ;--------------------------------------------------------
                                    104 ; code
                                    105 ;--------------------------------------------------------
                                    106 	.area CSEG    (CODE)
                                    107 ;------------------------------------------------------------
                                    108 ;Allocation info for local variables in function 'Crc16_Modbus'
                                    109 ;------------------------------------------------------------
                                    110 ;len           Allocated with name '_Crc16_Modbus_PARM_2'
                                    111 ;data          Allocated with name '_Crc16_Modbus_data_10000_4'
                                    112 ;crc           Allocated to registers r3 r4 
                                    113 ;i             Allocated to registers r2 
                                    114 ;bit           Allocated to registers r7 
                                    115 ;------------------------------------------------------------
                                    116 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:3: uint16_t Crc16_Modbus(const uint8_t *data, uint8_t len)
                                    117 ;	-----------------------------------------
                                    118 ;	 function Crc16_Modbus
                                    119 ;	-----------------------------------------
      0001C7                        120 _Crc16_Modbus:
                           000007   121 	ar7 = 0x07
                           000006   122 	ar6 = 0x06
                           000005   123 	ar5 = 0x05
                           000004   124 	ar4 = 0x04
                           000003   125 	ar3 = 0x03
                           000002   126 	ar2 = 0x02
                           000001   127 	ar1 = 0x01
                           000000   128 	ar0 = 0x00
      0001C7 85 82 16         [24]  129 	mov	_Crc16_Modbus_data_10000_4,dpl
      0001CA 85 83 17         [24]  130 	mov	(_Crc16_Modbus_data_10000_4 + 1),dph
      0001CD 85 F0 18         [24]  131 	mov	(_Crc16_Modbus_data_10000_4 + 2),b
                                    132 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:5: uint16_t crc = 0xFFFFU;
      0001D0 7B FF            [12]  133 	mov	r3,#0xff
      0001D2 7C FF            [12]  134 	mov	r4,#0xff
                                    135 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:9: for (i = 0U; i < len; ++i) {
      0001D4 7A 00            [12]  136 	mov	r2,#0x00
      0001D6                        137 00109$:
      0001D6 C3               [12]  138 	clr	c
      0001D7 EA               [12]  139 	mov	a,r2
      0001D8 95 15            [12]  140 	subb	a,_Crc16_Modbus_PARM_2
      0001DA 50 45            [24]  141 	jnc	00105$
                                    142 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:10: crc ^= data[i];
      0001DC EA               [12]  143 	mov	a,r2
      0001DD 25 16            [12]  144 	add	a, _Crc16_Modbus_data_10000_4
      0001DF F8               [12]  145 	mov	r0,a
      0001E0 E4               [12]  146 	clr	a
      0001E1 35 17            [12]  147 	addc	a, (_Crc16_Modbus_data_10000_4 + 1)
      0001E3 F9               [12]  148 	mov	r1,a
      0001E4 AF 18            [24]  149 	mov	r7,(_Crc16_Modbus_data_10000_4 + 2)
      0001E6 88 82            [24]  150 	mov	dpl,r0
      0001E8 89 83            [24]  151 	mov	dph,r1
      0001EA 8F F0            [24]  152 	mov	b,r7
      0001EC 12 09 25         [24]  153 	lcall	__gptrget
      0001EF 7F 00            [12]  154 	mov	r7,#0x00
      0001F1 62 03            [12]  155 	xrl	ar3,a
      0001F3 EF               [12]  156 	mov	a,r7
      0001F4 62 04            [12]  157 	xrl	ar4,a
                                    158 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:11: for (bit = 0U; bit < 8U; ++bit) {
      0001F6 7F 00            [12]  159 	mov	r7,#0x00
      0001F8                        160 00106$:
                                    161 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:12: if ((crc & 0x0001U) != 0U) {
      0001F8 EB               [12]  162 	mov	a,r3
      0001F9 30 E0 15         [24]  163 	jnb	acc.0,00102$
                                    164 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:13: crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      0001FC 8B 05            [24]  165 	mov	ar5,r3
      0001FE EC               [12]  166 	mov	a,r4
      0001FF C3               [12]  167 	clr	c
      000200 13               [12]  168 	rrc	a
      000201 CD               [12]  169 	xch	a,r5
      000202 13               [12]  170 	rrc	a
      000203 CD               [12]  171 	xch	a,r5
      000204 FE               [12]  172 	mov	r6,a
      000205 63 05 01         [24]  173 	xrl	ar5,#0x01
      000208 63 06 A0         [24]  174 	xrl	ar6,#0xa0
      00020B 8D 03            [24]  175 	mov	ar3,r5
      00020D 8E 04            [24]  176 	mov	ar4,r6
      00020F 80 07            [24]  177 	sjmp	00107$
      000211                        178 00102$:
                                    179 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:15: crc >>= 1;
      000211 EC               [12]  180 	mov	a,r4
      000212 C3               [12]  181 	clr	c
      000213 13               [12]  182 	rrc	a
      000214 CB               [12]  183 	xch	a,r3
      000215 13               [12]  184 	rrc	a
      000216 CB               [12]  185 	xch	a,r3
      000217 FC               [12]  186 	mov	r4,a
      000218                        187 00107$:
                                    188 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:11: for (bit = 0U; bit < 8U; ++bit) {
      000218 0F               [12]  189 	inc	r7
      000219 BF 08 00         [24]  190 	cjne	r7,#0x08,00152$
      00021C                        191 00152$:
      00021C 40 DA            [24]  192 	jc	00106$
                                    193 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:9: for (i = 0U; i < len; ++i) {
      00021E 0A               [12]  194 	inc	r2
      00021F 80 B5            [24]  195 	sjmp	00109$
      000221                        196 00105$:
                                    197 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:20: return crc;
      000221 8B 82            [24]  198 	mov	dpl, r3
      000223 8C 83            [24]  199 	mov	dph, r4
                                    200 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:21: }
      000225 22               [24]  201 	ret
                                    202 ;------------------------------------------------------------
                                    203 ;Allocation info for local variables in function 'Crc16_Check'
                                    204 ;------------------------------------------------------------
                                    205 ;len           Allocated with name '_Crc16_Check_PARM_2'
                                    206 ;frame         Allocated to registers r5 r6 r7 
                                    207 ;crc           Allocated with name '_Crc16_Check_crc_10000_14'
                                    208 ;------------------------------------------------------------
                                    209 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:23: uint8_t Crc16_Check(const uint8_t *frame, uint8_t len)
                                    210 ;	-----------------------------------------
                                    211 ;	 function Crc16_Check
                                    212 ;	-----------------------------------------
      000226                        213 _Crc16_Check:
      000226 AD 82            [24]  214 	mov	r5, dpl
      000228 AE 83            [24]  215 	mov	r6, dph
      00022A AF F0            [24]  216 	mov	r7, b
                                    217 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:27: if ((frame == 0) || (len < 3U)) {
      00022C ED               [12]  218 	mov	a,r5
      00022D 4E               [12]  219 	orl	a,r6
      00022E 60 06            [24]  220 	jz	00101$
      000230 74 FD            [12]  221 	mov	a,#0x100 - 0x03
      000232 25 08            [12]  222 	add	a,_Crc16_Check_PARM_2
      000234 40 04            [24]  223 	jc	00102$
      000236                        224 00101$:
                                    225 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:28: return 0U;
      000236 75 82 00         [24]  226 	mov	dpl, #0x00
      000239 22               [24]  227 	ret
      00023A                        228 00102$:
                                    229 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:31: crc = Crc16_Modbus(frame, (uint8_t)(len - 2U));
      00023A E5 08            [12]  230 	mov	a,_Crc16_Check_PARM_2
      00023C 24 FE            [12]  231 	add	a,#0xfe
      00023E F5 15            [12]  232 	mov	_Crc16_Modbus_PARM_2,a
      000240 8D 82            [24]  233 	mov	dpl, r5
      000242 8E 83            [24]  234 	mov	dph, r6
      000244 8F F0            [24]  235 	mov	b, r7
      000246 C0 07            [24]  236 	push	ar7
      000248 C0 06            [24]  237 	push	ar6
      00024A C0 05            [24]  238 	push	ar5
      00024C 12 01 C7         [24]  239 	lcall	_Crc16_Modbus
      00024F 85 82 09         [24]  240 	mov	_Crc16_Check_crc_10000_14,dpl
      000252 85 83 0A         [24]  241 	mov	(_Crc16_Check_crc_10000_14 + 1),dph
      000255 D0 05            [24]  242 	pop	ar5
      000257 D0 06            [24]  243 	pop	ar6
      000259 D0 07            [24]  244 	pop	ar7
                                    245 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:32: return ((frame[len - 2U] == (uint8_t)crc) &&
      00025B A9 08            [24]  246 	mov	r1,_Crc16_Check_PARM_2
      00025D 7A 00            [12]  247 	mov	r2,#0x00
      00025F E9               [12]  248 	mov	a,r1
      000260 24 FE            [12]  249 	add	a,#0xfe
      000262 F8               [12]  250 	mov	r0,a
      000263 EA               [12]  251 	mov	a,r2
      000264 34 FF            [12]  252 	addc	a,#0xff
      000266 FC               [12]  253 	mov	r4,a
      000267 E8               [12]  254 	mov	a,r0
      000268 2D               [12]  255 	add	a, r5
      000269 F8               [12]  256 	mov	r0,a
      00026A EC               [12]  257 	mov	a,r4
      00026B 3E               [12]  258 	addc	a, r6
      00026C FC               [12]  259 	mov	r4,a
      00026D 8F 03            [24]  260 	mov	ar3,r7
      00026F 88 82            [24]  261 	mov	dpl,r0
      000271 8C 83            [24]  262 	mov	dph,r4
      000273 8B F0            [24]  263 	mov	b,r3
      000275 12 09 25         [24]  264 	lcall	__gptrget
      000278 AB 09            [24]  265 	mov	r3,_Crc16_Check_crc_10000_14
      00027A B5 03 1D         [24]  266 	cjne	a,ar3,00106$
                                    267 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:33: (frame[len - 1U] == (uint8_t)(crc >> 8))) ? 1U : 0U;
      00027D 19               [12]  268 	dec	r1
      00027E B9 FF 01         [24]  269 	cjne	r1,#0xff,00135$
      000281 1A               [12]  270 	dec	r2
      000282                        271 00135$:
      000282 E9               [12]  272 	mov	a,r1
      000283 2D               [12]  273 	add	a, r5
      000284 FD               [12]  274 	mov	r5,a
      000285 EA               [12]  275 	mov	a,r2
      000286 3E               [12]  276 	addc	a, r6
      000287 FE               [12]  277 	mov	r6,a
      000288 8D 82            [24]  278 	mov	dpl,r5
      00028A 8E 83            [24]  279 	mov	dph,r6
      00028C 8F F0            [24]  280 	mov	b,r7
      00028E 12 09 25         [24]  281 	lcall	__gptrget
      000291 AF 0A            [24]  282 	mov	r7,(_Crc16_Check_crc_10000_14 + 1)
      000293 B5 07 04         [24]  283 	cjne	a,ar7,00106$
      000296 7F 01            [12]  284 	mov	r7,#0x01
      000298 80 02            [24]  285 	sjmp	00107$
      00029A                        286 00106$:
      00029A 7F 00            [12]  287 	mov	r7,#0x00
      00029C                        288 00107$:
      00029C 8F 82            [24]  289 	mov	dpl,r7
                                    290 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:34: }
      00029E 22               [24]  291 	ret
                                    292 ;------------------------------------------------------------
                                    293 ;Allocation info for local variables in function 'Crc16_Append'
                                    294 ;------------------------------------------------------------
                                    295 ;payload_len   Allocated with name '_Crc16_Append_PARM_2'
                                    296 ;frame         Allocated to registers r5 r6 r7 
                                    297 ;crc           Allocated with name '_Crc16_Append_crc_10000_18'
                                    298 ;------------------------------------------------------------
                                    299 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:36: void Crc16_Append(uint8_t *frame, uint8_t payload_len)
                                    300 ;	-----------------------------------------
                                    301 ;	 function Crc16_Append
                                    302 ;	-----------------------------------------
      00029F                        303 _Crc16_Append:
                                    304 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:38: uint16_t crc = Crc16_Modbus(frame, payload_len);
      00029F AD 82            [24]  305 	mov	r5,dpl
      0002A1 AE 83            [24]  306 	mov	r6,dph
      0002A3 AF F0            [24]  307 	mov	r7,b
      0002A5 85 0B 15         [24]  308 	mov	_Crc16_Modbus_PARM_2,_Crc16_Append_PARM_2
      0002A8 C0 07            [24]  309 	push	ar7
      0002AA C0 06            [24]  310 	push	ar6
      0002AC C0 05            [24]  311 	push	ar5
      0002AE 12 01 C7         [24]  312 	lcall	_Crc16_Modbus
      0002B1 85 82 0C         [24]  313 	mov	_Crc16_Append_crc_10000_18,dpl
      0002B4 85 83 0D         [24]  314 	mov	(_Crc16_Append_crc_10000_18 + 1),dph
      0002B7 D0 05            [24]  315 	pop	ar5
      0002B9 D0 06            [24]  316 	pop	ar6
      0002BB D0 07            [24]  317 	pop	ar7
                                    318 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:39: frame[payload_len] = (uint8_t)crc;
      0002BD E5 0B            [12]  319 	mov	a,_Crc16_Append_PARM_2
      0002BF 2D               [12]  320 	add	a, r5
      0002C0 F8               [12]  321 	mov	r0,a
      0002C1 E4               [12]  322 	clr	a
      0002C2 3E               [12]  323 	addc	a, r6
      0002C3 F9               [12]  324 	mov	r1,a
      0002C4 8F 02            [24]  325 	mov	ar2,r7
      0002C6 AC 0C            [24]  326 	mov	r4,_Crc16_Append_crc_10000_18
      0002C8 88 82            [24]  327 	mov	dpl,r0
      0002CA 89 83            [24]  328 	mov	dph,r1
      0002CC 8A F0            [24]  329 	mov	b,r2
      0002CE EC               [12]  330 	mov	a,r4
      0002CF 12 08 BD         [24]  331 	lcall	__gptrput
                                    332 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:40: frame[payload_len + 1U] = (uint8_t)(crc >> 8);
      0002D2 AC 0B            [24]  333 	mov	r4,_Crc16_Append_PARM_2
      0002D4 7B 00            [12]  334 	mov	r3,#0x00
      0002D6 0C               [12]  335 	inc	r4
      0002D7 BC 00 01         [24]  336 	cjne	r4,#0x00,00103$
      0002DA 0B               [12]  337 	inc	r3
      0002DB                        338 00103$:
      0002DB EC               [12]  339 	mov	a,r4
      0002DC 2D               [12]  340 	add	a, r5
      0002DD FD               [12]  341 	mov	r5,a
      0002DE EB               [12]  342 	mov	a,r3
      0002DF 3E               [12]  343 	addc	a, r6
      0002E0 FE               [12]  344 	mov	r6,a
      0002E1 AC 0D            [24]  345 	mov	r4,(_Crc16_Append_crc_10000_18 + 1)
      0002E3 8D 82            [24]  346 	mov	dpl,r5
      0002E5 8E 83            [24]  347 	mov	dph,r6
      0002E7 8F F0            [24]  348 	mov	b,r7
      0002E9 EC               [12]  349 	mov	a,r4
                                    350 ;	/home/keake/Projects/WindPressureDetectionSystem/风压传感器/src/crc16.c:41: }
      0002EA 02 08 BD         [24]  351 	ljmp	__gptrput
                                    352 	.area CSEG    (CODE)
                                    353 	.area CONST   (CODE)
                                    354 	.area XINIT   (CODE)
                                    355 	.area CABS    (ABS,CODE)
