#ifndef THERMISTORTABLES_H_
#define THERMISTORTABLES_H_

#define ADC_VREF_4       3300*4  // 3.3 * 1000




#if (THERMISTORHEATER == 1) || (THERMISTORBED == 1) //100k bed thermistor

#define NUMTEMPS_1 61
const short temptable_1[NUMTEMPS_1][2] = {
{	(ADC_VREF_4*23) / 0xFFF	,	300	},
{	(ADC_VREF_4*25) / 0xFFF	,	295	},
{	(ADC_VREF_4*27) / 0xFFF	,	290	},
{	(ADC_VREF_4*28) / 0xFFF	,	285	},
{	(ADC_VREF_4*31) / 0xFFF	,	280	},
{	(ADC_VREF_4*33) / 0xFFF	,	275	},
{	(ADC_VREF_4*35) / 0xFFF	,	270	},
{	(ADC_VREF_4*38) / 0xFFF	,	265	},
{	(ADC_VREF_4*41) / 0xFFF	,	260	},
{	(ADC_VREF_4*44) / 0xFFF	,	255	},
{	(ADC_VREF_4*48) / 0xFFF	,	250	},
{	(ADC_VREF_4*52) / 0xFFF	,	245	},
{	(ADC_VREF_4*56) / 0xFFF	,	240	},
{	(ADC_VREF_4*61) / 0xFFF	,	235	},
{	(ADC_VREF_4*66) / 0xFFF	,	230	},
{	(ADC_VREF_4*71) / 0xFFF	,	225	},
{	(ADC_VREF_4*78) / 0xFFF	,	220	},
{	(ADC_VREF_4*84) / 0xFFF	,	215	},
{	(ADC_VREF_4*92) / 0xFFF	,	210	},
{	(ADC_VREF_4*100) / 0xFFF	,	205	},
{	(ADC_VREF_4*109) / 0xFFF	,	200	},
{	(ADC_VREF_4*120) / 0xFFF	,	195	},
{	(ADC_VREF_4*131) / 0xFFF	,	190	},
{	(ADC_VREF_4*143) / 0xFFF	,	185	},
{	(ADC_VREF_4*156) / 0xFFF	,	180	},
{	(ADC_VREF_4*171) / 0xFFF	,	175	},
{	(ADC_VREF_4*187) / 0xFFF	,	170	},
{	(ADC_VREF_4*205) / 0xFFF	,	165	},
{	(ADC_VREF_4*224) / 0xFFF	,	160	},
{	(ADC_VREF_4*245) / 0xFFF	,	155	},
{	(ADC_VREF_4*268) / 0xFFF	,	150	},
{	(ADC_VREF_4*293) / 0xFFF	,	145	},
{	(ADC_VREF_4*320) / 0xFFF	,	140	},
{	(ADC_VREF_4*348) / 0xFFF	,	135	},
{	(ADC_VREF_4*379) / 0xFFF	,	130	},
{	(ADC_VREF_4*411) / 0xFFF	,	125	},
{	(ADC_VREF_4*445) / 0xFFF	,	120	},
{	(ADC_VREF_4*480) / 0xFFF	,	115	},
{	(ADC_VREF_4*516) / 0xFFF	,	110	},
{	(ADC_VREF_4*553) / 0xFFF	,	105	},
{	(ADC_VREF_4*591) / 0xFFF	,	100	},
{	(ADC_VREF_4*628) / 0xFFF	,	95	},
{	(ADC_VREF_4*665) / 0xFFF	,	90	},
{	(ADC_VREF_4*702) / 0xFFF	,	85	},
{	(ADC_VREF_4*737) / 0xFFF	,	80	},
{	(ADC_VREF_4*770) / 0xFFF	,	75	},
{	(ADC_VREF_4*801) / 0xFFF	,	70	},
{	(ADC_VREF_4*830) / 0xFFF	,	65	},
{	(ADC_VREF_4*857) / 0xFFF	,	60	},
{	(ADC_VREF_4*881) / 0xFFF	,	55	},
{	(ADC_VREF_4*903) / 0xFFF	,	50	},
{	(ADC_VREF_4*922) / 0xFFF	,	45	},
{	(ADC_VREF_4*939) / 0xFFF	,	40	},
{	(ADC_VREF_4*954) / 0xFFF	,	35	},
{	(ADC_VREF_4*966) / 0xFFF	,	30	},
{	(ADC_VREF_4*977) / 0xFFF	,	25	},
{	(ADC_VREF_4*985) / 0xFFF	,	20	},
{	(ADC_VREF_4*993) / 0xFFF	,	15	},
{	(ADC_VREF_4*999) / 0xFFF	,	10	},
{	(ADC_VREF_4*1004) / 0xFFF	,	5	},
{	(ADC_VREF_4*1008) / 0xFFF	,	0	} //safety
};
#endif

#if (THERMISTORHEATER == 2) || (THERMISTORBED == 2) //200k bed thermistor verified by arcol
#define NUMTEMPS_2 64
const short temptable_2[NUMTEMPS_2][2] = {
   { (ADC_VREF_4*16) / 0xFFF, 315},
   { (ADC_VREF_4*17) / 0xFFF, 310},
   { (ADC_VREF_4*18) / 0xFFF, 305},
   { (ADC_VREF_4*19) / 0xFFF, 300},
   { (ADC_VREF_4*20) / 0xFFF, 295},
   { (ADC_VREF_4*21) / 0xFFF, 290},
   { (ADC_VREF_4*22) / 0xFFF, 285},
   { (ADC_VREF_4*23) / 0xFFF, 280},
   { (ADC_VREF_4*24) / 0xFFF, 275},
   { (ADC_VREF_4*25) / 0xFFF, 270},
   { (ADC_VREF_4*29) / 0xFFF, 265},
   { (ADC_VREF_4*30) / 0xFFF, 260},
   { (ADC_VREF_4*35) / 0xFFF, 255},
   { (ADC_VREF_4*40) / 0xFFF, 250},
   { (ADC_VREF_4*45) / 0xFFF, 245},
   { (ADC_VREF_4*50) / 0xFFF, 240},
   { (ADC_VREF_4*55) / 0xFFF, 235},
   { (ADC_VREF_4*60) / 0xFFF, 230},
   { (ADC_VREF_4*65) / 0xFFF, 225},
   { (ADC_VREF_4*70) / 0xFFF, 220},
   { (ADC_VREF_4*90) / 0xFFF, 215},
   { (ADC_VREF_4*95) / 0xFFF, 210},
   { (ADC_VREF_4*103) / 0xFFF, 205},
   { (ADC_VREF_4*105) / 0xFFF, 200},
   { (ADC_VREF_4*115) / 0xFFF, 195},
   { (ADC_VREF_4*130) / 0xFFF, 190},
   { (ADC_VREF_4*150) / 0xFFF, 185},
   { (ADC_VREF_4*167) / 0xFFF, 180},
   { (ADC_VREF_4*190) / 0xFFF, 175},
   { (ADC_VREF_4*200) / 0xFFF, 170},
   { (ADC_VREF_4*230) / 0xFFF, 165},
   { (ADC_VREF_4*250) / 0xFFF, 160},
   { (ADC_VREF_4*270) / 0xFFF, 155},
   { (ADC_VREF_4*300) / 0xFFF, 150},
   { (ADC_VREF_4*330) / 0xFFF, 145},
   { (ADC_VREF_4*360) / 0xFFF, 140},
   { (ADC_VREF_4*380) / 0xFFF, 135},
   { (ADC_VREF_4*408) / 0xFFF, 130},
   { (ADC_VREF_4*450) / 0xFFF, 125},
   { (ADC_VREF_4*500) / 0xFFF, 120},
   { (ADC_VREF_4*530) / 0xFFF, 115},
   { (ADC_VREF_4*550) / 0xFFF, 110},
   { (ADC_VREF_4*570) / 0xFFF, 105},
   { (ADC_VREF_4*595) / 0xFFF, 100},
   { (ADC_VREF_4*615) / 0xFFF,  95},
   { (ADC_VREF_4*640) / 0xFFF,  90},
   { (ADC_VREF_4*665) / 0xFFF,  85},
   { (ADC_VREF_4*700) / 0xFFF,  80},
   { (ADC_VREF_4*740) / 0xFFF,  75},
   { (ADC_VREF_4*780) / 0xFFF,  70},
   { (ADC_VREF_4*810) / 0xFFF,  65},
   { (ADC_VREF_4*840) / 0xFFF,  60},
   { (ADC_VREF_4*880) / 0xFFF,  55},
   { (ADC_VREF_4*920) / 0xFFF,  50},
   { (ADC_VREF_4*960) / 0xFFF,  45},
   { (ADC_VREF_4*980) / 0xFFF,  40},
   { (ADC_VREF_4*990) / 0xFFF,  35},
   { (ADC_VREF_4*1000) / 0xFFF,  30},
   { (ADC_VREF_4*1005) / 0xFFF,  25},
   { (ADC_VREF_4*1006) / 0xFFF,  20},
   { (ADC_VREF_4*1009) / 0xFFF,  15},
   { (ADC_VREF_4*1010) / 0xFFF,  10},
   { (ADC_VREF_4*1020) / 0xFFF,   5},
   { (ADC_VREF_4*1023) / 0xFFF,   0} //safety
};

#endif
#if (THERMISTORHEATER == 3) || (THERMISTORBED == 3) //mendel-parts
#define NUMTEMPS_3 28
const short temptable_3[NUMTEMPS_3][2] = {
		{ (ADC_VREF_4*1) / 0xFFF,864},
		{ (ADC_VREF_4*21) / 0xFFF,300},
		{ (ADC_VREF_4*25) / 0xFFF,290},
		{ (ADC_VREF_4*29) / 0xFFF,280},
		{ (ADC_VREF_4*33) / 0xFFF,270},
		{ (ADC_VREF_4*39) / 0xFFF,260},
		{ (ADC_VREF_4*46) / 0xFFF,250},
		{ (ADC_VREF_4*54) / 0xFFF,240},
		{ (ADC_VREF_4*64) / 0xFFF,230},
		{ (ADC_VREF_4*75) / 0xFFF,220},
		{ (ADC_VREF_4*90) / 0xFFF,210},
		{ (ADC_VREF_4*107) / 0xFFF,200},
		{ (ADC_VREF_4*128) / 0xFFF,190},
		{ (ADC_VREF_4*154) / 0xFFF,180},
		{ (ADC_VREF_4*184) / 0xFFF,170},
		{ (ADC_VREF_4*221) / 0xFFF,160},
		{ (ADC_VREF_4*265) / 0xFFF,150},
		{ (ADC_VREF_4*316) / 0xFFF,140},
		{ (ADC_VREF_4*375) / 0xFFF,130},
		{ (ADC_VREF_4*441) / 0xFFF,120},
		{ (ADC_VREF_4*513) / 0xFFF,110},
		{ (ADC_VREF_4*588) / 0xFFF,100},
		{ (ADC_VREF_4*734) / 0xFFF,80},
		{ (ADC_VREF_4*856) / 0xFFF,60},
		{ (ADC_VREF_4*938) / 0xFFF,40},
		{ (ADC_VREF_4*986) / 0xFFF,20},
		{ (ADC_VREF_4*1008) / 0xFFF,0},
		{ (ADC_VREF_4*1018) / 0xFFF,-20}
	};

#endif
#if (THERMISTORHEATER == 4) || (THERMISTORBED == 4) //10k thermistor

#define NUMTEMPS_4 20
const short temptable_4[NUMTEMPS_4][2] = {
   { (ADC_VREF_4*1) / 0xFFF, 430},
   { (ADC_VREF_4*54) / 0xFFF, 137},
   { (ADC_VREF_4*107) / 0xFFF, 107},
   { (ADC_VREF_4*160) / 0xFFF, 91},
   { (ADC_VREF_4*213) / 0xFFF, 80},
   { (ADC_VREF_4*266) / 0xFFF, 71},
   { (ADC_VREF_4*319) / 0xFFF, 64},
   { (ADC_VREF_4*372) / 0xFFF, 57},
   { (ADC_VREF_4*425) / 0xFFF, 51},
   { (ADC_VREF_4*478) / 0xFFF, 46},
   { (ADC_VREF_4*531) / 0xFFF, 41},
   { (ADC_VREF_4*584) / 0xFFF, 35},
   { (ADC_VREF_4*637) / 0xFFF, 30},
   { (ADC_VREF_4*690) / 0xFFF, 25},
   { (ADC_VREF_4*743) / 0xFFF, 20},
   { (ADC_VREF_4*796) / 0xFFF, 14},
   { (ADC_VREF_4*849) / 0xFFF, 7},
   { (ADC_VREF_4*902) / 0xFFF, 0},
   { (ADC_VREF_4*955) / 0xFFF, -11},
   { (ADC_VREF_4*1008) / 0xFFF, -35}
};
#endif

#if (THERMISTORHEATER == 5) || (THERMISTORBED == 5) //100k ParCan thermistor (104GT-2)

#define NUMTEMPS_5 61
const short temptable_5[NUMTEMPS_5][2] = {
{ (ADC_VREF_4*1) / 0xFFF, 713},
{ (ADC_VREF_4*18) / 0xFFF, 316},
{ (ADC_VREF_4*35) / 0xFFF, 266},
{ (ADC_VREF_4*52) / 0xFFF, 239},
{ (ADC_VREF_4*69) / 0xFFF, 221},
{ (ADC_VREF_4*86) / 0xFFF, 208},
{ (ADC_VREF_4*103) / 0xFFF, 197},
{ (ADC_VREF_4*120) / 0xFFF, 188},
{ (ADC_VREF_4*137) / 0xFFF, 181},
{ (ADC_VREF_4*154) / 0xFFF, 174},
{ (ADC_VREF_4*171) / 0xFFF, 169},
{ (ADC_VREF_4*188) / 0xFFF, 163},
{ (ADC_VREF_4*205) / 0xFFF, 159},
{ (ADC_VREF_4*222) / 0xFFF, 154},
{ (ADC_VREF_4*239) / 0xFFF, 150},
{ (ADC_VREF_4*256) / 0xFFF, 147},
{ (ADC_VREF_4*273) / 0xFFF, 143},
{ (ADC_VREF_4*290) / 0xFFF, 140},
{ (ADC_VREF_4*307) / 0xFFF, 136},
{ (ADC_VREF_4*324) / 0xFFF, 133},
{ (ADC_VREF_4*341) / 0xFFF, 130},
{ (ADC_VREF_4*358) / 0xFFF, 128},
{ (ADC_VREF_4*375) / 0xFFF, 125},
{ (ADC_VREF_4*392) / 0xFFF, 122},
{ (ADC_VREF_4*409) / 0xFFF, 120},
{ (ADC_VREF_4*426) / 0xFFF, 117},
{ (ADC_VREF_4*443) / 0xFFF, 115},
{ (ADC_VREF_4*460) / 0xFFF, 112},
{ (ADC_VREF_4*477) / 0xFFF, 110},
{ (ADC_VREF_4*494) / 0xFFF, 108},
{ (ADC_VREF_4*511) / 0xFFF, 106},
{ (ADC_VREF_4*528) / 0xFFF, 103},
{ (ADC_VREF_4*545) / 0xFFF, 101},
{ (ADC_VREF_4*562) / 0xFFF, 99},
{ (ADC_VREF_4*579) / 0xFFF, 97},
{ (ADC_VREF_4*596) / 0xFFF, 95},
{ (ADC_VREF_4*613) / 0xFFF, 92},
{ (ADC_VREF_4*630) / 0xFFF, 90},
{ (ADC_VREF_4*647) / 0xFFF, 88},
{ (ADC_VREF_4*664) / 0xFFF, 86},
{ (ADC_VREF_4*681) / 0xFFF, 84},
{ (ADC_VREF_4*698) / 0xFFF, 81},
{ (ADC_VREF_4*715) / 0xFFF, 79},
{ (ADC_VREF_4*732) / 0xFFF, 77},
{ (ADC_VREF_4*749) / 0xFFF, 75},
{ (ADC_VREF_4*766) / 0xFFF, 72},
{ (ADC_VREF_4*783) / 0xFFF, 70},
{ (ADC_VREF_4*800) / 0xFFF, 67},
{ (ADC_VREF_4*817) / 0xFFF, 64},
{ (ADC_VREF_4*834) / 0xFFF, 61},
{ (ADC_VREF_4*851) / 0xFFF, 58},
{ (ADC_VREF_4*868) / 0xFFF, 55},
{ (ADC_VREF_4*885) / 0xFFF, 52},
{ (ADC_VREF_4*902) / 0xFFF, 48},
{ (ADC_VREF_4*919) / 0xFFF, 44},
{ (ADC_VREF_4*936) / 0xFFF, 40},
{ (ADC_VREF_4*953) / 0xFFF, 34},
{ (ADC_VREF_4*970) / 0xFFF, 28},
{ (ADC_VREF_4*987) / 0xFFF, 20},
{ (ADC_VREF_4*1004) / 0xFFF, 8},
{ (ADC_VREF_4*1021) / 0xFFF, 0}
};
#endif

#if (THERMISTORHEATER == 6) || (THERMISTORBED == 6) // 100k Epcos thermistor
#define NUMTEMPS_6 36
const short temptable_6[NUMTEMPS_6][2] = {
   { (ADC_VREF_4*28 / 0xFFF	, 250},
   { (ADC_VREF_4*31 / 0xFFF	, 245},
   { (ADC_VREF_4*35 / 0xFFF	, 240},
   { (ADC_VREF_4*39 / 0xFFF	, 235},
   { (ADC_VREF_4*42 / 0xFFF	, 230},
   { (ADC_VREF_4*44 / 0xFFF	, 225},
   { (ADC_VREF_4*49 / 0xFFF	, 220},
   { (ADC_VREF_4*53 / 0xFFF	, 215},
   { (ADC_VREF_4*62 / 0xFFF	, 210},
   { (ADC_VREF_4*71 / 0xFFF	, 205}, //fitted graphically
   { (ADC_VREF_4*78 / 0xFFF	, 200}, //fitted graphically
   { (ADC_VREF_4*94 / 0xFFF	, 190},
   { (ADC_VREF_4*102 / 0xFFF	, 185},
   { (ADC_VREF_4*116 / 0xFFF	, 170},
   { (ADC_VREF_4*143 / 0xFFF	, 160},
   { (ADC_VREF_4*183 / 0xFFF	, 150},
   { (ADC_VREF_4*223 / 0xFFF	, 140},
   { (ADC_VREF_4*270 / 0xFFF	, 130},
   { (ADC_VREF_4*318 / 0xFFF	, 120},
   { (ADC_VREF_4*383 / 0xFFF	, 110},
   { (ADC_VREF_4*413 / 0xFFF	, 105},
   { (ADC_VREF_4*439 / 0xFFF	, 100},
   { (ADC_VREF_4*484 / 0xFFF	, 95},
   { (ADC_VREF_4*513 / 0xFFF	, 90},
   { (ADC_VREF_4*607 / 0xFFF	, 80},
   { (ADC_VREF_4*664 / 0xFFF	, 70},
   { (ADC_VREF_4*781 / 0xFFF	, 60},
   { (ADC_VREF_4*810 / 0xFFF	, 55},
   { (ADC_VREF_4*849 / 0xFFF	, 50},
   { (ADC_VREF_4*914 / 0xFFF	, 45},
   { (ADC_VREF_4*914 / 0xFFF	, 40},
   { (ADC_VREF_4*935 / 0xFFF	, 35},
   { (ADC_VREF_4*954 / 0xFFF	, 30},
   { (ADC_VREF_4*970 / 0xFFF	, 25},
   { (ADC_VREF_4*978 / 0xFFF	, 22},
   { (ADC_VREF_4*1008 / 0xFFF	, 3}
};
#endif

#if (THERMISTORHEATER == 7) || (THERMISTORBED == 7) // 100k Honeywell 135-104LAG-J01
#define NUMTEMPS_7 55
const short temptable_7[NUMTEMPS_7][2] = {
   { (ADC_VREF_4*46 / 0xFFF	, 270},
   { (ADC_VREF_4*50 / 0xFFF	, 265},
   { (ADC_VREF_4*54 / 0xFFF	, 260},
   { (ADC_VREF_4*58 / 0xFFF	, 255},
   { (ADC_VREF_4*62 / 0xFFF	, 250},
   { (ADC_VREF_4*67 / 0xFFF	, 245},
   { (ADC_VREF_4*72 / 0xFFF	, 240},
   { (ADC_VREF_4*79 / 0xFFF	, 235},
   { (ADC_VREF_4*85 / 0xFFF	, 230},
   { (ADC_VREF_4*91 / 0xFFF	, 225},
   { (ADC_VREF_4*99 / 0xFFF	, 220},
   { (ADC_VREF_4*107 / 0xFFF	, 215},
   { (ADC_VREF_4*116 / 0xFFF	, 210},
   { (ADC_VREF_4*126 / 0xFFF	, 205},
   { (ADC_VREF_4*136 / 0xFFF	, 200},
   { (ADC_VREF_4*149 / 0xFFF	, 195},
   { (ADC_VREF_4*160 / 0xFFF	, 190},
   { (ADC_VREF_4*175 / 0xFFF	, 185},
   { (ADC_VREF_4*191 / 0xFFF	, 180},
   { (ADC_VREF_4*209 / 0xFFF	, 175},
   { (ADC_VREF_4*224 / 0xFFF	, 170},
   { (ADC_VREF_4*246 / 0xFFF	, 165},
   { (ADC_VREF_4*267 / 0xFFF	, 160},
   { (ADC_VREF_4*293 / 0xFFF	, 155},
   { (ADC_VREF_4*316 / 0xFFF	, 150},
   { (ADC_VREF_4*340 / 0xFFF	, 145},
   { (ADC_VREF_4*364 / 0xFFF	, 140},
   { (ADC_VREF_4*396 / 0xFFF	, 135},
   { (ADC_VREF_4*425 / 0xFFF	, 130},
   { (ADC_VREF_4*460 / 0xFFF	, 125},
   { (ADC_VREF_4*489 / 0xFFF	, 120},
   { (ADC_VREF_4*526 / 0xFFF	, 115},
   { (ADC_VREF_4*558 / 0xFFF	, 110},
   { (ADC_VREF_4*591 / 0xFFF	, 105},
   { (ADC_VREF_4*628 / 0xFFF	, 100},
   { (ADC_VREF_4*660 / 0xFFF	, 95},
   { (ADC_VREF_4*696 / 0xFFF	, 90},
   { (ADC_VREF_4*733 / 0xFFF	, 85},
   { (ADC_VREF_4*761 / 0xFFF	, 80},
   { (ADC_VREF_4*794 / 0xFFF	, 75},
   { (ADC_VREF_4*819 / 0xFFF	, 70},
   { (ADC_VREF_4*847 / 0xFFF	, 65},
   { (ADC_VREF_4*870 / 0xFFF	, 60},
   { (ADC_VREF_4*892 / 0xFFF	, 55},
   { (ADC_VREF_4*911 / 0xFFF	, 50},
   { (ADC_VREF_4*929 / 0xFFF	, 45},
   { (ADC_VREF_4*944 / 0xFFF	, 40},
   { (ADC_VREF_4*959 / 0xFFF	, 35},
   { (ADC_VREF_4*971 / 0xFFF	, 30},
   { (ADC_VREF_4*981 / 0xFFF	, 25},
   { (ADC_VREF_4*989 / 0xFFF	, 20},
   { (ADC_VREF_4*994 / 0xFFF	, 15},
   { (ADC_VREF_4*1001 / 0xFFF	, 10},
   { (ADC_VREF_4*1005 / 0xFFF	, 5},
   { (ADC_VREF_4*1021 / 0xFFF	, 0} //safety

};
#endif



#if THERMISTORHEATER == 1
#define NUMTEMPS NUMTEMPS_1
#define temptable temptable_1
#elif THERMISTORHEATER == 2
#define NUMTEMPS NUMTEMPS_2
#define temptable temptable_2
#elif THERMISTORHEATER == 3
#define NUMTEMPS NUMTEMPS_3
#define temptable temptable_3
#elif THERMISTORHEATER == 4
#define NUMTEMPS NUMTEMPS_4
#define temptable temptable_4
#elif THERMISTORHEATER == 5
#define NUMTEMPS NUMTEMPS_5
#define temptable temptable_5
#elif THERMISTORHEATER == 6
#define NUMTEMPS NUMTEMPS_6
#define temptable temptable_6
#elif THERMISTORHEATER == 7
#define NUMTEMPS NUMTEMPS_7
#define temptable temptable_7
#elif defined HEATER_USES_THERMISTOR
#error No heater thermistor table specified
#endif
#if THERMISTORBED == 1
#define BNUMTEMPS NUMTEMPS_1
#define bedtemptable temptable_1
#elif THERMISTORBED == 2
#define BNUMTEMPS NUMTEMPS_2
#define bedtemptable temptable_2
#elif THERMISTORBED == 3
#define BNUMTEMPS NUMTEMPS_3
#define bedtemptable temptable_3
#elif THERMISTORBED == 4
#define BNUMTEMPS NUMTEMPS_4
#define bedtemptable temptable_4
#elif THERMISTORBED == 5
#define BNUMTEMPS NUMTEMPS_5
#define bedtemptable temptable_5
#elif THERMISTORBED == 6
#define BNUMTEMPS NUMTEMPS_6
#define bedtemptable temptable_6
#elif THERMISTORBED == 7
#define BNUMTEMPS NUMTEMPS_7
#define bedtemptable temptable_7
#elif defined BED_USES_THERMISTOR
#error No bed thermistor table specified
#endif

#endif //THERMISTORTABLES_H_
