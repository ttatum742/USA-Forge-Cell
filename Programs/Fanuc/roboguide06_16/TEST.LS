/PROG  TEST
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 693;
CREATE		= DATE 25-04-22  TIME 07:56:38;
MODIFIED	= DATE 25-04-22  TIME 08:10:48;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 6;
MEMORY_SIZE	= 1173;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
LOCAL_REGISTERS	= 0,0,0;
/APPL

AUTO_SINGULARITY_HEADER;
  ENABLE_SINGULARITY_AVOIDANCE   : TRUE;
/MN
   1:  LBL[1] ;
   2:J P[1] 100% CNT100    ;
   3:J P[2] 100% CNT100    ;
   4:J P[3] 100% CNT100    ;
   5:J P[4] 100% CNT100    ;
   6:  JMP LBL[1] ;
/POS
P[1]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =   725.313  mm,	Y =   424.235  mm,	Z =   639.788  mm,
	W =  -174.502 deg,	P =   -89.999 deg,	R =    -5.498 deg
};
P[2]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -230.402  mm,	Y =   424.235  mm,	Z =   639.788  mm,
	W =  -162.340 deg,	P =   -89.999 deg,	R =    90.000 deg
};
P[3]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =  -319.300  mm,	Y =   129.235  mm,	Z =   819.073  mm,
	W =  -162.340 deg,	P =   -89.999 deg,	R =    90.000 deg
};
P[4]{
   GP1:
	UF : 0, UT : 1,		CONFIG : 'N U T, 0, 0, 0',
	X =   392.843  mm,	Y =   352.092  mm,	Z =   489.787  mm,
	W =  -162.340 deg,	P =   -89.999 deg,	R =    90.000 deg
};
/END
