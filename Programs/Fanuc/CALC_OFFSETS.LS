/PROG  CALC_OFFSETS
/ATTR
OWNER		= ASCBIN;
COMMENT		= "";
PROG_SIZE	= 420;
CREATE		= DATE 25-06-26  TIME 12:34:02;
MODIFIED	= DATE 25-06-27  TIME 12:38:24;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 14;
MEMORY_SIZE	= 736;
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
/MN
   1:  !Radial offset piecewise ;
   2:  IF (R[2:rod_diameter]<=R[20:critical_dia]) THEN ;
   3:  R[16:radial_offset]=(R[2:rod_diameter]*.5) ;
   4:  ELSE ;
   5:  R[16:radial_offset]=(R[2:rod_diameter]/1.414) ;
   6:  ENDIF ;
   7:   ;
   8:  !Jig height adjustment ;
   9:  R[16:radial_offset]=(R[16:radial_offset]-R[19:jig_height]) ;
  10:   ;
  11:  !Length offset ;
  12:   ;
  13:  !MISC ;
  14:   ;
/POS
/END
