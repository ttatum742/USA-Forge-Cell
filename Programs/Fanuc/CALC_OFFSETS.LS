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
  13:  !height reduction piecewise  ;
  14:  R[39:hexHead_zofs]=((1-0.8125)*R[2:rod_diameter]);
  15:  R[40:12point_zofs]=((1-0.9818)*R[2:rod_diameter]) ;
  16:   ;
  17:  IF (R[3:headType]=1) THEN ;
  18:     R[41:curFrgpick_zofs]=R[39:hexHead_zofs] ;
  19:  ELSE ;
  20:     IF (R[3:headType]=2) THEN ;
  21:         R[41:curFrgpick_zofs]=R[40:12point_zofs] ;
  22:     ELSE ;
  23:     !add adtnl head type ;
  24:     !height diffs formula ;
  25:     !here: ;
  26:   ;
  27:     !if too many options ;
  28:     !replace w/ select logic ;
  29:     ENDIF ;
  30:   ;
  31:  ENDIF ;
  32:  PR[38,3:frg1stpickofs]=R[41:curFrgpick_zofs] ;
  33:   ;
  34:  !MISC ;
  35:   ;
/POS
/END
