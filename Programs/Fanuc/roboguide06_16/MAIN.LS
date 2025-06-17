/PROG  MAIN
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 1373;
CREATE		= DATE 25-06-16  TIME 11:40:30;
MODIFIED	= DATE 25-06-16  TIME 11:40:30;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 54;
MEMORY_SIZE	= 1885;
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
   1:  !Main forge cell program ;
   2:   ;
   3:  !PLC writes these registers: ;
   4:  !R[1] = Part Length ;
   5:  !R[2] = Rod diameter ;
   6:  !R[3] = Head type ;
   7:  !R[4] = RunQty ;
   8:   ;
   9:  !PC writes these registers: ;
  10:  !R[80-85] = joint torque derivati ;
  11:   ;
  12:  !PLC controls these digital input ;
  13:  !DI[1] = Furnace done signal ;
  14:  !DI[2] = Part ready signal ;
  15:  !DI[4] = Furnace on signal ;
  16:   ;
  17:  CALL INIT_VARS    ;
  18:  CALL SAFETY_CHECK    ;
  19:  CALL HOME    ;
  20:  CALL CONNECTION_TEST    ;
  21:  LBL[1] ;
  22:  FOR R[6:rowCount]=1 TO 8 ;
  23:  FOR R[7:colCount]=1 TO 8 ;
  24:  CALL CALC_POS    ;
  25:  CALL PICK_PART    ;
  26:   ;
  27:  CALL PLACE_FURNACE    ;
  28:  CALL PUSHTOBACKSTOP    ;
  29:  !Issue part ready signal to PLC ;
  30:  DO[1]=ON ;
  31:  !Wait for PLC acknowledgement ;
  32:  WAIT DI[4]=ON    ;
  33:  !Wait for furnace done signal fro ;
  34:  WAIT DI[1]=ON    ;
  35:  CALL PICK_FURNACE    ;
  36:   ;
  37:  CALL PLACE_FORGE    ;
  38:  !Wait for forge ready signal from ;
  39:  WAIT DI[2]=ON    ;
  40:  CALL PICK_FORGE    ;
  41:  CALL OUTPUT_PART    ;
  42:  !Increment part count ;
  43:  R[5:PARTS_REMAINING]=(R[5:PARTS_REMAINING]-1) ;
  44:   ;
  45:  !Parts remaining check ;
  46:  IF (R[5:PARTS_REMAINING]=0) THEN ;
  47:  JMP LBL[99] ;
  48:   ;
  49:  ELSE ;
  50:  ENDIF ;
  51:  ENDFOR ;
  52:  ENDFOR ;
  53:  LBL[99] ;
  54:  END ;
/POS
/END
