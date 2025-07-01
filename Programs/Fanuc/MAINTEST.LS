/PROG  MAINTEST
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Test program - p";
PROG_SIZE	= 1029;
CREATE		= DATE 25-06-26  TIME 16:57:52;
MODIFIED	= DATE 25-06-27  TIME 12:38:24;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 39;
MEMORY_SIZE	= 1553;
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
   1:  !Test program - pick from input a ;
   2:   ;
   3:  !PLC writes these registers: ;
   4:  !R[1] = Part Length ;
   5:  !R[2] = Rod diameter ;
   6:  !R[3] = Head type ;
   7:  !R[4] = RunQty ;
   8:  !R[12] = backstop setting ;
   9:   ;
  10:  !Inits ;
  11:  !constants for testing purposes ;
  12:  R[1:part_length]=12    ;
  13:  R[2:rod_diameter]=1    ;
  14:  R[12:backstop_set]=3    ;
  15:  CALL INIT_VARS    ;
  16:  CALL SAFETY_CHECK    ;
  17:  CALL HOME    ;
  18:   ;
  19:  !Test loop - pick and output ;
  20:  LBL[1] ;
  21:  FOR R[6:rowCount]=1 TO 4 ;
  22:  FOR R[7:colCount]=1 TO 16 ;
  23:  !Calculate position in input arra ;
  24:  CALL CALC_POS    ;
  25:  !Pick part from input array ;
  26:  CALL PICK_PART    ;
  27:  !Output the part ;
  28:  CALL PLACE_FURNACE1    ;
  29:  !Parts remaining check ;
  30:  IF (R[5:PARTS_REMAINING]=0) THEN ;
  31:  JMP LBL[99] ;
  32:  ELSE ;
  33:  R[5:PARTS_REMAINING]=(R[5:PARTS_REMAINING]-1) ;
  34:  ENDIF ;
  35:  ENDFOR ;
  36:  ENDFOR ;
  37:   ;
  38:  LBL[99] ;
  39:  END ;
/POS
/END
