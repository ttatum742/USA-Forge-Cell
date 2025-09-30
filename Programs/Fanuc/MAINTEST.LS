/PROG  MAINTEST
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Test program - p";
PROG_SIZE	= 1496;
CREATE		= DATE 25-06-26  TIME 16:57:52;
MODIFIED	= DATE 25-09-29  TIME 09:28:22;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 46;
MEMORY_SIZE	= 2024;
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
  12:  R[1:part_length]=6.25    ;
  13:  R[2:rod_diameter]=1.125    ;
  14:  R[12:backstop_set]=7.5    ;
  15:  CALL INIT_VARS    ;
  16:  //CALL SAFETY_CHECK    ;
  17:  //CALL HOME    ;
  18:   ;
  19:  !Test loop - pick and output ;
  20:  LBL[1] ;
  21:  FOR R[6:rowCount]=1 TO 3 ;
  22:  CALL SETROW    ;
  23:  FOR R[7:colCount]=1 TO 16 ;
  24:  !Calculate position in input arra ;
  25:  CALL CALC_POS    ;
  26:  !Pick part from input array ;
  27:  CALL PICK_PART    ;
  28:  !Output the part ;
  29:  CALL PLACE_FURNACE1    ;
  30:  //CALL PUSHTOBACKSTOP(1) ;
  31:  //CALL PICK_FURNACE(1) ;
  32:  //CALL PLACE_FORGE    ;
  33:  //UFRAME_NUM=3 ;
  34:  //UTOOL_NUM=2 ;
  35:  //J P[1] 85% CNT100    ;
  36:  !Parts remaining check ;
  37:  IF (R[5:PARTS_REMAINING]=0) THEN ;
  38:  JMP LBL[99] ;
  39:  ELSE ;
  40:  R[5:PARTS_REMAINING]=(R[5:PARTS_REMAINING]-1) ;
  41:  ENDIF ;
  42:  ENDFOR ;
  43:  ENDFOR ;
  44:   ;
  45:  LBL[99] ;
  46:  END ;
/POS
P[1]{
   GP1:
	UF : F, UT : F,		CONFIG : 'N D B, 0, 0, 0',
	X = ********  mm,	Y = ********  mm,	Z = ********  mm,
	W = ******** deg,	P = ******** deg,	R = ******** deg
};
/END
