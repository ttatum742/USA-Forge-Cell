/PROG  MAIN
/ATTR
OWNER   = MNEDITOR;
COMMENT   = "preheater furnace forge cell";
PROG_SIZE = 200;
CREATE    = DATE 25-06-17  TIME 12:00:00;
MODIFIED  = DATE 25-06-17  TIME 12:00:00;
FILE_NAME = ;
VERSION   = 0;
LINE_COUNT  = 0;
MEMORY_SIZE = 500;
PROTECT   = READ_WRITE;
TCD:  STACK_SIZE  = 0,
      TASK_PRIORITY = 50,
      TIME_SLICE  = 0,
      BUSY_LAMP_OFF = 0,
      ABORT_REQUEST = 0,
      PAUSE_REQUEST = 0;
DEFAULT_GROUP = 1,*,*,*,*;
CONTROL_CODE  = 00000000 00000000;
LOCAL_REGISTERS = 0,0,0;
/APPL
AUTO_SINGULARITY_HEADER;
  ENABLE_SINGULARITY_AVOIDANCE   : TRUE;
/MN
   1: !Dual furnace forge cell program;
   2:   ;
   3: !PLC writes these registers:;
   4: !R[1] = Part Length;
   5: !R[2] = Rod diameter;
   6: !R[3] = Head type;
   7: !R[4] = RunQty;
   8: !R[12] = backstop setting;
   9:   ;
  10: !furnace status registers:;
  11: !R[22] = Furnace 1 state (0=empty, 1=loading, 2=heating, 3=ready);
  12: !R[23] = Furnace 2 state (0=empty, 1=loading, 2=heating, 3=ready);
  13: !R[24] = Priority furnace (1 or 2, which is ready first);
  14: !R[25] = Available furnace (1 or 2, which can accept part);
  15:   ;
  16: !Digital I/O mapping (placeholders for now):;
  17: !DI[2] = Part ready signal;
  18: !DI[4] = Furnace 1 done signal;
  19: !DI[5] = Furnace 2 done signal;
  20: !DI[6] = Press ready signal;
  21: !DO[1] = Part ready signal to PLC;
  22: !DO[2] = Furnace 1 start signal;
  23: !DO[3] = Furnace 2 start signal;
  24:   ;
  25: !Inits;
  26: CALL INIT_VARS  ;
  27: CALL SAFETY_CHECK ;
  28: CALL HOME ;
  29: CALL CONNECTION_TEST ;
  30:   ;
  31: !Main processing loop;
  32: LBL[1]  ;
  33: FOR R[6:rowCount]=1 TO 4  ;
  34:   FOR R[7:colCount]=1 TO 16  ;
  35:   ;
  36:   ;
  37:   ;
  38:   ENDFOR  ;
  39: ENDFOR  ;
  40:   ;
  41: !Process any remaining parts in furnaces;
  42: LBL[2] ;
  43: CALL FINISH_REMAINING_PARTS ;
  44:   ;
  45: LBL[99] ;
  46: END ;
/POS
/END
