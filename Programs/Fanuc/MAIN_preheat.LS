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
   1: !Preheat/Final temperature forge cell program;
   2:   ;
   3: !PLC writes these registers:;
   4: !R[1] = Part Length;
   5: !R[2] = Rod diameter;
   6: !R[3] = Head type;
   7: !R[4] = RunQty;
   8: !R[12] = backstop setting;
   9:   ;
  10: !furnace status registers:;
  11: !R[22] = Furnace 1 state (preheat furnace: 0=empty, 1=loading, 2=heating, 3=ready);
  12: !R[23] = Furnace 2 state (final temp furnace: 0=empty, 1=loading, 2=heating, 3=ready);
  13: !R[24] = Priority furnace (1 or 2, which is ready first);
  14: !R[25] = Available furnace (1 or 2, which can accept part);
  15:   ;
  16: !Digital I/O mapping (placeholders for now):;
  17: !DI[2] = Part ready signal;
  18: !DI[4] = Furnace 1 done signal (preheat complete);
  19: !DI[5] = Furnace 2 done signal (final temp ready);
  20: !DI[6] = Press ready signal;
  21: !DO[1] = Part ready signal to PLC;
  22: !DO[2] = Furnace 1 start signal (preheat);
  23: !DO[3] = Furnace 2 start signal (final temp);
  24:   ;
  25: !Inits;
  26: CALL INIT_VARS  ;
  27: CALL SAFETY_CHECK ;
  28: CALL HOME ;
  29: CALL CONNECTION_TEST ;
  30:   ;
  31: !Main processing loop - preheat system;
  32: LBL[1]  ;
  33: FOR R[6:rowCount]=1 TO 4  ;
  34:   FOR R[7:colCount]=1 TO 16  ;
  35:     !PRIORITY 1: Process final temp parts immediately (minimize heat loss);
  36:     CALL CHECK_READY_FURNACE ;
  37:     IF (R[24:PRIORITY_FURNACE]=2) THEN  ;
  38:       CALL PROCESS_READY_FURNACE ;
  39:     ENDIF ;
  40:   ;
  41:     !PRIORITY 2: Transfer preheated parts to final furnace;
  42:     IF (R[22:FURNACE1_STATE]=3 AND R[23:FURNACE2_STATE]=0) THEN  ;
  43:       CALL TRANSFER_TO_FINAL_FURNACE ;
  44:     ENDIF ;
  45:   ;
  46:     !PRIORITY 3: Keep preheat furnace loaded (maximize throughput);
  47:     IF (R[22:FURNACE1_STATE]=0) THEN  ;
  48:       CALL CALC_POS ;
  49:       CALL PICK_PART  ;
  50:       CALL LOAD_FURNACE ;
  51:     ENDIF ;
  52:   ;
  53:     !Parts remaining check;
  54:     IF (R[5:PARTS_REMAINING]=0) THEN  ;
  55:       JMP LBL[2] ;
  56:     ENDIF ;
  57:   ENDFOR  ;
  58: ENDFOR  ;
  59:   ;
  60: !Process any remaining parts in furnaces;
  61: LBL[2] ;
  62: CALL FINISH_REMAINING_PARTS ;
  63:   ;
  64: LBL[99] ;
  65: END ;
/POS
/END
