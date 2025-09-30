/PROG  MAIN_DUAL
/ATTR
OWNER   = MNEDITOR;
COMMENT   = "Dual furnace forge cell";
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
  11: !R[22] = Furnace 1 state ;
  12: !(0=empty, 1=loading, 2=heating, 3=ready);
  13: !R[23] = Furnace 2 state ;
  14: !(0=empty, 1=loading, 2=heating, 3=ready);
  15: !R[24] = Priority furnace ;
  16: !(1 or 2, which is ready first);
  17: !R[25] = Available furnace ;
  18: !(1 or 2, which can accept part);
  19:   ;
  20: !Digital I/O mapping (placeholders for now):;
  21: !DI[2] = Part @ press ready ;
  22: !DI[4] = Furnace 1 done signal;
  23: !DI[5] = Furnace 2 done signal;
  24: !DI[6] = Press ready signal;
  25: !DO[18] = Part in press;
  26:   ;
  27:   ;
  28: !Inits;
  29: CALL INIT_VARS  ;
  30:   !add home routine here;
  31:   ;
  32: !Main processing loop;
  33: LBL[1]  ;
  34: FOR R[6:rowCount]=1 TO 3  ;
  35:   FOR R[7:colCount]=1 TO 16  ;
  36:     !Check if any furnace is ready for pickup (priority);
  37:     CALL CHECK_READY_FURNACE ;
  38:     IF (R[24:PRIORITY_FURNACE]>0) THEN  ;
  39:       !Process ready furnace first to minimize heat loss;
  40:       CALL PROCESS_READY_FURNACE ;
  41:     ELSE  ;
  42:     ENDIF ;
  43:   ;
  44:     !Load next part if furnace available;
  45:     CALL CHECK_AVAILABLE_FURNACE ;
  46:     IF (R[25:AVAILABLE_FURNACE]>0) THEN  ;
  47:       CALL CALC_POS ;
  48:       CALL PICK_PART  ;
  49:       CALL LOAD_FURNACE ;
  50:     ELSE  ;
  51:       !No furnace available, wait for one to finish;
  52:       CALL WAIT_FOR_FURNACE ;
  53:       CALL PROCESS_READY_FURNACE ;
  54:       !Try loading again;
  55:       CALL CHECK_AVAILABLE_FURNACE ;
  56:       IF (R[25:AVAILABLE_FURNACE]>0) THEN  ;
  57:         CALL CALC_POS ;
  58:         CALL PICK_PART  ;
  59:         CALL LOAD_FURNACE ;
  60:       ELSE ;
  61:       ENDIF ;
  62:     ENDIF ;
  63:   ;
  64:     !Parts remaining check;
  65:     IF (R[5:PARTS_REMAINING]=0) THEN  ;
  66:       JMP LBL[2] ;
  67:     ELSE ;
  68:     ENDIF ;
  69:   ENDFOR  ;
  70: ENDFOR  ;
  71:   ;
  72: !Process any remaining parts in furnaces;
  73: LBL[2] ;
  74: CALL FINISH_REMAINING_PARTS ;
  75:   ;
  76: LBL[99] ;
  77: END ;
/POS
/END
