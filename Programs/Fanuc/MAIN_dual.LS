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
   4: !using GI's ;
   5:   ;
   6: !R[1] = Part Length;
   7: !R[2] = Rod diameter;
   8: !R[3] = Head type;
   9: !R[4] = RunQty;
  10: !R[12] = backstop setting;
  11:   ;
  12: !furnace status registers:;
  13: !R[22] = Furnace 1 state ;
  14: !(0=empty, 1=loading, 2=heating, 3=ready);
  15: !R[23] = Furnace 2 state ;
  16: !(0=empty, 1=loading, 2=heating, 3=ready);
  17: !R[24] = Priority furnace ;
  18: !(1 or 2, which is ready first);
  19: !R[25] = Available furnace ;
  20: !(1 or 2, which can accept part);
  21:   ;
  22: !Digital I/O mapping ;
  23: !DI[10] = Part @ press ready ;
  24: !DI[13] = Furnace 1 done signal;
  25: !DI[14] = Furnace 2 done signal;
  26: !DI[xx] = Press ready signal;
  27: !DO[25] = Part in press;
  28:   ;
  29:   ;
  30: !Inits;
  31: CALL INIT_VARS  ;
  32:   !add home routine here;
  33:   ;
  34: !Main processing loop;
  35: LBL[1]  ;
  36: FOR R[6:rowCount]=1 TO 3  ;
  37:   FOR R[7:colCount]=1 TO 16  ;
  38:     !Check if any furnace is ready for pickup (priority);
  39:     CALL CHECK_READY_FURNACE ;
  40:     IF (R[24:PRIORITY_FURNACE]>0) THEN  ;
  41:       !Process ready furnace first to minimize heat loss;
  42:       CALL PROCESS_READY_FURNACE ;
  43:     ELSE  ;
  44:     ENDIF ;
  45:   ;
  46:     !Load next part if furnace available;
  47:     CALL CHECK_AVAILABLE_FURNACE ;
  48:     IF (R[25:AVAILABLE_FURNACE]>0) THEN  ;
  49:       CALL CALC_POS ;
  50:       CALL PICK_PART  ;
  51:       CALL LOAD_FURNACE ;
  52:     ELSE  ;
  53:       !No furnace available, wait for one to finish;
  54:       CALL WAIT_FOR_FURNACE ;
  55:       CALL PROCESS_READY_FURNACE ;
  56:       !Try loading again;
  57:       CALL CHECK_AVAILABLE_FURNACE ;
  58:       IF (R[25:AVAILABLE_FURNACE]>0) THEN  ;
  59:         CALL CALC_POS ;
  60:         CALL PICK_PART  ;
  61:         CALL LOAD_FURNACE ;
  62:       ELSE ;
  63:       ENDIF ;
  64:     ENDIF ;
  65:   ;
  66:     !Parts remaining check;
  67:     IF (R[5:PARTS_REMAINING]=0) THEN  ;
  68:       JMP LBL[2] ;
  69:     ELSE ;
  70:     ENDIF ;
  71:   ENDFOR  ;
  72: ENDFOR  ;
  73:   ;
  74: !Process any remaining parts in furnaces;
  75: LBL[2] ;
  76: CALL FINISH_REMAINING_PARTS ;
  77:   ;
  78: LBL[99] ;
  79: END ;
/POS
/END
