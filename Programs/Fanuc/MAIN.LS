/PROG  MAIN
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
: !Dual furnace forge cell program;
:;
: !PLC writes these registers:;
: !R[1] = Part Length;
: !R[2] = Rod diameter;
: !R[3] = Head type;
: !R[4] = RunQty;
:;
: !New furnace status registers:;
:!UPDATE THESE!!!!;
: !R[10] = Furnace 1 state (0=empty, 1=loading, 2=heating, 3=ready);
: !R[11] = Furnace 2 state (0=empty, 1=loading, 2=heating, 3=ready);
: !R[14] = Priority furnace (1 or 2, which is ready first);
: !R[15] = Available furnace (1 or 2, which can accept part);
:;
: !Digital I/O mapping:;
: !DI[2] = Part ready signal;
: !DI[4] = Furnace 1 done signal;
: !DI[5] = Furnace 2 done signal;
: !DI[6] = Press ready signal;
: !DO[1] = Part ready signal to PLC;
: !DO[2] = Furnace 1 start signal;
: !DO[3] = Furnace 2 start signal;
:;
: CALL INIT_VARS  ;
: CALL SAFETY_CHECK ;
: CALL HOME ;
: CALL CONNECTION_TEST  ;
:;
: !Main processing loop;
: LBL[1]  ;
: FOR R[6:rowCount]=1 TO 4  ;
:   FOR R[7:colCount]=1 TO 16  ;
:     !Check if any furnace is ready for pickup (priority);
:     CALL CHECK_READY_FURNACE ;
:     IF (R[14:PRIORITY_FURNACE]>0) THEN  ;
:       !Process ready furnace first to minimize heat loss;
:       CALL PROCESS_READY_FURNACE ;
:     ENDIF ;
:;
:     !Load next part if furnace available;
:     CALL CHECK_AVAILABLE_FURNACE ;
:     IF (R[15:AVAILABLE_FURNACE]>0) THEN  ;
:       CALL CALC_POS ;
:       CALL PICK_PART  ;
:       CALL LOAD_FURNACE ;
:     ELSE  ;
:       !No furnace available, wait for one to finish;
:       CALL WAIT_FOR_FURNACE ;
:       CALL PROCESS_READY_FURNACE ;
:       !Try loading again;
:       CALL CHECK_AVAILABLE_FURNACE ;
:       IF (R[15:AVAILABLE_FURNACE]>0) THEN  ;
:         CALL CALC_POS ;
:         CALL PICK_PART  ;
:         CALL LOAD_FURNACE ;
:       ENDIF ;
:     ENDIF ;
:;
:     !Parts remaining check;
:     IF (R[5:PARTS_REMAINING]=0) THEN  ;
:       JMP LBL[2] ;
:     ENDIF ;
:   ENDFOR  ;
: ENDFOR  ;
:;
: !Process any remaining parts in furnaces;
: LBL[2] ;
: CALL FINISH_REMAINING_PARTS ;
:;
: LBL[99] ;
: END ;
/POS
/END