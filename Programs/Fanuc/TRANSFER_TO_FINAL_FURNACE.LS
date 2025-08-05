/PROG TRANSFER_TO_FINAL_FURNACE
/ATTR
OWNER   = MNEDITOR;
COMMENT   = "Transfer part from preheat to final temp furnace";
PROG_SIZE = 200;
CREATE    = DATE 25-08-05  TIME 12:00:00;
MODIFIED  = DATE 25-08-05  TIME 12:00:00;
FILE_NAME = ;
VERSION   = 0;
LINE_COUNT  = 0;
MEMORY_SIZE = 500;
PROTECT   = read_WRITE;
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
   1: !Transfer preheated part from furnace 1 to furnace 2;
   2: !All furnace positions are relative to UF2;
   3: UFRAME_NUM=2 ;
   4: UTOOL_NUM=1 ;
   5:   ;
   6: !Pick from preheat furnace using existing subroutine;
   7: CALL PICK_FURNACE1 ;
   8:   ;
   9: !Place in final temp furnace using existing subroutine;
  10: CALL PLACE_FURNACE2 ;
  11:   ;
  12: !Push part to backstop in furnace 2 using force control;
  13: CALL PUSHTOBACKSTOP(2) ;
  14:   ;
  15: !Update furnace states;
  16: R[22:FURNACE1_STATE]=0 ;
  17: R[23:FURNACE2_STATE]=1 ;
  18: !Start final temp heating;
  19: DO[3:furnace2_start]=ON ;
  20: WAIT 0.5(sec) ;
  21: DO[3:furnace2_start]=OFF ;
  22: R[23:FURNACE2_STATE]=2 ;
  23:   ;
/POS
/END