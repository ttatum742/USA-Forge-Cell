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
   6: !STEP 1: Pick from preheat furnace (furnace 1);
   7: CALL GRIPPER_OPEN ;
   8: !approach furnace 1;
   9: J PR[4:furnace_approach] 100% CNT100 ;
  10: L PR[2:furnace_1] 4000mm/sec FINE Offset,PR[27:aprchFrnOffs] ;
  11: !move to part location accounting for push distance;
  12: L PR[2:furnace_1] 750mm/sec FINE Offset,PR[24:pickoffset] Tool_Offset,PR[30:push_dist1] ;
  13: CALL GRIPPER_CLOSE ;
  14: !extract part with remove offset plus push distance;
  15: L PR[2:furnace_1] 1200mm/sec FINE Offset,PR[25:rmvPart] Tool_Offset,PR[30:push_dist1] ;
  16: !move back to approach position;
  17: J PR[4:furnace_approach] 100% CNT100 ;
  18:   ;
  19: !STEP 2: Place in final temp furnace (furnace 2);
  20: !reset backoff offset;
  21: PR[29,3:backoff_pshStrt]=R[15:part_gripped] ;
  22: !approach furnace 2;
  23: J PR[4:furnace_approach] 100% CNT100 ;
  24: L PR[3:furnace_2] 4000mm/sec FINE Offset,PR[27:aprchFrnOffs] ;
  25: L PR[3:furnace_2] 750mm/sec FINE Offset,PR[26:placeoffset] ;
  26: CALL GRIPPER_OPEN ;
  27: L PR[3:furnace_2] 1200mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  28: PR[5:pushstart]=LPOS ;
  29: !move backoff position to push start position;
  30: PR[29,3:backoff_pshStrt]=(PR[29,3:backoff_pshStrt]+10) ;
  31: L PR[3:furnace_2] 4000mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  32: CALL GRIPPER_CLOSE ;
  33:   ;
  34: !STEP 3: Update furnace states;
  35: R[22:FURNACE1_STATE]=0 ;
  36: R[23:FURNACE2_STATE]=1 ;
  37: !Start final temp heating;
  38: DO[3:furnace2_start]=ON ;
  39: WAIT 0.5(sec) ;
  40: DO[3:furnace2_start]=OFF ;
  41: R[23:FURNACE2_STATE]=2 ;
  42:   ;
/POS
/END