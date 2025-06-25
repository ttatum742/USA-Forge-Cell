/PROG  PUSHTOBACKSTOP
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 399;
CREATE		= DATE 25-06-05  TIME 09:26:14;
MODIFIED	= DATE 25-06-10  TIME 06:13:06;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 13;
MEMORY_SIZE	= 867;
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
   1:  CALL GRIPPER_CLOSE    ;
   2:L PR[5:pushstart] 250mm/sec CNT100    ;
   3:  SKIP CONDITION R[80:j1dt]<=(-800)    ;
   4:  //SKIP CONDITION R[91:j6d2t]>=200    ;
   5:   ;
   6:L PR[2:furnace] 20mm/sec FINE Skip,LBL[1],PR[6:pushend]=LPOS    ;
   7:L PR[5:pushstart] 500mm/sec FINE    ;
   8:  CALL GRIPPER_OPEN    ;
   9:  IF AR[1]=1 THEN ;
  10:   PR[30,3:push_dist1]=(PR[5,3:pushstart]-PR[6,3:pushend]) ;
  11:  ELSE ;
  12:   IF AR[1]=2 THEN ;
  13:     PR[31,3:push_dist2]=(PR[5,3:pushstart]-PR[6,3:pushend]) ;
  14:   ELSE ;
  15:     MESSAGE[AR1 undefined] ;
  16:   ENDIF ;
  17:  ENDIF ;
  18:  JMP LBL[99] ;
  19:  LBL[1] ;
  20:  MESSAGE[overtravel err] ;
  21:  ABORT ;
  22:  LBL[99] ;
/POS
/END
