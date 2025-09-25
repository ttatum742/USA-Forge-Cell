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
   3:  SKIP CONDITION R[54:j5torque]>=26    ;
   4:   ;
   5:L PR[5:pushstart] 75mm/sec FINE Offset,PR[35] Skip,LBL[1],PR[6:pushend]=LPOS    ;
   6:L PR[5:pushstart] 500mm/sec FINE    ;
   7:  CALL GRIPPER_OPEN    ;
   8:  IF (AR[1]=1) THEN ;
   9:   PR[30,3:push_dist1]=(PR[5,3:pushstart]-PR[6,3:pushend]) ;
  10:  ELSE ;
  11:   IF (AR[1]=2) THEN ;
  12:     PR[31,3:push_dist2]=(PR[5,3:pushstart]-PR[6,3:pushend]) ;
  13:   ELSE ;
  14:     MESSAGE[furnace arg undefined] ;
  15:   ENDIF ;
  16:  ENDIF ;
  17:  JMP LBL[99] ;
  18:  LBL[1] ;
  19:  MESSAGE[overtravel err] ;
  20:  ABORT ;
  21:  LBL[99] ;
/POS
/END
