/PROG  OUTPUT_PART
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 301;
CREATE		= DATE 25-06-16  TIME 11:35:34;
MODIFIED	= DATE 25-06-16  TIME 11:35:34;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 11;
MEMORY_SIZE	= 637;
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
   1:J PR[100:zeros] 100% CNT100    ;
   2:J PR[7:output] 100% CNT100 Offset,PR[20:zoffset150]    ;
   3:L PR[7:output] 500mm/sec FINE    ;
   4:  CALL GRIPPER_OPEN    ;
   5:   ;
   6:   ;
   7:   ;
   8:   ;
   9:   ;
  10:   ;
  11:   ;
/POS
/END
