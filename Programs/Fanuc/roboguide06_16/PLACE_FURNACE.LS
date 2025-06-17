/PROG  PLACE_FURNACE
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 401;
CREATE		= DATE 25-06-05  TIME 09:25:54;
MODIFIED	= DATE 25-06-06  TIME 08:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 15;
MEMORY_SIZE	= 729;
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
   1:L PR[2:furnace] 500mm/sec CNT100 Tool_Offset,PR[21:approach offs] Offset,PR[20:zoffset150]    ;
   2:L PR[2:furnace] 200mm/sec FINE Offset,PR[20:zoffset150]    ;
   3:L PR[2:furnace] 150mm/sec FINE    ;
   4:  CALL GRIPPER_OPEN    ;
   5:  !swap with forcepush start ;
   6:L PR[2:furnace] 200mm/sec FINE Tool_Offset,PR[21:approach offs]    ;
   7:  CALL GRIPPER_CLOSE    ;
   8:   ;
   9:   ;
  10:   ;
  11:   ;
  12:   ;
  13:   ;
  14:   ;
  15:   ;
/POS
/END
