/PROG  HOME
/ATTR
OWNER		= ASCBIN;
COMMENT		= "";
PROG_SIZE	= 210;
CREATE		= DATE 25-06-16  TIME 11:17:36;
MODIFIED	= DATE 25-06-16  TIME 11:19:48;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 3;
MEMORY_SIZE	= 570;
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
/MN
   1:  !add dcs and safety checks to thi ;
   2:  PR[100:zeros]=JPOS-JPOS    ;
   3:J PR[100:zeros] 20% FINE    ;
/POS
/END
