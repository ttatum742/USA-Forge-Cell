/PROG  MEASURE_POINT
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Take measurement at current position";
PROG_SIZE	= 400;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 25;
MEMORY_SIZE	= 600;
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
   1:  !Take laser measurement at current position ;
   2:   ;
   3:  !Send current position to PC ;
   4:  R[88:current_x]=PR[11,1:scan_point] ;
   5:  R[89:current_y]=PR[11,2:scan_point] ;
   6:  R[90:current_z]=PR[11,3:scan_point] ;
   7:   ;
   8:  !Increment point counter ;
   9:  R[91:point_counter]=(R[91]+1) ;
  10:   ;
  11:  !Wait for sensor stabilization ;
  12:  WAIT    .03(sec) ;
  13:   ;
  14:  !Signal PC to take measurement ;
  15:  R[172:measure_trigger]=1 ;
  16:   ;
  17:  !Wait for PC measurement completion ;
  18:  WAIT R[173:measure_complete]=1 TIMEOUT=2000    ;
  19:   ;
  20:  !Get measurement result ;
  21:  R[140:measurement]=R[174:laser_reading] ;
  22:   ;
  23:  !Reset measurement flags ;
  24:  R[172:measure_trigger]=0 ;
  25:  R[173:measure_complete]=0 ;
/POS
/END