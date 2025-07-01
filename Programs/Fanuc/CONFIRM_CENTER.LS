/PROG  CONFIRM_CENTER
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Confirm die center location";
PROG_SIZE	= 500;
CREATE		= DATE 25-07-01  TIME 10:00:00;
MODIFIED	= DATE 25-07-01  TIME 10:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 25;
MEMORY_SIZE	= 900;
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
   1:  !Confirmation scan around predicted center ;
   2:  !PC provides estimated center in R[157], R[158] ;
   3:   ;
   4:  !Confirmation parameters ;
   5:  R[173:confirm_radius]=40 ;
   6:  R[174:confirm_points]=8 ;
   7:  R[156:total_points]=R[174:confirm_points] ;
   8:  R[155:point_counter]=0 ;
   9:   ;
  10:  !Radial confirmation scan ;
  11:  FOR R[179:point]=0 TO (R[174:confirm_points]-1) ;
  12:   ;
  13:  !Calculate angle for this point ;
  14:  R[180:angle]=(R[179:point]*45) ;
  15:   ;
  16:  !Calculate position relative to estimated center ;
  17:  R[152:current_x]=(R[157:center_x]+R[173:confirm_radius]*COS(R[180:angle])) ;
  18:  R[153:current_y]=(R[158:center_y]+R[173:confirm_radius]*SIN(R[180:angle])) ;
  19:   ;
  20:  !Move to confirmation point ;
  21:  PR[12:confirm_point,1]=R[152:current_x] ;
  22:  PR[12:confirm_point,2]=R[153:current_y] ;
  23:L PR[12:confirm_point] 200mm/sec FINE    ;
  24:   ;
  25:  !Update position registers and counter ;
  26:  R[154:current_z]=PR[12,3:confirm_point] ;
  27:  R[155:point_counter]=(R[155:point_counter]+1) ;
  28:   ;
  29:  !Wait for PC to process ;
  30:  WAIT    .1(sec) ;
  31:   ;
  32:  ENDFOR ;
/POS
/END
