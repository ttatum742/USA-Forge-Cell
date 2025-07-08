/PROG  GRID_SCAN
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Grid pattern die scan";
PROG_SIZE	= 600;
CREATE		= DATE 25-07-01  TIME 10:00:00;
MODIFIED	= DATE 25-07-01  TIME 10:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 35;
MEMORY_SIZE	= 1000;
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
   1:  !Grid scan subroutine ;
   2:  !Grid parameters ;
   3:  R[97:grid_size]=9 ;
   4:  R[98:step_size]=25 ;
   5:  R[99:start_offset]=-100 ;
   6:  R[92:total_points]=(R[97:grid_size]*R[97:grid_size]) ;
   7:  R[91:point_counter]=0 ;
   8:   ;
   9:  !Move to scan start position ;
  10:J PR[10:die_center] 50% FINE Offset,PR[20:scan_offset]    ;
  11:   ;
  12:  !Grid scan loop ;
  13:  FOR R[102:row]=0 TO (R[97:grid_size]-1) ;
  14:  FOR R[103:col]=0 TO (R[97:grid_size]-1) ;
  15:   ;
  16:  !Calculate grid position ;
  17:  R[104:x_pos]=(R[99:start_offset]+(R[103:col]*R[98:step_size])) ;
  18:  R[105:y_pos]=(R[99:start_offset]+(R[102:row]*R[98:step_size])) ;
  19:   ;
  20:  !Move to measurement point ;
  21:  PR[11:scan_point,1]=R[104:x_pos] ;
  22:  PR[11:scan_point,2]=R[105:y_pos] ;
  23:L PR[11:scan_point] 200mm/sec FINE    ;
  24:   ;
  25:  !Send position to PC ;
  26:  R[88:current_x]=R[104:x_pos] ;
  27:  R[89:current_y]=R[105:y_pos] ;
  28:  R[90:current_z]=PR[11,3:scan_point] ;
  29:  R[91:point_counter]=(R[91:point_counter]+1) ;
  30:   ;
  31:  !Wait for PC to process measurement ;
  32:  WAIT    .1(sec) ;
  33:   ;
  34:  ENDFOR ;
  35:  ENDFOR ;
/POS
/END
