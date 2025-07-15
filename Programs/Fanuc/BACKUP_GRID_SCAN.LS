/PROG  BACKUP_GRID_SCAN
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Backup grid scan if edge detection fails";
PROG_SIZE	= 600;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 40;
MEMORY_SIZE	= 800;
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
   1:  !Backup grid scan if edge detection fails ;
   2:   ;
   3:  !Reduced 5x5 grid parameters ;
   4:  R[160:grid_size]=5 ;
   5:  R[161:step_size]=10 ;
   6:  R[162:start_offset]=-20 ;
   7:  R[163:total_points]=(R[160]*R[160]) ;
   8:  R[164:point_count]=0 ;
   9:   ;
  10:  !Execute compact grid scan ;
  11:  FOR R[165:row]=0 TO (R[160]-1) ;
  12:  FOR R[166:col]=0 TO (R[160]-1) ;
  13:   ;
  14:  !Calculate grid position ;
  15:  R[167:x_pos]=(R[162]+(R[166]*R[161])) ;
  16:  R[168:y_pos]=(R[162]+(R[165]*R[161])) ;
  17:   ;
  18:  !Move to measurement point ;
  19:  PR[11,1:scan_point]=R[167] ;
  20:  PR[11,2:scan_point]=R[168] ;
  21:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[121:scan_z_offset]) ;
  22:L PR[11:scan_point] 300mm/sec CNT10    ;
  23:   ;
  24:  !Take measurement ;
  25:  CALL MEASURE_POINT    ;
  26:  R[164]=(R[164]+1) ;
  27:   ;
  28:  !Send progress to PC ;
  29:  R[169:progress]=(R[164]*100/R[163]) ;
  30:  R[92:total_points]=R[163] ;
  31:   ;
  32:  ENDFOR ;
  33:  ENDFOR ;
  34:   ;
  35:  !Signal backup scan complete ;
  36:  R[87:status]=3 ;
  37:   ;
  38:  !Note: PC will analyze backup grid data ;
  39:  !using standard grid analysis methods ;
  40:  !to calculate well center ;
/POS
/END