/PROG  ZONE_MAPPING_SCAN
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Phase 1: Map measurement zones";
PROG_SIZE	= 800;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 60;
MEMORY_SIZE	= 1200;
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
   1:  !Phase 1: Zone Mapping Scan ;
   2:  !Map measurement landscape to identify zones ;
   3:   ;
   4:  !Initialize zone mapping ;
   5:  R[119:zone_map_points]=0 ;
   6:  R[120:die_face_count]=0 ;
   7:  R[121:die_face_sum]=0 ;
   8:  R[122:scan_z_offset]=0 ;
   9:   ;
  10:  !Cross-pattern scan along X-axis ;
  11:  FOR R[123:x_scan]=-40 TO 40 STEP 20 ;
  12:  PR[11,1:scan_point]=R[123:x_scan] ;
  13:  PR[11,2:scan_point]=0 ;
  14:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[122]) ;
  15:L PR[11:scan_point] 400mm/sec CNT20    ;
  16:  CALL MEASURE_POINT    ;
  17:  CALL CLASSIFY_MEASUREMENT    ;
  18:  ENDFOR ;
  19:   ;
  20:  !Cross-pattern scan along Y-axis ;
  21:  FOR R[124:y_scan]=-40 TO 40 STEP 20 ;
  22:  PR[11,1:scan_point]=0 ;
  23:  PR[11,2:scan_point]=R[124:y_scan] ;
  24:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[122]) ;
  25:L PR[11:scan_point] 400mm/sec CNT20    ;
  26:  CALL MEASURE_POINT    ;
  27:  CALL CLASSIFY_MEASUREMENT    ;
  28:  ENDFOR ;
  29:   ;
  30:  !Diagonal corner measurements ;
  31:  PR[11,1:scan_point]=-30 ;
  32:  PR[11,2:scan_point]=-30 ;
  33:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[122]) ;
  34:L PR[11:scan_point] 400mm/sec CNT20    ;
  35:  CALL MEASURE_POINT    ;
  36:  CALL CLASSIFY_MEASUREMENT    ;
  37:   ;
  38:  PR[11,1:scan_point]=30 ;
  39:  PR[11,2:scan_point]=30 ;
  40:L PR[11:scan_point] 400mm/sec CNT20    ;
  41:  CALL MEASURE_POINT    ;
  42:  CALL CLASSIFY_MEASUREMENT    ;
  43:   ;
  44:  PR[11,1:scan_point]=30 ;
  45:  PR[11,2:scan_point]=-30 ;
  46:L PR[11:scan_point] 400mm/sec CNT20    ;
  47:  CALL MEASURE_POINT    ;
  48:  CALL CLASSIFY_MEASUREMENT    ;
  49:   ;
  50:  PR[11,1:scan_point]=-30 ;
  51:  PR[11,2:scan_point]=30 ;
  52:L PR[11:scan_point] 400mm/sec CNT20    ;
  53:  CALL MEASURE_POINT    ;
  54:  CALL CLASSIFY_MEASUREMENT    ;
  55:   ;
  56:  !Analyze zone map results ;
  57:  CALL ANALYZE_ZONE_MAP    ;
  58:   ;
  59:  !Signal PC that zone mapping is complete ;
  60:  R[87:status]=10 ;
/POS
/END