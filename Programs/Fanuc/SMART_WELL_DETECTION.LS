/PROG  SMART_WELL_DETECTION
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Phase 2: Smart well edge detection";
PROG_SIZE	= 1200;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 85;
MEMORY_SIZE	= 1800;
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
   1:  !Phase 2: Smart Well Edge Detection ;
   2:  !Use zone info to find central well edges ;
   3:   ;
   4:  !Calculate die face reference height ;
   5:  R[125:die_face_count]=0 ;
   6:  R[126:die_face_sum]=0 ;
   7:  R[127:reference_face_height]=50 ;
   8:   ;
   9:  !Calculate die face height from zone 4 points ;
  10:  FOR R[128:i]=0 TO (R[119:zone_map_points]-1) ;
  11:  IF R[131+R[128]*3]=4 THEN
  12:  !Zone 4 = die face measurements ;
  13:  R[129:point_x]=R[119+R[128]*3] ;
  14:  R[130:point_y]=R[120+R[128]*3] ;
  15:   ;
  16:  !Move to remeasure this point ;
  17:  PR[11,1:scan_point]=R[129] ;
  18:  PR[11,2:scan_point]=R[130] ;
  19:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[122:scan_z_offset]) ;
  20:L PR[11:scan_point] 300mm/sec CNT10    ;
  21:  CALL MEASURE_POINT    ;
  22:   ;
  23:  IF R[140:measurement]>0 THEN
  24:  R[126]=(R[126]+R[140]) ;
  25:  R[125]=(R[125]+1) ;
  26:  ENDIF ;
  27:  ENDIF ;
  28:  ENDFOR ;
  29:   ;
  30:  !Set reference face height ;
  31:  IF R[125]>0 THEN
  32:  R[127]=(R[126]/R[125]) ;
  33:  !Adjust Z for optimal scanning ;
  34:  R[131:optimal_z_offset]=(R[127]-25) ;
  35:  ELSE
  36:  R[131]=-25 ;
  37:  ENDIF ;
  38:   ;
  39:  !Initialize well edge detection ;
  40:  R[137:well_edge_count]=0 ;
  41:  R[138:num_directions]=8 ;
  42:   ;
  43:  !Search in 8 directions for well edges ;
  44:  FOR R[139:direction]=0 TO 7 ;
  45:  R[140:angle]=(R[139]*45) ;
  46:  R[141:found_well_edge]=0 ;
  47:  R[142:min_search]=3 ;
  48:  R[143:max_search]=25 ;
  49:   ;
  50:  !Binary search for well-to-face transition ;
  51:  WHILE (R[143]-R[142])>1 AND R[141]=0 ;
  52:  R[144:test_radius]=((R[142]+R[143])/2) ;
  53:  R[145:test_x]=(R[144]*COS(R[140])) ;
  54:  R[146:test_y]=(R[144]*SIN(R[140])) ;
  55:   ;
  56:  !Move to test position ;
  57:  PR[11,1:scan_point]=R[145] ;
  58:  PR[11,2:scan_point]=R[146] ;
  59:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[131]) ;
  60:L PR[11:scan_point] 350mm/sec CNT5    ;
  61:  CALL MEASURE_POINT    ;
  62:   ;
  63:  IF R[140:measurement]<0 THEN
  64:  !Still in well - search further out ;
  65:  R[142]=R[144] ;
  66:  ELSE
  67:  !Got reading - check if die face height ;
  68:  R[147:height_diff]=ABS(R[140]-R[127]) ;
  69:   ;
  70:  IF R[147]<3 THEN
  71:  !Found well edge! ;
  72:  R[148+R[137]*2]=R[145] ;
  73:  R[149+R[137]*2]=R[146] ;
  74:  R[137]=(R[137]+1) ;
  75:  R[141]=1 ;
  76:  ELSE
  77:  !Wrong height - search closer ;
  78:  R[143]=R[144] ;
  79:  ENDIF ;
  80:  ENDIF ;
  81:  ENDWHILE ;
  82:  ENDFOR ;
  83:   ;
  84:  !Signal PC completion of well detection ;
  85:  R[87:status]=20 ;
/POS
/END