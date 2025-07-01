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
   5:  R[330:die_face_count]=0 ;
   6:  R[331:die_face_sum]=0 ;
   7:  R[332:reference_face_height]=50 ;
   8:   ;
   9:  !Calculate die face height from zone 4 points ;
  10:  FOR R[333:i]=0 TO (R[300:zone_map_points]-1) ;
  11:  IF R[322+R[333]*3]=4 THEN
  12:  !Zone 4 = die face measurements ;
  13:  R[334:point_x]=R[320+R[333]*3] ;
  14:  R[335:point_y]=R[321+R[333]*3] ;
  15:   ;
  16:  !Move to remeasure this point ;
  17:  PR[11,1:scan_point]=R[334] ;
  18:  PR[11,2:scan_point]=R[335] ;
  19:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[303:scan_z_offset]) ;
  20:L PR[11:scan_point] 300mm/sec CNT10    ;
  21:  CALL MEASURE_POINT    ;
  22:   ;
  23:  IF R[400:measurement]>0 THEN
  24:  R[331]=(R[331]+R[400]) ;
  25:  R[330]=(R[330]+1) ;
  26:  ENDIF ;
  27:  ENDIF ;
  28:  ENDFOR ;
  29:   ;
  30:  !Set reference face height ;
  31:  IF R[330]>0 THEN
  32:  R[332]=(R[331]/R[330]) ;
  33:  !Adjust Z for optimal scanning ;
  34:  R[336:optimal_z_offset]=(R[332]-25) ;
  35:  ELSE
  36:  R[336]=-25 ;
  37:  ENDIF ;
  38:   ;
  39:  !Initialize well edge detection ;
  40:  R[340:well_edge_count]=0 ;
  41:  R[341:num_directions]=8 ;
  42:   ;
  43:  !Search in 8 directions for well edges ;
  44:  FOR R[342:direction]=0 TO 7 ;
  45:  R[343:angle]=(R[342]*45) ;
  46:  R[344:found_well_edge]=0 ;
  47:  R[345:min_search]=3 ;
  48:  R[346:max_search]=25 ;
  49:   ;
  50:  !Binary search for well-to-face transition ;
  51:  WHILE (R[346]-R[345])>1 AND R[344]=0 ;
  52:  R[347:test_radius]=((R[345]+R[346])/2) ;
  53:  R[348:test_x]=(R[347]*COS(R[343])) ;
  54:  R[349:test_y]=(R[347]*SIN(R[343])) ;
  55:   ;
  56:  !Move to test position ;
  57:  PR[11,1:scan_point]=R[348] ;
  58:  PR[11,2:scan_point]=R[349] ;
  59:  PR[11,3:scan_point]=(PR[10,3:die_center]+R[336]) ;
  60:L PR[11:scan_point] 350mm/sec CNT5    ;
  61:  CALL MEASURE_POINT    ;
  62:   ;
  63:  IF R[400:measurement]<0 THEN
  64:  !Still in well - search further out ;
  65:  R[345]=R[347] ;
  66:  ELSE
  67:  !Got reading - check if die face height ;
  68:  R[350:height_diff]=ABS(R[400]-R[332]) ;
  69:   ;
  70:  IF R[350]<3 THEN
  71:  !Found well edge! ;
  72:  R[351+R[340]*2]=R[348] ;
  73:  R[352+R[340]*2]=R[349] ;
  74:  R[340]=(R[340]+1) ;
  75:  R[344]=1 ;
  76:  ELSE
  77:  !Wrong height - search closer ;
  78:  R[346]=R[347] ;
  79:  ENDIF ;
  80:  ENDIF ;
  81:  ENDWHILE ;
  82:  ENDFOR ;
  83:   ;
  84:  !Signal PC completion of well detection ;
  85:  R[151:status]=20 ;
/POS
/END