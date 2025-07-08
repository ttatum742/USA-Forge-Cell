/PROG  ANALYZE_ZONE_MAP
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Analyze zone mapping results";
PROG_SIZE	= 500;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 35;
MEMORY_SIZE	= 700;
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
   1:  !Analyze zone mapping results ;
   2:   ;
   3:  !Count zones ;
   4:  R[108:central_well_count]=0 ;
   5:  R[109:die_face_count]=R[120] ;
   6:  R[110:transition_count]=0 ;
   7:  R[111:outer_edge_count]=0 ;
   8:   ;
   9:  !Count each zone type ;
  10:  FOR R[112:i]=0 TO (R[119]-1) ;
  11:  SELECT R[121+R[112]*3]=1,R[108]=(R[108]+1) ;
  12:         R[121+R[112]*3]=2,R[110]=(R[110]+1) ;
  13:         R[121+R[112]*3]=3,R[111]=(R[111]+1) ;
  14:  ENDFOR ;
  15:   ;
  16:  !Calculate average die face height ;
  17:  IF R[109]>0 THEN
  18:  R[113:avg_die_face_height]=(R[121]/R[109]) ;
  19:  !Set optimal Z offset ;
  20:  R[122:scan_z_offset]=(R[113]-25) ;
  21:  ELSE
  22:  !No die face found - use default ;
  23:  R[122:scan_z_offset]=-25 ;
  24:  ENDIF ;
  25:   ;
  26:  !Estimate well center from zone 1 points ;
  27:  R[114:well_center_x]=0 ;
  28:  R[115:well_center_y]=0 ;
  29:  IF R[108]>0 THEN
  30:  FOR R[116:i]=0 TO (R[119]-1) ;
  31:  IF R[121+R[116]*3]=1 THEN
  32:  R[114]=(R[114]+R[119+R[116]*3]) ;
  33:  R[115]=(R[115]+R[120+R[116]*3]) ;
  34:  ENDIF ;
  35:  ENDFOR ;
  36:  R[114]=(R[114]/R[108]) ;
  37:  R[115]=(R[115]/R[108]) ;
  38:   ;
  39:  !Adjust die center position to well center ;
  40:  PR[10,1:die_center]=(PR[10,1]+R[114]) ;
  41:  PR[10,2:die_center]=(PR[10,2]+R[115]) ;
  42:  ENDIF ;
  43:   ;
  44:  !Send zone analysis to PC ;
  45:  R[117:zone_analysis_complete]=1 ;
/POS
/END