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
   4:  R[420:central_well_count]=0 ;
   5:  R[421:die_face_count]=R[301] ;
   6:  R[422:transition_count]=0 ;
   7:  R[423:outer_edge_count]=0 ;
   8:   ;
   9:  !Count each zone type ;
  10:  FOR R[424:i]=0 TO (R[300]-1) ;
  11:  SELECT R[322+R[424]*3]=1,R[420]=(R[420]+1) ;
  12:         R[322+R[424]*3]=2,R[422]=(R[422]+1) ;
  13:         R[322+R[424]*3]=3,R[423]=(R[423]+1) ;
  14:  ENDFOR ;
  15:   ;
  16:  !Calculate average die face height ;
  17:  IF R[421]>0 THEN
  18:  R[425:avg_die_face_height]=(R[302]/R[421]) ;
  19:  !Set optimal Z offset ;
  20:  R[303:scan_z_offset]=(R[425]-25) ;
  21:  ELSE
  22:  !No die face found - use default ;
  23:  R[303:scan_z_offset]=-25 ;
  24:  ENDIF ;
  25:   ;
  26:  !Estimate well center from zone 1 points ;
  27:  R[426:well_center_x]=0 ;
  28:  R[427:well_center_y]=0 ;
  29:  IF R[420]>0 THEN
  30:  FOR R[428:i]=0 TO (R[300]-1) ;
  31:  IF R[322+R[428]*3]=1 THEN
  32:  R[426]=(R[426]+R[320+R[428]*3]) ;
  33:  R[427]=(R[427]+R[321+R[428]*3]) ;
  34:  ENDIF ;
  35:  ENDFOR ;
  36:  R[426]=(R[426]/R[420]) ;
  37:  R[427]=(R[427]/R[420]) ;
  38:   ;
  39:  !Adjust die center position to well center ;
  40:  PR[10,1:die_center]=(PR[10,1]+R[426]) ;
  41:  PR[10,2:die_center]=(PR[10,2]+R[427]) ;
  42:  ENDIF ;
  43:   ;
  44:  !Send zone analysis to PC ;
  45:  R[430:zone_analysis_complete]=1 ;
/POS
/END