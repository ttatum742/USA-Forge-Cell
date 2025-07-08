/PROG  CLASSIFY_MEASUREMENT
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Classify measurement into zones";
PROG_SIZE	= 600;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 45;
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
   1:  !Classify measurement into zones ;
   2:  !R[140] = measurement result ;
   3:  !R[88] = current X position ;
   4:  !R[89] = current Y position ;
   5:   ;
   6:  !Calculate distance from center ;
   7:  R[103:dx]=R[88] ;
   8:  R[104:dy]=R[89] ;
   9:  R[105:dist_from_center]=SQRT(R[103]*R[103]+R[104]*R[104]) ;
  10:   ;
  11:  !Classify based on measurement result ;
  12:  IF R[140:measurement]<0 THEN
  13:  !Unmeasurable - determine type ;
  14:  IF R[105]<15 THEN
  15:  R[106:zone_type]=1 ;
  16:  ELSE
  17:  IF R[105]>35 THEN
  18:  R[106:zone_type]=3 ;
  19:  ELSE
  20:  R[106:zone_type]=2 ;
  21:  ENDIF ;
  22:  ENDIF ;
  23:  ELSE
  24:  !Valid measurement ;
  25:  R[107:height_category]=TRUNC(R[140]/10) ;
  26:  IF R[107]>=4 AND R[107]<=8 THEN
  27:  R[106:zone_type]=4 ;
  28:  !Accumulate die face data ;
  29:  R[121:die_face_sum]=(R[121]+R[140]) ;
  30:  R[120:die_face_count]=(R[120]+1) ;
  31:  ELSE
  32:  R[106:zone_type]=5 ;
  33:  ENDIF ;
  34:  ENDIF ;
  35:   ;
  36:  !Store classification ;
  37:  R[119+R[119]*3]=R[88] ;
  38:  R[120+R[119]*3]=R[89] ;
  39:  R[121+R[119]*3]=R[106] ;
  40:   ;
  41:  !Increment zone map counter ;
  42:  R[119:zone_map_points]=(R[119]+1) ;
  43:   ;
  44:  !Zone types: 1=central well, 2=transition ;
  45:  !3=outer edge, 4=die face, 5=other valid ;
/POS
/END