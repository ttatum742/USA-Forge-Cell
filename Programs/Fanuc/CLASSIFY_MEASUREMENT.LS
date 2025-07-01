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
   2:  !R[400] = measurement result ;
   3:  !R[152] = current X position ;
   4:  !R[153] = current Y position ;
   5:   ;
   6:  !Calculate distance from center ;
   7:  R[410:dx]=R[152] ;
   8:  R[411:dy]=R[153] ;
   9:  R[412:dist_from_center]=SQRT(R[410]*R[410]+R[411]*R[411]) ;
  10:   ;
  11:  !Classify based on measurement result ;
  12:  IF R[400:measurement]<0 THEN
  13:  !Unmeasurable - determine type ;
  14:  IF R[412]<15 THEN
  15:  R[413:zone_type]=1 ;
  16:  ELSE
  17:  IF R[412]>35 THEN
  18:  R[413:zone_type]=3 ;
  19:  ELSE
  20:  R[413:zone_type]=2 ;
  21:  ENDIF ;
  22:  ENDIF ;
  23:  ELSE
  24:  !Valid measurement ;
  25:  R[414:height_category]=TRUNC(R[400]/10) ;
  26:  IF R[414]>=4 AND R[414]<=8 THEN
  27:  R[413:zone_type]=4 ;
  28:  !Accumulate die face data ;
  29:  R[302:die_face_sum]=(R[302]+R[400]) ;
  30:  R[301:die_face_count]=(R[301]+1) ;
  31:  ELSE
  32:  R[413:zone_type]=5 ;
  33:  ENDIF ;
  34:  ENDIF ;
  35:   ;
  36:  !Store classification ;
  37:  R[320+R[300]*3]=R[152] ;
  38:  R[321+R[300]*3]=R[153] ;
  39:  R[322+R[300]*3]=R[413] ;
  40:   ;
  41:  !Increment zone map counter ;
  42:  R[300:zone_map_points]=(R[300]+1) ;
  43:   ;
  44:  !Zone types: 1=central well, 2=transition ;
  45:  !3=outer edge, 4=die face, 5=other valid ;
/POS
/END