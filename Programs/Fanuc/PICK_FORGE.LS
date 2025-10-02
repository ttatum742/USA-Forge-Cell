/PROG PICK_FORGE
/ATTR
/MN
   1:   !All forge positions are relative to UF4 ;
   2:  !UTOOL DEPENDENT ON HEAD TYPE ;
   3:  UFRAME_NUM=4 ;
   4:  IF (R[3:headType]=1) THEN ;
   5:     UTOOL_NUM=3 ;
   6:  ELSE ;
   7:     IF(R[3:headType]=2) THEN ;
   8:         UTOOL_NUM=6 ;
   9:     ELSE ;
  10:     !add new head type here ;
  11:     ENDIF ;
  12:  ENDIF ;
  13:   ;
  14:  ! robot should be clear ;
  15:  ! of forge at wait position ;
  16:  ! when program starts;
  17:   ; 
  18:  !mislabeling pr14 ;
  19:  !bc pick/place positions ;
  20:  !are the same but w/ ;
  21:  !different utools ;
  22:  WAIT DI[10]=ON ;
  23:  DO[25]=OFF ;
  24:  TIMER[1]=START ;
  25:  WAIT (TIMER[1]=GI[7]) ;
  26:  TIMER[1]=STOP ;
  27:  TIMER[1]=RESET ;
  28:  L PR[14] 500mm/sec CNT50 Offset,PR[20:zoffset150] ;
  29:  CALL GRIPPER_OPEN ;
  30:  L PR[14] 450mm/sec FINE ;
  31:  CALL GRIPPER_CLOSE ;
  32:  PAYLOAD[2] ;
  33:  L PR[14] 1000mm/sec FINE Offset,PR[36] ;
  34:  L PR[13:forge_aprch] 1500mm/sec CNT100 ;
  35:  !EXIT INSTRUCTIONS ;
  36:  UTOOL_NUM=2 ;
  37:  UFRAME_NUM=3 ;
  38:  J PR[15:forge_exit] 85% CNT100 ;
  39:  J PR[10:array_approach] 85% CNT100;
  40:  UTOOL_NUM=6 ;
  41:  UFRAME_NUM=0 ;
  42:   ;
/POS
/END
