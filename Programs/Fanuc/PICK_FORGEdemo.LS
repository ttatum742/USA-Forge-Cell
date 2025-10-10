/PROG PICK_FORGEdemo
/ATTR
/MN
   1:   !All forge positions are relative to UF4 ;
   2:  !UTOOL DEPENDENT ON HEAD TYPE ;
   3:  UFRAME_NUM=4 ;
   4:  UTOOL_NUM=3 ;
   5:  ! robot should be clear ;
   6:  ! of forge at wait position ;
   7:  ! when program starts;
   8:   ; 
   9:  !mislabeling pr14 ;
  10:  !bc pick/place positions ;
  11:  !are the same but w/ ;
  12:  !different utools ;
  13:  WAIT DI[10]=ON ;
  14:  DO[25]=OFF ;
  15:  TIMER[1]=START ;
  16:  WAIT (TIMER[1]=GI[7]) ;
  17:  TIMER[1]=STOP ;
  18:  TIMER[1]=RESET ;
  19:  L PR[47] 500mm/sec CNT20;
  20:  CALL GRIPPER_OPEN ;
  21:  L PR[46] 450mm/sec FINE ;
  22:  CALL GRIPPER_CLOSE ;
  23:  PAYLOAD[2] ;
  24:  L PR[47] 500mm/sec FINE ;
  25:  L PR[13:forge_aprch] 1500mm/sec CNT100 ;
  26:  !EXIT INSTRUCTIONS ;
  27:  UTOOL_NUM=2 ;
  28:  UFRAME_NUM=3 ;
  29:  J PR[15:forge_exit] 85% CNT100 ;
  30:  J PR[10:array_approach] 85% CNT100;
  31:  UTOOL_NUM=3 ;
  32:  UFRAME_NUM=0 ;
  33:   ;
/POS
/END
