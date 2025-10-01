/PROG PICK_FORGE
/ATTR
/MN
   1:   !All forge positions are relative to UF4 ;
   2:  !UTOOL DEPENDENT ON HEAD TYPE ;
   3:  UFRAME_NUM=4 ;
   4:  IF (R[3:headType]=1) THEN ;
   5:     UTOOL_NUM=UTOOLNUM FOR FLATS!!!! ;
   6:  ELSE ;
   7:     IF(R[3:headType]=2) THEN ;
   8:         UTOOL_NUM=UTOOLNUM FOR SMALL FLATS!!!! ;
   9:     ELSE ;
  10:   !add new head type here ;
  11:         ENDIF ;
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
  22:  L PR[14:forge_pick] 2000mm/sec CNT50 Offset,PR[20:zoffset150] ;
  23:  CALL GRIPPER_OPEN ;
  24:  L PR[14:forge_pick] 450mm/sec FINE ;
  25:  CALL GRIPPER_CLOSE ;
  26:  L PR[14:forge_pick] 1200mm/sec FINE Offset,PR[32:frg_pick_ofs] ;
  27:  L PR[13:forge_aprch] 2000mm/sec CNT100 ;
/POS
/END
