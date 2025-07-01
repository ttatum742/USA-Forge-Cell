/PROG PICK_PART
/ATTR
/MN
   1:   !input_base is taught to top left pick point ;
   2:   !input base is taught in UF3 (inputArray) ;
   3:   !all other points are calculated from this point ;
   4:   !input_base should be taught using the teaching jig ;
   5:   ;
   6: UTOOL_NUM=1   ;
   7: UFRAME_NUM=3  ;
   8:   ;
   9: CALL GRIPPER_OPEN ;
  10:J PR[10:array_approach] 100% CNT100 ;
  11:J PR[8:calc_pos] 100% FINE Tool_Offset,PR[23:approachOffs_t] ;
  12:   !on the fly conversion from UF3 to UF1 ;
  13: UFRAME_NUM=1  ;
  14: PR[11:pick_pos]=LPOS  ;
  15: PR[11:pick_pos]=PR[11:pick_pos]+PR[23:approachOffs_t] ;
  16:L PR[11:pick_pos] 750mm/sec CNT100 Offset,PR[22:approachOffs] ;
  17:L PR[11:pick_pos] 750mm/sec FINE Offset,PR[24:pickoffset] ;
  18: CALL GRIPPER_CLOSE ;
  19:L PR[11:pick_pos] 1000mm/sec CNT100 Offset,PR[25:rmvPart] ;
  20:   !add safe array exit motion instructions! ;
  21:L PR[11:pick_pos] 4000mm/sec CNT100 Offset,PR[25:rmvPart] Tool_Offset,PR[33:pointdown] ;
/POS
/END
