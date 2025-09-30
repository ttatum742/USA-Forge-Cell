/PROG PICK_FORGE
/ATTR
/MN
   1:   !All forge positions are relative to UF4 ;
   2:  UFRAME_NUM=4 ;
   3:  UTOOL_NUM=2 ;
   4:   !robot should be clear of forge at wait position when program starts;
   5:   !start motion ;
   6:  L PR[14:forge_pick] 4000mm/sec CNT100 Offset,PR[20:zoffset150] ;
   7:  CALL GRIPPER_OPEN ;
   8:  L PR[14:forge_pick] 750mm/sec FINE ;
   9:  CALL GRIPPER_CLOSE ;
  10:  L PR[14:forge_pick] 1200mm/sec FINE Offset,PR[32:frg_pick_ofs] ;
  11:  L PR[13:forge_aprch] 4000mm/sec CNT100 ;
/POS
/END
