/PROG PLACE_FORGE
/ATTR
/MN
   1:   !All forge positions are relative to UF4 ;
   2:  UFRAME_NUM=4 ;
   3:  UTOOL_NUM=5 ;
   4:   ;
   5:   ;
   6:   ;
   7:   ;
   8:   ;
   9:  J PR[13:forgeApproach] 100% FINE ;
  10:  L PR[14:forge_place] 750mm/sec FINE Offset,PR[32:above_frg];
  11:  L PR[14:forge_place] 500mm/sec FINE Offset,PR[34:frgPlace_ofs] ;
  12:  CALL GRIPPER_OPEN ;
  13:  L PR[14:forge_place] 750mm/sec FINE Offset,PR[32:above_frg] ;
  14:  L PR[13:forgeApproach] 2000mm/sec FINE ;
  15:   ;
  16:   ;
  17:   ;
/POS
/END
