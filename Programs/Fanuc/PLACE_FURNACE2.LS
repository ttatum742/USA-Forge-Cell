/PROG PLACE_FURNACE2
/ATTR
/MN
   1: !All furnace positions are relative to UF2 ;
   2: UFRAME_NUM=2 ;
   3: UTOOL_NUM=1 ;
   4: !reset backoff offset ;
   5: PR[29,3:backoff_pshStrt]=R[15:part_gripped] ;
   6: !approach furnace ;
   7: J PR[4:furnace_approach] 100% CNT100 ;
   8: L PR[3:furnace_2] 4000mm/sec FINE Offset,PR[27:aprchFrnOffs] ;
   9: L PR[3:furnace_2] 750mm/sec FINE Offset,PR[26:placeoffset] ;
  10: CALL GRIPPER_OPEN ;
  11: L PR[3:furnace_2] 1200mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  12: PR[5:pushstart]=LPOS ;
  13: !move backoff position to push start position ;
  14: PR[29,3:backoff_pshStrt]=(PR[29,3:backoff_pshStrt]+10) ;
  15: L PR[3:furnace_2] 4000mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  16: CALL GRIPPER_CLOSE ;
/POS
/END
