/PROG PICK_FURNACE1
/ATTR
/MN
   1: !All furnace positions are relative to UF2 ;
   2: UFRAME_NUM=2 ;
   3: UTOOL_NUM=1 ;
   4: !start motion ;
   5: CALL GRIPPER_OPEN ;
   6: !approach furnace ;
   7: J PR[4:furnace_approach] 100% CNT100 ;
   8: L PR[2:furnace_1] 4000mm/sec FINE Offset,PR[27:aprchFrnOffs] ;
   9: !move to part location accounting for push distance ;
  10: L PR[2:furnace_1] 750mm/sec FINE Offset,PR[24:pickoffset] Tool_Offset,PR[30:push_dist1] ;
  11: CALL GRIPPER_CLOSE ;
  12: !extract part with remove offset plus push distance ;
  13: L PR[2:furnace_1] 1200mm/sec FINE Offset,PR[25:rmvPart] Tool_Offset,PR[30:push_dist1] ;
  14: !move back to approach position ;
  15: J PR[4:furnace_approach] 100% CNT100 ;
/POS
/END
