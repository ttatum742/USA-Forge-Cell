/PROG PICK_FURNACE2
/ATTR
/MN
   1: !All furnace positions are relative to UF2 ;
   2: UFRAME_NUM=2 ;
   3: UTOOL_NUM=1 ;
   4: !calculate last push distance;
   5: PR[30,3:push_distance]=(PR[6,3:pushend]-PR[5,3:pushstart]) ;
   6: !start motion ;
   7: CALL GRIPPER_OPEN ;
   8: !approach furnace ;
   9: J PR[4:furnace_approach] 100% CNT100 ;
  10: L PR[3:furnace_2] 4000mm/sec FINE Offset,PR[27:aprchFrnOffs] ;
  11: !move to part location accounting for push distance ;
  12: L PR[3:furnace_2] 750mm/sec FINE Offset,PR[24:pickoffset] Tool_Offset,PR[30:push_distance] ;
  13: CALL GRIPPER_CLOSE ;
  14: !extract part with remove offset plus push distance ;
  15: L PR[3:furnace_2] 1200mm/sec FINE Offset,PR[25:rmvPart] Tool_Offset,PR[30:push_distance] ;
  16: !move back to approach position ;
  17: J PR[4:furnace_approach] 100% CNT100 ;
/POS
/END
