/PROG PICK_FURNACE
/ATTR
/MN
   1:  !Called with AR[1] 1 or 2 ;
   2:   ;
   3:  !AR[1]: ;
   4:  !Left Furnace = 1 ;
   5:  !Right Furnace = 2 ;
   6:   ;
   7:  !All furnace positions are relative to UF2 ;
   8:  UFRAME_NUM=2 ;
   9:  UTOOL_NUM=1 ;
  10:   ;
  11:  !start motion ;
  12:  CALL GRIPPER_OPEN ;
  13:  !approach furnace ;
  14:  J PR[4:furnace_approach] 100% CNT100 ;
  15:   ;
  16:  !Set active furnace ;
  17:  IF (AR[1]=1) THEN ;
  18:       PR[55:activeFurnace]=PR[2:furnace_1] ;
  19:  ELSE ;
  20:       IF (AR[1]=2) THEN ;
  21:           PR[55:activeFurnace]=PR[3:furnace_2] ;
  22:       ELSE ;
  23:           MESSAGE[AR[1] undefined] ;
  24:  ENDIF ;
  25:  ENDIF ;
  26:   ;
  27:  L PR[55:activeFurnace] 2000mm/sec FINE Offset,PR[27:aprchFrnOffs] ;
  28:   ;
  29:  !move to part ;
  30:  !CHANGE THIS MOTION INSTRUCTION ;
  31:  L PR[55:activeFurnace] 750mm/sec FINE Offset,PR[24:pickoffset] Tool_Offset,PR[30:push_dist1] ;
  32:  CALL GRIPPER_CLOSE ;
  33:   ;
  34:  !extract part for regrip;
  35:  L PR[55:activeFurnace] 1200mm/sec FINE Offset,PR[25:rmvPart] Tool_Offset,PR[30:push_dist1] ;
  36:   ;
  37:  !Regrip part ;
  38:   ;
  39:  !move back to approach position ;
  40:  J PR[4:furnace_approach] 100% CNT100 ;
/POS
/END
