4/PROG PICK_FURNACE
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
   9:  UTOOL_NUM=2 ;
  10:   ;
  11:  !start motion ;
  12:  CALL GRIPPER_OPEN ;
  13:  !approach furnace ;
  14:  J PR[4:furnace_approach] 100% CNT100 ;
  15:   ;
  16:  !Set active furnace ;
  17:  IF (AR[1]=1) THEN ;
  18:       PR[55:activeFurnace]=PR[2:furnace_1] ;
  19:       PR[56:activeDrag]=PR[18:furn1_drag] ;
  20:       PR[57:activePick]=PR[16:furn1_pick] ;
  21:  ELSE ;
  22:       IF (AR[1]=2) THEN ;
  23:           PR[55:activeFurnace]=PR[3:furnace_2] ;
  24:           PR[56:activeDrag]=PR[19:furn2_drag] ;
  25:           PR[57:activePick]=PR[17:furn2_pick] ;
  26:       ELSE ;
  27:           MESSAGE[AR[1] undefined] ;
  28:       ENDIF ;
  29:  ENDIF ;
  30:   ;
  31:  L PR[55:activeFurnace] 1000mm/sec FINE Offset,PR[27:aprchFrnOffs] ;
  32:   ;
  33:  !move to part ;
  34:  !CHANGE THIS MOTION INSTRUCTION ;
  35:  L PR[56:activeDrag] 750mm/sec FINE  ;
  36:  CALL GRIPPER_CLOSE ;
  37:  PAYLOAD[2] ;
  38:   ;
  39:  !extract part for regrip;
  40:  L PR[56:activeDrag] 500mm/sec FINE Offset,PR[21:zoffset235]  ;
  41:  CALL GRIPPER_OPEN ;
  42:  PAYLOAD[1] ;
  43:   ;
  44:  !Regrip part ;
  45:  L PR[57:activePick] 750mm/sec FINE ;
  46:  CALL GRIPPER_CLOSE ;
  47:  PAYLOAD[2] ;
  48:   ;
  49:  !move back to approach position ;
  50:  J PR[4:furnace_approach] 70% CNT100 ;
/POS
/END
