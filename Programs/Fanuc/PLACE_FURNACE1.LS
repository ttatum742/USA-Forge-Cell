/PROG PLACE_FURNACE1
/ATTR
/MN
   1: !All furnace positions are relative to UF2 ;
   2: UFRAME_NUM=2 ;
   3: UTOOL_NUM=1 ;
   4: !reset backoff offset ;
   5: PR[29,3:backoff_pshStrt]=R[15:part_gripped] ;
   6: PR[29,2:backoff_pshStrt]=R[16:radial_offset] ;
   7: PR[29,5:backoff_pshStrt]=0 ;
   8:  ;
   9: !approach furnace ;
  10: J PR[4:furnace_approach] 100% CNT100 ;
  11: L PR[2:furnace_1] 1250mm/sec CNT20 Offset,PR[27:aprchFrnOffs] ;
  12: L PR[2:furnace_1] 750mm/sec FINE Offset,PR[26:placeoffset] ;
  13: CALL GRIPPER_OPEN ;
  14:   ;
  15:  !add rotational offset to avoid fixture ;
  16:  PR[29,5:backoff_pshStrt]=33 ;
  17:   ;
  18:  !add +y adjustment ;
  19:  PR[29,2:backoff_pshStrt]=(PR[29,2:backoff_pshStrt]+20) ;
  20:   ;
  21: L PR[2:furnace_1] 1000mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  22:  !store pushstart ;
  23: PR[5:pushstart]=LPOS ;
  24:   ;
  25: !move backoff position to push start position ;
  26: PR[29,3:backoff_pshStrt]=(PR[29,3:backoff_pshStrt]+25) ;
  27: L PR[2:furnace_1] 2000mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  28: CALL GRIPPER_CLOSE ;
/POS
/END
