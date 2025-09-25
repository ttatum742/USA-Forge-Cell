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
   8: !approach furnace ;
   9: J PR[4:furnace_approach] 100% CNT100 ;
  10: L PR[2:furnace_1] 1250mm/sec CNT20 Offset,PR[27:aprchFrnOffs] ;
  11: L PR[2:furnace_1] 750mm/sec FINE Offset,PR[26:placeoffset] ;
  12: CALL GRIPPER_OPEN ;
  13:   ;
  14:  !add rotational offset to avoid fixture ;
  15:  PR[29,5:backoff_pshStrt]=33 ;
  16:   ;
  17:  !add +y adjustment ;
  18:  PR[29,2:backoff_pshStrt]=(PR[29,2:backoff_pshStrt]+5) ;
  19:   ;
  20: L PR[2:furnace_1] 1000mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  21:  !store pushstart ;
  22: PR[5:pushstart]=LPOS ;
  23:   ;
  24: !move backoff position to push start position ;
  25: PR[29,3:backoff_pshStrt]=(PR[29,3:backoff_pshStrt]+25) ;
  26: L PR[2:furnace_1] 2000mm/sec FINE Offset,PR[29:backoff_pshStrt] ;
  27: CALL GRIPPER_CLOSE ;
/POS
/END
