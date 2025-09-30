/PROG INCREMENT_COUNT
/ATTR
/MN
   1:  ! AR[1] = 1 ;
   2:  ! = good part output ;
   3:  ! AR[1] = 2 ;
   4:  ! = bad part output ;
   5:   ;
   6:  !this program is for ;
   7:  !things happening once ;
   8:  !per processed part ;
   9:   ;
  10:  IF (AR[1]=1) THEN ;
  11:       !good part ;
  12:       R[27:goodParts]=R[27:goodParts]+1 ;
  13:       GO[3]=R[27:goodParts] ;
  14:       R[5:PARTS_REMAINING]=(R[5:PARTS_REMAINING]-1) ;
  15:   ;
  16:  ELSE ;
  17:  ENDIF ;
  18:   ;
  19:  IF (AR[1]=2) THEN ;
  20:       !bad part ;
  21:       R[28:badParts]=R[28:badParts]+1 ;
  22:       GO[4]=R[28:badParts] ;
  23:       R[5:PARTS_REMAINING]=(R[5:PARTS_REMAINING]-1) ;
  24:  ELSE ;
  25:  ENDIF ;
  26:   ;
  27:   ;
  28:   ;
/POS
/END
