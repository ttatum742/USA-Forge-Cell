/PROG PICK_OFFS
/ATTR
/MN
   1:  ! R[100] through R[163] ;
   2:  ! store offsets for a ref dia ;
   3:  ! R[1] = current row (1-4) ;
   4:  ! R[2] = current column (1-16) ;
   5:  ! R[3] = part diameter ;
   6:  ! R[x] = reference diameter ;
   7:   ;
   8:  ! Calculate which offset to use: ;
   9:  R[99] = (R[6]-1)*16 + R[7] + 99  ! Index into R[100-163] ;
  10:   ;
  11:  ! Access the stored offset ;
  12:  R[98] = R[R[99]]  ;
  13:   ;
  14:   ;
  15:   ;
  16:  ! Apply R[98] as offset ;
/POS
/END
