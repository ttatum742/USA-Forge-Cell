/PROG WAIT_FOR_FURNACE
/ATTR
/MN
: !Wait for any furnace to finish;
: LBL[1] ;
: !Check furnace 1;
: IF (DI[4:FURN1_DONE]=ON AND R[10:FURNACE1_STATE]=2) THEN  ;
:   R[10:FURNACE1_STATE]=3 ;
:   R[14:PRIORITY_FURNACE]=1 ;
:   JMP LBL[99] ;
: ENDIF ;
: !Check furnace 2;
: IF (DI[5:FURN2_DONE]=ON AND R[11:FURNACE2_STATE]=2) THEN  ;
:   R[11:FURNACE2_STATE]=3 ;
:   R[14:PRIORITY_FURNACE]=2 ;
:   JMP LBL[99] ;
: ENDIF ;
: !Brief pause before checking again;
: WAIT   .1(sec) ;
: JMP LBL[1] ;
: LBL[99] ;
: END ;
/POS
/END
