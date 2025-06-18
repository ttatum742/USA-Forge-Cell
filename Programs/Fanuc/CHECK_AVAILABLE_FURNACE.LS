/PROG CHECK_AVAILABLE_FURNACE
/ATTR
/MN
: !Find available furnace for loading;
: R[25:AVAILABLE_FURNACE]=0 ;
: !Check furnace 1 first;
: IF (R[22:FURNACE1_STATE]=0) THEN  ;
:   R[25:AVAILABLE_FURNACE]=1 ;
: !Check furnace 2;
: ELSE  ;
:   IF (R[23:FURNACE2_STATE]=0) THEN  ;
:     R[25:AVAILABLE_FURNACE]=2 ;
:   ENDIF ;
: ENDIF ;
: END ;
/POS
/END
