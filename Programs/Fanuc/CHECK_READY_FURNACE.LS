/PROG CHECK_READY_FURNACE
/ATTR
/MN
: !Check which furnace is ready;
: R[24:PRIORITY_FURNACE]=0 ;
: !Check furnace 1;
: IF (R[22:FURNACE1_STATE]=3) THEN  ;
:   R[24:PRIORITY_FURNACE]=1 ;
: !Check furnace 2 if furnace 1 not ready;
: ELSE  ;
:   IF (R[23:FURNACE2_STATE]=3) THEN  ;
:     R[24:PRIORITY_FURNACE]=2 ;
:   ENDIF ;
: ENDIF ;
: END ;
/POS
/END
