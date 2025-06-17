/PROG CHECK_READY_FURNACE
/ATTR
/MN
: !Check which furnace is ready first (priority);
: R[14:PRIORITY_FURNACE]=0 ;
: !Check furnace 1;
: IF (R[10:FURNACE1_STATE]=3) THEN  ;
:   R[14:PRIORITY_FURNACE]=1 ;
: !Check furnace 2 if furnace 1 not ready;
: ELSE  ;
:   IF (R[11:FURNACE2_STATE]=3) THEN  ;
:     R[14:PRIORITY_FURNACE]=2 ;
:   ENDIF ;
: ENDIF ;
: END ;
/POS
/END
