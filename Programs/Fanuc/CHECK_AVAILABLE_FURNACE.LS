/PROG CHECK_AVAILABLE_FURNACE
/ATTR
/MN
: !Find available furnace for loading;
: R[25:AVAILABLE_FURNACE]=0 ;
: !Check if first load or timer expired;
: IF (R[43:LAST_FURN_LOADED]=0) THEN  ;
:   !First load - no timer check;
:   IF (R[22:FURNACE1_STATE]=0) THEN  ;
:     R[25:AVAILABLE_FURNACE]=1 ;
:   ELSE  ;
:     IF (R[23:FURNACE2_STATE]=0) THEN  ;
:       R[25:AVAILABLE_FURNACE]=2 ;
:     ENDIF ;
:   ENDIF ;
: ELSE  ;
:   !Subsequent loads - check timer;
:   IF (TIMER[4]>=R[42:MIN_LOAD_DELAY]) THEN  ;
:     IF (R[22:FURNACE1_STATE]=0) THEN  ;
:       R[25:AVAILABLE_FURNACE]=1 ;
:     ELSE  ;
:       IF (R[23:FURNACE2_STATE]=0) THEN  ;
:         R[25:AVAILABLE_FURNACE]=2 ;
:       ENDIF ;
:     ENDIF ;
:   ELSE  ;
:     !Timer not expired - block load;
:     R[25:AVAILABLE_FURNACE]=0 ;
:   ENDIF ;
: ENDIF ;
: END ;
/POS
/END
