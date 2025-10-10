/PROG LOAD_FURNACE
/ATTR
/MN
: !Load part into available furnace;
: IF (R[25:AVAILABLE_FURNACE]=1) THEN  ;
:   !Load furnace 1;
:   CALL PLACE_FURNACE(1)  ;
:   CALL PUSHTOBACKSTOP(1) ;
:   R[22:FURNACE1_STATE]=2 ;
:   !Record load and start timer;
:   R[43:LAST_FURN_LOADED]=1 ;
:   TIMER[4]=RESET ;
:   TIMER[4]=START ;
: ELSE  ;
:   IF (R[25:AVAILABLE_FURNACE]=2) THEN  ;
:     !Load furnace 2;
:     CALL PLACE_FURNACE(2)  ;
:     CALL PUSHTOBACKSTOP(2) ;
:     R[23:FURNACE2_STATE]=2 ;
:     !Record load and start timer;
:     R[43:LAST_FURN_LOADED]=2 ;
:     TIMER[4]=RESET ;
:     TIMER[4]=START ;
:   ELSE ;
:   ENDIF ;
: ENDIF ;
: END ;
/POS
/END
