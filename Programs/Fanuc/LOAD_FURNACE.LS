/PROG LOAD_FURNACE
/ATTR
/MN
: !Load part into available furnace;
: IF (R[25:AVAILABLE_FURNACE]=1) THEN  ;
:   !Load furnace 1;
:   CALL PLACE_FURNACE1  ;
:   CALL PUSHTOBACKSTOP(1) ;
:   R[22:FURNACE1_STATE]=2 ;
:   DO[2:FURN1_START]=ON ;
:   DO[1:PART_READY]=ON ;
: ELSE  ;
:   IF (R[25:AVAILABLE_FURNACE]=2) THEN  ;
:     !Load furnace 2;
:     CALL PLACE_FURNACE2  ;
:     CALL PUSHTOBACKSTOP(2) ;
:     R[23:FURNACE2_STATE]=2 ;
:     DO[3:FURN2_START]=ON ;
:     DO[1:PART_READY]=ON ;
:   ENDIF ;
: ENDIF ;
: END ;
/POS
/END
