/PROG PROCESS_READY_FURNACE
/ATTR
/MN
: !Process the ready furnace immediately;
: IF (R[24:PRIORITY_FURNACE]=1) THEN  ;
:   !Process furnace 1;
:   CALL PICK_FURNACE1 ;
:   CALL PLACE_FORGE  ;
:   WAIT DI[6:PRESS_READY]=ON  ;
:   CALL PICK_FORGE ;
:   CALL OUTPUT_PART  ;
:   R[22:FURNACE1_STATE]=0 ;
:   DO[2:FURN1_START]=OFF ;
:   CALL INCREMENT_COUNT  ;
: ELSE  ;
:   IF (R[24:PRIORITY_FURNACE]=2) THEN  ;
:     !Process furnace 2;
:     CALL PICK_FURNACE2 ;
:     CALL PLACE_FORGE  ;
:     WAIT DI[6:PRESS_READY]=ON  ;
:     CALL PICK_FORGE ;
:     CALL OUTPUT_PART  ;
:     R[23:FURNACE2_STATE]=0 ;
:     DO[3:FURN2_START]=OFF ;
:     CALL INCREMENT_COUNT  ;
:   ENDIF ;
: ENDIF ;
: R[24:PRIORITY_FURNACE]=0 ;
: END ;
/POS
/END
