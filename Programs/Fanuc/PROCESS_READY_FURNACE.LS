/PROG PROCESS_READY_FURNACE
/ATTR
/MN
: !Process the ready furnace immediately;
: IF (R[24:PRIORITY_FURNACE]=1) THEN  ;
:   !Process furnace 1;
:   CALL PICK_FURNACE(1) ;
:   IF (DI[11:f1OT]=ON) THEN ;
:       CALL OUTPUT_PART    ;
:       CALL INCREMENT_COUNT(2) ;
:       JMP LBL[10] ;
:   ELSE ;
:   ENDIF ; 
:   CALL PLACE_FORGE  ;
:   DO[25:ready4stamp]=ON ;
:   WAIT DI[6:PRESS_READY]=ON  ;
:   DO[25:ready4stamp]=OFF ; 
:   CALL PICK_FORGE ;
:   CALL OUTPUT_PART  ;
:   R[22:FURNACE1_STATE]=0 ;
:   CALL INCREMENT_COUNT(1)  ;
:   LBL[10] ;
: ELSE  ;
:   IF (R[24:PRIORITY_FURNACE]=2) THEN  ;
:     !Process furnace 2;
:     CALL PICK_FURNACE(2) ;
:     IF (DI[12:f2OT]=ON) THEN ; 
:       CALL OUTPUT_PART    ;
:       CALL INCREMENT_COUNT(2) ;
:       JMP LBL[20] ;
:     ELSE ;
:     ENDIF ; 
:     CALL PLACE_FORGE  ;
:     DO[25:ready4stamp]=ON ;
:     WAIT DI[6:PRESS_READY]=ON  ;
:     DO[25:ready4stamp]=OFF ;
:     CALL PICK_FORGE ;
:     CALL OUTPUT_PART  ;
:     R[23:FURNACE2_STATE]=0 ;
:     CALL INCREMENT_COUNT  ;
:     LBL[20] ; 
:   ENDIF ;
: ENDIF ;
: R[24:PRIORITY_FURNACE]=0 ;
: END ;
/POS
/END
