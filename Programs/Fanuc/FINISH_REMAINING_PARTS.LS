/PROG FINISH_REMAINING_PARTS
/ATTR
/MN
: !Process any parts still in furnaces;
: LBL[1] ;
: !Check if any furnaces still have parts;
: IF (R[10:FURNACE1_STATE]>0 OR R[11:FURNACE2_STATE]>0) THEN  ;
:   CALL WAIT_FOR_FURNACE ;
:   CALL PROCESS_READY_FURNACE ;
:   JMP LBL[1] ;
: ENDIF ;
: END ;
/POS
/END
