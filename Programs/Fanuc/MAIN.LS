/PROG  MAIN
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 163;
CREATE		= DATE 25-06-11  TIME 15:09:22;
MODIFIED	= DATE 25-06-11  TIME 15:09:22;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 0;
MEMORY_SIZE	= 407;
PROTECT		= READ_WRITE;
TCD:  STACK_SIZE	= 0,
      TASK_PRIORITY	= 50,
      TIME_SLICE	= 0,
      BUSY_LAMP_OFF	= 0,
      ABORT_REQUEST	= 0,
      PAUSE_REQUEST	= 0;
DEFAULT_GROUP	= 1,*,*,*,*;
CONTROL_CODE	= 00000000 00000000;
LOCAL_REGISTERS	= 0,0,0;
/APPL

AUTO_SINGULARITY_HEADER;
  ENABLE_SINGULARITY_AVOIDANCE   : TRUE;
/MN
: !Main forge cell program;
:;
: !PLC writes these registers:;
: !R[1] = Part Length;
: !R[2] = Rod diameter;
: !R[3] = Head type;
: !R[4] = RunQty;
:;
: !PLC controls these digital inputs:;
: !DI[1] = Furnace done signal;
: !DI[2] = Part ready signal;
: !DI[3] = Parts remaining signal;
: !DI[4] = Furnace on signal;
:;
: CALL SAFETY_CHECK;
: CALL INIT_VARS;
: CALL HOME;
: CALL CONNECTION_TEST;
: LBL[1:START];
: FOR R[6:rowCount]=1 TO 8;
:   FOR R[7:colCount]=1 TO 8;
:     CALL CALC_POS;
:     CALL PICKPART;
:     CALL PLACEFURNACE;
:     CALL PUSHBACKSTOP;
:     !Issue part ready signal to PLC;
:     DO[1:PART_READY] = ON;
:     !Wait for PLC acknowledgement;
:     WAIT DI[4:FURNACE_ON] = ON;
:     !Wait for furnace done signal from PLC;
:     WAIT DI[1:FURNACE_DONE] = ON;
:     CALL PICKFURNACE;
:     CALL PLACEFORGE;
:     !Wait for forge ready signal from PLC;
:     WAIT DI[2:PART_READY] = ON;
:     CALL PICKFORGE;
:     CALL OUTPUTPART;
:     !Parts remaining check;
:     IF R[5:PARTS_REMAINING] = 0 THEN;
:       JMP LBL[99:END];
:     ELSE;
:     ENDIF;
:   ENDFOR;
: ENDFOR;
: LBL[99:END];
: END;
/POS
/END
