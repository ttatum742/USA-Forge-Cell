/PROG  MAIN_OLD
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
: !PC writes these registers:;
: !R[80-85] = joint torque derivatives;
:;
: !PLC controls these digital inputs:;
: !DI[2] = Part ready signal;
: !DI[4] = Furnace on signal;
:;
: CALL INIT_VARS  ;
: CALL SAFETY_CHECK ;
: CALL HOME ;
: CALL CONNECTION_TEST  ;
: LBL[1]  ;
: FOR R[6:rowCount]=1 TO 4  ;
:   FOR R[7:colCount]=1 TO 8  ;
:     !Main processing loop;
:;
:     CALL CALC_POS ;
:     CALL PICK_PART  ;
:     CALL PLACE_FURNACE  ;
:     CALL PUSHTOBACKSTOP ;
:     !Issue part ready signal to PLC;
:     DO[1]=ON ;
:     !Wait for PLC acknowledgement;
:     WAIT DI[4]=ON  ;
:     !Wait for furnace done signal from PLC;
:     WAIT DI[4]=OFF  ;
:     CALL PICK_FURNACE ;
:     CALL PLACE_FORGE  ;
:     !Wait for forge ready signal from PLC;
:     WAIT DI[2]=ON  ;
:     CALL PICK_FORGE ;
:     CALL OUTPUT_PART  ;
:     !Increment part count;
:     CALL INCREMENT_COUNT  ;
:     !Parts remaining check;
:     IF (R[5:PARTS_REMAINING]=0) THEN  ;
:       JMP LBL[99] ;
:     ELSE  ;
:     ENDIF ;
:   ENDFOR  ;
: ENDFOR  ;
: LBL[99] ;
: END ;
/POS
/END
