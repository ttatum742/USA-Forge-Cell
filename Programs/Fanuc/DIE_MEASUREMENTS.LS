/PROG  DIE_MEASUREMENT
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Die Center Measurement";
PROG_SIZE	= 800;
CREATE		= DATE 25-07-01  TIME 10:00:00;
MODIFIED	= DATE 25-07-01  TIME 10:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 45;
MEMORY_SIZE	= 1200;
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
   1:  !Die Center Measurement Program ;
   2:  !PC Communication Registers: ;
   3:  !R[86] = Command (1=start, 2=grid, 3=confirm, 0=stop) ;
   4:  !R[87] = Status (0=ready, 1=busy, 2=complete, 3=error) ;
   5:  !R[88] = Current X position ;
   6:  !R[89] = Current Y position ;
   7:  !R[90] = Current Z position ;
   8:  !R[91] = Measurement point counter ;
   9:  !R[92] = Total points planned ;
  10:  !R[93] = Die center X result ;
  11:  !R[94] = Die center Y result ;
  12:  !R[95] = Average die height ;
  13:  !R[96] = Confidence level (0-100) ;
  14:   ;
  15:  !Initialize communication ;
  16:  R[86:command]=0 ;
  17:  R[87:status]=0 ;
  18:  R[91:point_counter]=0 ;
  19:   ;
  20:  !Wait for PC to send start command ;
  21:  WAIT R[86:command]=1 ;
  22:  R[87:status]=1 ;
  23:   ;
  24:  !Phase 1: Grid measurement ;
  25:  R[86:command]=2 ;
  26:  CALL GRID_SCAN ;
  27:   ;
  28:  !Wait for PC analysis ;
  29:  WAIT R[86:command]=3 ;
  30:   ;
  31:  !Phase 2: Confirmation measurement ;
  32:  CALL CONFIRM_CENTER ;
  33:   ;
  34:  !Wait for final results ;
  35:  WAIT R[87:status]=2 ;
  36:   ;
  37:  !Check confidence level ;
  38:  IF R[96:confidence]<70 THEN ;
  39:  MESSAGE[Low confidence in die center] ;
  40:  R[87:status]=3 ;
  41:  ABORT ;
  42:  ENDIF ;
  43:   ;
  44:  !Results available in R[93-159] ;
  45:  R[87:status]=0 ;
/POS
/END
