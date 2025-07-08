/PROG  DIE_MEASUREMENTS_ENHANCED
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Enhanced 3-phase die measurement coordinator";
PROG_SIZE	= 800;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 65;
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
   1:  !Enhanced Die Measurement Coordinator ;
   2:  !Orchestrates 3-phase well detection ;
   3:   ;
   4:  !Initialize measurement system ;
   5:  R[86:command]=1 ;
   6:  R[87:status]=1 ;
   7:  R[91:point_counter]=0 ;
   8:  R[119:zone_map_points]=0 ;
   9:  R[137:well_edge_count]=0 ;
  10:   ;
  11:  !Move to die scanning position ;
  12:J PR[10:die_center] 50% FINE    ;
  13:   ;
  14:  !===== PHASE 1: ZONE MAPPING SCAN ===== ;
  15:  !Signal PC to start zone mapping ;
  16:  R[86:command]=2 ;
  17:  R[87:status]=1 ;
  18:   ;
  19:  CALL ZONE_MAPPING_SCAN    ;
  20:   ;
  21:  !Wait for PC to analyze zones ;
  22:  WAIT R[87:status]=10 TIMEOUT=10000    ;
  23:   ;
  24:  IF R[87]<>10 THEN
  25:  !Zone mapping failed ;
  26:  R[87:status]=3 ;
  27:  MESSAGE[Zone mapping failed] ;
  28:  JMP LBL[99] ;
  29:  ENDIF ;
  30:   ;
  31:  !===== PHASE 2: SMART WELL DETECTION ===== ;
  32:  !Signal PC to start well detection ;
  33:  R[86:command]=3 ;
  34:  R[87:status]=1 ;
  35:   ;
  36:  CALL SMART_WELL_DETECTION    ;
  37:   ;
  38:  !Wait for PC to analyze well edges ;
  39:  WAIT R[87:status]=20 TIMEOUT=15000    ;
  40:   ;
  41:  IF R[87]<>20 THEN
  42:  !Well detection failed ;
  43:  R[87:status]=3 ;
  44:  MESSAGE[Well detection failed] ;
  45:  JMP LBL[99] ;
  46:  ENDIF ;
  47:   ;
  48:  !===== PHASE 3: EDGE VALIDATION ===== ;
  49:  !Signal PC to validate edges ;
  50:  R[86:command]=4 ;
  51:  R[87:status]=1 ;
  52:   ;
  53:  CALL VALIDATE_WELL_EDGES    ;
  54:   ;
  55:  !Wait for final results ;
  56:  WAIT R[87:status]=2 OR R[87:status]=3 TIMEOUT=5000    ;
  57:   ;
  58:  IF R[87]=2 THEN
  59:  !Success - display results ;
  60:  MESSAGE[Well Center: R[93] R[94]] ;
  61:  MESSAGE[Confidence: R[96]%] ;
  62:  ELSE
  63:  !Validation failed or low confidence ;
  64:  MESSAGE[Edge validation failed] ;
  65:  ENDIF ;
  66:   ;
  67:  LBL[99] ;
  68:  !Reset command register ;
  69:  R[86:command]=0 ;
/POS
/END