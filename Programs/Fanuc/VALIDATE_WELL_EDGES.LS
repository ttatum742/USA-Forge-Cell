/PROG  VALIDATE_WELL_EDGES
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Phase 3: Edge validation and refinement";
PROG_SIZE	= 1000;
CREATE		= DATE 25-07-01  TIME 12:00:00;
MODIFIED	= DATE 25-07-01  TIME 12:00:00;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 75;
MEMORY_SIZE	= 1500;
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
   1:  !Phase 3: Edge Validation and Refinement ;
   2:  !Validate detected edges form coherent well ;
   3:   ;
   4:  !Calculate preliminary center from edges ;
   5:  IF R[340:well_edge_count]>=4 THEN
   6:  R[360:sum_x]=0 ;
   7:  R[361:sum_y]=0 ;
   8:   ;
   9:  !Sum all edge coordinates ;
  10:  FOR R[362:i]=0 TO (R[340]-1) ;
  11:  R[360]=(R[360]+R[351+R[362]*2]) ;
  12:  R[361]=(R[361]+R[352+R[362]*2]) ;
  13:  ENDFOR ;
  14:   ;
  15:  !Calculate preliminary center ;
  16:  R[363:prelim_center_x]=(R[360]/R[340]) ;
  17:  R[364:prelim_center_y]=(R[361]/R[340]) ;
  18:   ;
  19:  !Validate edge consistency ;
  20:  R[365:radius_sum]=0 ;
  21:  R[366:valid_edges]=0 ;
  22:  R[367:first_radius]=0 ;
  23:   ;
  24:  FOR R[368:i]=0 TO (R[340]-1) ;
  25:  R[369:edge_x]=R[351+R[368]*2] ;
  26:  R[370:edge_y]=R[352+R[368]*2] ;
  27:  R[371:dx]=(R[369]-R[363]) ;
  28:  R[372:dy]=(R[370]-R[364]) ;
  29:  R[373:radius]=SQRT(R[371]*R[371]+R[372]*R[372]) ;
  30:   ;
  31:  !Check radius consistency ;
  32:  IF R[366]>0 THEN
  33:  R[374:avg_radius]=(R[365]/R[366]) ;
  34:  R[375:radius_diff]=ABS(R[373]-R[374]) ;
  35:   ;
  36:  IF R[375]<5 THEN
  37:  !Consistent edge ;
  38:  R[365]=(R[365]+R[373]) ;
  39:  R[366]=(R[366]+1) ;
  40:  !Mark as valid edge ;
  41:  R[380+R[368]]=1 ;
  42:  ENDIF ;
  43:  ELSE
  44:  !First edge - auto valid ;
  45:  R[365]=R[373] ;
  46:  R[366]=1 ;
  47:  R[367]=R[373] ;
  48:  R[380+R[368]]=1 ;
  49:  ENDIF ;
  50:  ENDFOR ;
  51:   ;
  52:  !Calculate final center if sufficient edges ;
  53:  IF R[366]>=4 THEN
  54:  !Recalculate using only consistent edges ;
  55:  R[390:final_sum_x]=0 ;
  56:  R[391:final_sum_y]=0 ;
  57:   ;
  58:  FOR R[392:i]=0 TO (R[340]-1) ;
  59:  IF R[380+R[392]]=1 THEN
  60:  R[390]=(R[390]+R[351+R[392]*2]) ;
  61:  R[391]=(R[391]+R[352+R[392]*2]) ;
  62:  ENDIF ;
  63:  ENDFOR ;
  64:   ;
  65:  !Final results ;
  66:  R[157:final_center_x]=(R[390]/R[366]) ;
  67:  R[158:final_center_y]=(R[391]/R[366]) ;
  68:  R[159:well_diameter]=((R[365]/R[366])*2) ;
  69:  R[160:confidence]=(R[366]*12.5) ;
  70:  R[151:status]=2 ;
  71:  ELSE
  72:  !Insufficient edges - need backup ;
  73:  CALL BACKUP_GRID_SCAN    ;
  74:  ENDIF ;
  75:  ENDIF ;
/POS
/END