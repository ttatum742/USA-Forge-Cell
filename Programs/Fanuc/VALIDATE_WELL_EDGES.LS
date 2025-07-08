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
   5:  IF R[137:well_edge_count]>=4 THEN
   6:  R[178:sum_x]=0 ;
   7:  R[179:sum_y]=0 ;
   8:   ;
   9:  !Sum all edge coordinates ;
  10:  FOR R[180:i]=0 TO (R[137]-1) ;
  11:  R[178]=(R[178]+R[148+R[180]*2]) ;
  12:  R[179]=(R[179]+R[149+R[180]*2]) ;
  13:  ENDFOR ;
  14:   ;
  15:  !Calculate preliminary center ;
  16:  R[181:prelim_center_x]=(R[178]/R[137]) ;
  17:  R[182:prelim_center_y]=(R[179]/R[137]) ;
  18:   ;
  19:  !Validate edge consistency ;
  20:  R[183:radius_sum]=0 ;
  21:  R[184:valid_edges]=0 ;
  22:  R[185:first_radius]=0 ;
  23:   ;
  24:  FOR R[186:i]=0 TO (R[137]-1) ;
  25:  R[187:edge_x]=R[148+R[186]*2] ;
  26:  R[188:edge_y]=R[149+R[186]*2] ;
  27:  R[189:dx]=(R[187]-R[181]) ;
  28:  R[190:dy]=(R[188]-R[182]) ;
  29:  R[191:radius]=SQRT(R[189]*R[189]+R[190]*R[190]) ;
  30:   ;
  31:  !Check radius consistency ;
  32:  IF R[184]>0 THEN
  33:  R[192:avg_radius]=(R[183]/R[184]) ;
  34:  R[193:radius_diff]=ABS(R[191]-R[192]) ;
  35:   ;
  36:  IF R[193]<5 THEN
  37:  !Consistent edge ;
  38:  R[183]=(R[183]+R[191]) ;
  39:  R[184]=(R[184]+1) ;
  40:  !Mark as valid edge ;
  41:  R[194+R[186]]=1 ;
  42:  ENDIF ;
  43:  ELSE
  44:  !First edge - auto valid ;
  45:  R[183]=R[191] ;
  46:  R[184]=1 ;
  47:  R[185]=R[191] ;
  48:  R[194+R[186]]=1 ;
  49:  ENDIF ;
  50:  ENDFOR ;
  51:   ;
  52:  !Calculate final center if sufficient edges ;
  53:  IF R[184]>=4 THEN
  54:  !Recalculate using only consistent edges ;
  55:  R[195:final_sum_x]=0 ;
  56:  R[196:final_sum_y]=0 ;
  57:   ;
  58:  FOR R[197:i]=0 TO (R[137]-1) ;
  59:  IF R[194+R[197]]=1 THEN
  60:  R[195]=(R[195]+R[148+R[197]*2]) ;
  61:  R[196]=(R[196]+R[149+R[197]*2]) ;
  62:  ENDIF ;
  63:  ENDFOR ;
  64:   ;
  65:  !Final results ;
  66:  R[93:final_center_x]=(R[195]/R[184]) ;
  67:  R[94:final_center_y]=(R[196]/R[184]) ;
  68:  R[95:well_diameter]=((R[183]/R[184])*2) ;
  69:  R[96:confidence]=(R[184]*12.5) ;
  70:  R[87:status]=2 ;
  71:  ELSE
  72:  !Insufficient edges - need backup ;
  73:  CALL BACKUP_GRID_SCAN    ;
  74:  ENDIF ;
  75:  ENDIF ;
/POS
/END