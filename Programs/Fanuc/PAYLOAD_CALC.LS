/PROG  PAYLOAD_CALC
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "Calculates approximate part weight";
PROG_SIZE	= 415;
CREATE		= DATE 25-05-05  TIME 16:48:58;
MODIFIED	= DATE 25-05-07  TIME 03:59:18;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 6;
MEMORY_SIZE	= 827;
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
   1:  !ONLY RUN AFTER INIT_VARS ;
   2:  !SO UNITS STAY CONSISTENT ;
   3:   ;
   4:  !###################### ;
   5:  !## INPUT VALIDATION ## ;
   6:  !###################### ;
   7:  IF ((R[1:part_length] < 63.5) OR (R[1:part_length] > 304.8)) THEN ;
   8:       MESSAGE[Error: Invalid length entered] ;
   9:       ABORT ;
  10:  ELSE ;
  11:       IF ((R[2:rod_diameter] < 12.7) OR (R[2:rod_diameter] > 38.1)) THEN ;
  12:           MESSAGE[Error: Invalid diameter entered] ;
  13:           ABORT ;
  14:       ELSE ;
  15:       ENDIF ;
  16:  ENDIF ;
  17:   ;
  18:  !################# ;
  19:  !## WEIGHT CALC ## ;
  20:  !################# ;
  21:   ;
  22:  !Calculate volume in mm3;
  23:  R[999:volume] = (3.14 * (R[16:radial_offset] * R[16:radial_offset]) * R[1:part_length])   ;
  24:   ;
  25:  !Calculate weight in kg ;
  26:   ;
  27:  !Assuming steel density of ;
  28:  !0.00000000785 kg/mm3 ;
  29:   ;
  30:  !Also adding EOAT weight ;
  31:  R[998:EOAweight] = (R[999:volume] * 0.00000000785) + 4.73 ;
  32:   ;
  33:  !############## ;
  34:  !## COG CALC ## ;
  35:  !############## ;
  36:   ;
  37:  !First calc part COG elements ;
  38:   ;
  39:  !Using weighted avg formula ;
  40:  !for composite bodies ;
  41:  !to include EE COG ;
  42:   ;
  43:   ;
  44:  !############## ;
  45:  !## MOI CALC ## ;
  46:  !############## ;
  47:   ;
  48:  !First calc part MOI elements ;
  49:   ;
  50:  !Use parallel axis theorem to ;
  51:  !combine inertia tensors ;
  52:   ;
  53:   ;
/POS
/END
