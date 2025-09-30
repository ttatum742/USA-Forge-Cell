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
  23:  !Inputs already converted to mm in INIT_VARS;
  24:  !Volume = π * r² * L;
  25:  R[29:part_volume] = (3.14159 * ((R[2:rod_diameter] / 2) * (R[2:rod_diameter] / 2)) * R[1:part_length])   ;
  27:   ;
  28:  !Calculate part weight in kg ;
  29:  !Steel density: 0.00000785 kg/mm³ ;
  30:  R[30:part_weight] = R[29:part_volume] * 0.00000785 ;
  31:   ;
  32:  !Calculate combined weight ;
  33:  !EE weight: 3.6kg ;
  34:  R[31:combined_weight] = R[30:part_weight] + 3.6 ;
  35:   ;
  36:  !############## ;
  37:  !## COG CALC ## ;
  38:  !############## ;
  39:   ;
  40:  !Calculate combined COG ;
  41:  !EE COG: (0, 0, 100mm) ;
  42:  !Part COG: Assume at geometric center ;
  43:  !Part COG = (0, 0, part_length/2) ;
  44:   ;
  45:  !Weighted average COG calculation ;
  46:  !X-component (both at 0) ;
  47:  R[32:combined_cog_x] = 0 ;
  48:   ;
  49:  !Y-component (both at 0) ;
  50:  R[33:combined_cog_y] = 0 ;
  51:   ;
  52:  !Z-component weighted average ;
  53:  !COG_z = (m1*z1 + m2*z2)/(m1 + m2) ;
  54:  R[34:combined_cog_z] = ((3.6 * 100) + (R[30:part_weight] * (R[1:part_length] / 2))) / R[31:combined_weight] ;
  55:   ;
  56:  !############## ;
  57:  !## MOI CALC ## ;
  58:  !############## ;
  59:   ;
  60:  !Calculate combined MOI ;
  61:  !For cylindrical part: ;
  62:  !Ixx = Iyy = (1/12)*m*(3*r² + L²) ;
  63:  !Izz = (1/2)*m*r² ;
  64:   ;
  65:  !Part MOI about its center ;
  66:  !All dimensions already in mm ;
  69:   ;
  70:  !Combined MOI using parallel axis theorem ;
  71:  !I_combined = I_part + I_EE + m*d² terms ;
  72:   ;
  73:  !Assuming EE MOI is small compared to part ;
  74:  !and using simplified calculation ;
  75:   ;
  76:  !XX and YY components ;
  77:  R[35:combined_moi_xx] = (R[30:part_weight] * ((3 * ((R[2:rod_diameter] / 2) * (R[2:rod_diameter] / 2))) + (R[1:part_length] * R[1:part_length])) / 12) ;
  78:  R[36:combined_moi_yy] = R[35:combined_moi_xx] ;
  79:   ;
  80:  !ZZ component ;
  81:  R[37:combined_moi_zz] = (R[30:part_weight] * ((R[2:rod_diameter] / 2) * (R[2:rod_diameter] / 2)) / 2) ;
  82:   ;
/POS
/END
