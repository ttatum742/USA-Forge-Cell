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
  23:  !Volume = π * r² * L;
  24:  R[29:part_volume] = (3.14159 * ((R[2:rod_diameter] / 2) * (R[2:rod_diameter] / 2)) * R[1:part_length])   ;
  25:   ;
  26:  !Calculate part weight in kg ;
  27:  !Steel density: 0.00000785 kg/mm³ ;
  28:  R[30:part_weight] = R[29:part_volume] * 0.00000785 ;
  29:   ;
  30:  !Calculate combined weight ;
  31:  !EE weight: 3.6kg ;
  32:  R[31:combined_weight] = R[30:part_weight] + 3.6 ;
  33:   ;
  34:  !############## ;
  35:  !## COG CALC ## ;
  36:  !############## ;
  37:   ;
  38:  !Calculate combined COG ;
  39:  !EE COG: (0, 0, 100mm) ;
  40:  !Part COG: Assume at geometric center ;
  41:  !Part COG = (0, 0, part_length/2) ;
  42:   ;
  43:  !Weighted average COG calculation ;
  44:  !X-component (both at 0) ;
  45:  R[32:combined_cog_x] = 0 ;
  46:   ;
  47:  !Y-component (both at 0) ;
  48:  R[33:combined_cog_y] = 0 ;
  49:   ;
  50:  !Z-component weighted average ;
  51:  !COG_z = (m1*z1 + m2*z2)/(m1 + m2) ;
  52:  R[34:combined_cog_z] = ((3.6 * 100) + (R[30:part_weight] * (R[1:part_length] / 2))) / R[31:combined_weight] ;
  53:   ;
  54:  !############## ;
  55:  !## MOI CALC ## ;
  56:  !############## ;
  57:   ;
  58:  !Calculate combined MOI ;
  59:  !For cylindrical part: ;
  60:  !Ixx = Iyy = (1/12)*m*(3*r² + L²) ;
  61:  !Izz = (1/2)*m*r² ;
  62:   ;
  63:  !Part MOI about its center ;
  64:   ;
  65:  !Combined MOI using parallel axis theorem ;
  66:  !I_combined = I_part + I_EE + m*d² terms ;
  67:   ;
  68:  !Assuming EE MOI is small compared to part ;
  69:  !and using simplified calculation ;
  70:   ;
  71:  !XX and YY components ;
  72:  R[35:combined_moi_xx] = (R[30:part_weight] * ((3 * ((R[2:rod_diameter] / 2) * (R[2:rod_diameter] / 2))) + (R[1:part_length] * R[1:part_length])) / 12) ;
  73:  R[36:combined_moi_yy] = R[35:combined_moi_xx] ;
  74:   ;
  75:  !ZZ component ;
  76:  R[37:combined_moi_zz] = (R[30:part_weight] * ((R[2:rod_diameter] / 2) * (R[2:rod_diameter] / 2)) / 2) ;
  77:   ;
/POS
/END
