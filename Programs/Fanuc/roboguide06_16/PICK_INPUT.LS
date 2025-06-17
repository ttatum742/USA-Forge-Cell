/PROG  PICK_INPUT
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "";
PROG_SIZE	= 1057;
CREATE		= DATE 25-06-05  TIME 09:24:42;
MODIFIED	= DATE 25-06-06  TIME 07:57:10;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 97;
MEMORY_SIZE	= 1441;
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
   1:  UTOOL_NUM=1 ;
   2:J PR[1:input] 50% FINE Tool_Offset,PR[21:approach offs]    ;
   3:  CALL GRIPPER_OPEN    ;
   4:  !wait for btn ;
   5:  WAIT DI[66]=ON    ;
   6:L PR[1:input] 200mm/sec FINE    ;
   7:  CALL GRIPPER_CLOSE    ;
   8:L PR[1:input] 200mm/sec FINE Offset,PR[20:zoffset150]    ;
   9:L PR[1:input] 200mm/sec CNT100 Offset,PR[20:zoffset150] Tool_Offset,PR[21:approach offs]    ;
  10:   ;
  11:   ;
  12:   ;
  13:   ;
  14:   ;
  15:   ;
  16:   ;
  17:   ;
  18:   ;
  19:   ;
  20:   ;
  21:   ;
  22:   ;
  23:   ;
  24:   ;
  25:   ;
  26:   ;
  27:   ;
  28:   ;
  29:   ;
  30:   ;
  31:   ;
  32:   ;
  33:   ;
  34:   ;
  35:   ;
  36:   ;
  37:   ;
  38:   ;
  39:   ;
  40:   ;
  41:   ;
  42:   ;
  43:   ;
  44:   ;
  45:   ;
  46:   ;
  47:   ;
  48:   ;
  49:   ;
  50:   ;
  51:   ;
  52:   ;
  53:   ;
  54:   ;
  55:   ;
  56:   ;
  57:   ;
  58:   ;
  59:   ;
  60:   ;
  61:   ;
  62:   ;
  63:   ;
  64:   ;
  65:   ;
  66:   ;
  67:   ;
  68:   ;
  69:   ;
  70:   ;
  71:   ;
  72:   ;
  73:   ;
  74:   ;
  75:   ;
  76:   ;
  77:   ;
  78:   ;
  79:   ;
  80:   ;
  81:   ;
  82:   ;
  83:   ;
  84:   ;
  85:   ;
  86:   ;
  87:   ;
  88:   ;
  89:   ;
  90:   ;
  91:   ;
  92:   ;
  93:   ;
  94:   ;
  95:   ;
  96:   ;
  97:   ;
/POS
/END
