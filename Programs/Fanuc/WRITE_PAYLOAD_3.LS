/PROG  WRITE_PAYLOAD_3
/ATTR
OWNER		= MNEDITOR;
COMMENT		= "stores weight data in sysvar";
PROG_SIZE	= 399;
CREATE		= DATE 25-06-05  TIME 09:26:14;
MODIFIED	= DATE 25-06-10  TIME 06:13:06;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 13;
MEMORY_SIZE	= 867;
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
  ENABLE_SINGULARITY_AVOIDANCE   : FALSE;

: !Program Description: Rewrites ;
: !payload id 3, doesnt set it. ;
: !set it by PAYLOAD[x] in your program
: !Takes the following arguments: ;
: ! AR[1] = Weight of payload (kg) ;
: ! AR[2-4] = CoG (cm). ;
: ! AR[5-7] = Inertias (kgfcm^2). ;
: ;
: !Payload in kg. ;
: $PLST_GRP1[3].$PAYLOAD=(AR[1]) ;
: !Payload CoG in cm. ;
: $PLST_GRP1[3].$PAYLOAD_X=(AR[2]) ;
: $PLST_GRP1[3].$PAYLOAD_Y=(AR[3]) ;
: $PLST_GRP1[3].$PAYLOAD_Z=(AR[4]) ;
: !Payload inertia in kgfcm^2. ;
: $PLST_GRP1[3].$PAYLOAD_IX=(AR[5]*980) ;
: $PLST_GRP1[3].$PAYLOAD_IY=(AR[6]*980) ;
: $PLST_GRP1[3].$PAYLOAD_IZ=(AR[7]*980) ;
:  ; 
/POS
/END
