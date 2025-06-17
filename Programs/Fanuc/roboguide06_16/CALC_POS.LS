/PROG  CALC_POS
/ATTR
OWNER		= ASCBIN;
COMMENT		= "";
PROG_SIZE	= 482;
CREATE		= DATE 25-06-16  TIME 11:36:42;
MODIFIED	= DATE 25-06-16  TIME 11:42:54;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 16;
MEMORY_SIZE	= 790;
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
/MN
   1:  !All in UF1 ;
   2:   ;
   3:  UFRAME_NUM=1 ;
   4:  !Load base position ;
   5:  PR[2:furnace]=PR[1:input]    ;
   6:   ;
   7:  !Calculate X offset (columns) ;
   8:  PR[2,1:furnace]=(PR[1,1:input]+(PR[3,1:pushstart]*(R[2:rod_diameter]-1))) ;
   9:   ;
  10:  !Calculate Y offset (rows) ;
  11:  PR[2,2:furnace]=(PR[1,2:input]+(PR[3,2:pushstart]*(R[1:part_length]-1))) ;
  12:   ;
  13:  !Z remains the same as base pos ;
  14:  PR[2,3:furnace]=PR[1,3:input]    ;
  15:   ;
  16:  END ;
/POS
/END
