/PROG  CALC_POS
/ATTR
OWNER		= ASCBIN;
COMMENT		= "";
PROG_SIZE	= 484;
CREATE		= DATE 25-06-26  TIME 13:23:26;
MODIFIED	= DATE 25-06-26  TIME 13:23:26;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 15;
MEMORY_SIZE	= 796;
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
   1:  !All calculated positions are rel ;
   2:   ;
   3:  !Load base position ;
   4:  PR[8:calc_pos]=PR[1:input_base]    ;
   5:   ;
   6:  !Calculate X offset (columns) ;
   7:  PR[8,1:calc_pos]=(PR[1,1:input_base]+(R[9:horiz_spacing]*(R[7:colCount]-1))) ;
   8:   ;
   9:  !Calculate Y offset (rows) ;
  10:  PR[8,2:calc_pos]=(PR[1,2:input_base]-(R[10:vert_spacing]*(R[6:rowCount]-1))) ;
  11:   ;
  12:  !Z remains the same as base pos ;
  13:  PR[8,3:calc_pos]=PR[1,3:input_base]    ;
  14:   ;
  15:  END ;
/POS
/END
