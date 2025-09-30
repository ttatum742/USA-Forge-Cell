/PROG  PICK_PART
/ATTR
OWNER		= ASCBIN;
COMMENT		= "";
PROG_SIZE	= 748;
CREATE		= DATE 25-06-26  TIME 19:22:24;
MODIFIED	= DATE 25-09-29  TIME 09:12:52;
FILE_NAME	= ;
VERSION		= 0;
LINE_COUNT	= 26;
MEMORY_SIZE	= 1040;
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
   1:  !input_base is taught to top left ;
   2:  !input base is taught in UF3 ;
   3:  !all other points are calculated  ;
   4:  !input_base should be taught usin ;
   5:   ;
   6:  UTOOL_NUM=2 ;
   7:  UFRAME_NUM=3 ;
   8:   ;
   9:  CALL GRIPPER_OPEN    ;
  10:J PR[10:array_approach] 100% CNT100    ;
  11:J PR[8:calc_pos] 100% FINE Tool_Offset,PR[23:approachOffs_t]    ;
  12:  !on the fly conversion from UF3  ;
  13:  UFRAME_NUM=1 ;
  14:  PR[11:pick_pos]=LPOS    ;
  15:  PR[11:pick_pos]=PR[11:pick_pos]+PR[23:approachOffs_t]    ;
  16:  CALL PICK_OFFS ;
  17:  PR[11,2:pick_pos]=PR[11,2:pick_pos]+R[98]  ;
  18:   ;
  19:L PR[11:pick_pos] 750mm/sec CNT75 Offset,PR[22:approachOffs]    ;
  20:   ;
  21:L PR[11:pick_pos] 500mm/sec FINE Offset,PR[24:pickoffset]    ;
  22:  CALL GRIPPER_CLOSE    ;
  23:  !for sim only! ;
  24:  //CALL PICKFROMARRAY    ;
  25:L PR[11:pick_pos] 1000mm/sec CNT50 Offset,PR[25:rmvPart]    ;
  26:  !add safe array exit motion instr ;
  27:  //L PR[11:pick_pos] 1000mm/sec FINE Offset,PR[25:rmvPart] Tool_Offset,PR[33:pointdown]    ;
  28:  PR[50:pickoffset_tool]=JPOS    ;
  29:J PR[50:pickoffset_tool] 100% CNT100 Offset,PR[51]    ;
/POS
/END
