/PROG PICKPART
/ATTR
/MN
: !input_base is taught to top left pick point;
: !all other points are calculated from this point;
: !input_base should be taught using the teaching jig;
:;
: UTOOL_NUM=1   ;
: UFRAME_NUM=1  ;
: CALL GRIPPER_OPEN ;
:J PR[11:array_approach] 100% CNT100 ;
:J PR[8:calc_pos] 100% CNT20 Offset,PR[22:approachOffs] ;
:L PR[8:calc_pos] 750mm/sec FINE Offset,PR[24:pickoffset] ;
: CALL GRIPPER_CLOSE ;
:L PR[8:calc_pos] 750mm/sec FINE Offset,PR[25:rmvPart] ;
: !add safe array exit motion instructions!;
:;
:;
/POS
/END
