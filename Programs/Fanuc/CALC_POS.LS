/PROG CALC_POS
/ATTR
/MN
:!All pick positions are relative to UF[1];
:;
:UFRAME_NUM=1   ;
:!Load base position;
:PR[2]=PR[1]    ;
:;
:!Calculate X offset (columns);
:PR[2,1]=(PR[1,1]+(PR[3,1]*(R[2]-1)))   ;
:;
:!Calculate Y offset (rows);
:PR[2,2]=(PR[1,2]+(PR[3,2]*(R[1]-1))) ;
:;
:!Z remains the same as base pos;
:PR[2,3]=PR[1,3]    ;
:;
:END    ;
/POS
/END
