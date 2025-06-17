/PROG CALC_OFFSETS
/ATTR
/MN
:!Radial offset piecewise;
: IF R[2:rod_diameter]<=R[20:critical_dia] THEN   ;
:   R[16:radial_offset]=(R[2:rod_diameter]*0.5)   ;
: ELSE   ;
:   R[16:radial_offset]=(R[2:rod_diameter]/1.414)   ;
: ENDIF   ;
:;
:!Jig height adjustment;
: R[16:radial_offset]=(R[16:radial_offset]-R[19:jig_height])  ;
:;
:!Length offset;
:;
: !MISC;
:;
/POS
/END
