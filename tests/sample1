Sample Problem Input Deck (from mcnp5 manual, pg 1-11)
c cell cards for sample problem
1 1 -0.0014 -7
2 2 -7.86 -8
3 3 -1.60 1 -2 -3 4 -5 6 7 8
4 0 -1:2:3:-4:5:-6
c end of cell cards for sample problem

C Beginning of surfaces for cube
1 PZ -5
2 PZ 5
3 PY 5
4 PY -5
5 PX 5
6 PX -5
C End of cube surfaces
7 S 0 -4 -2.5 .5 $ oxygen sphere
8 S 0 4 4.5 .5 $ iron sphere

IMP:N 1 1 1 0
SDEF POS=0 -4 -2.5
F2:N 8 $ flux across surface 8
F4:N 2 $ track length in cell 2
E0 1 12I 14
M1 8016 1 $ oxygen 16
M2 26000 1 $ natural iron
M3 6000 1 $ carbon
NPS 100000
