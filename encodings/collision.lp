#program base.
:- { pos(P,R,T) } > 1, node(P), time(T).

moveto(C,C',T) :- nextto(C,D,C'), pos(C,R,T-1), move(D,R,T).
               :- moveto(C,C',T), moveto(C',C,T), C < C'.
