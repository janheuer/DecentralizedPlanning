#program base.
start(C,R) :- init(object(robot,R),value(at,C)).
robot(R) :- init(object(robot,R),value(at,_)).
available(N) :- init(object(shelf,N),value(at,_)).

:- chooseShelf(N,R1), chooseShelf(N,R2), R1 < R2.

#include "./input.lp".
%#include "./goals-m.lp".
#include "./movement.lp".
#include "./collision.lp".
#include "./action-m.lp".
#include "./optimization.lp".
#include "./output.lp".
#include "./horizon-m.lp".