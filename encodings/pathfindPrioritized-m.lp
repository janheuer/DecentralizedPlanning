#program decentralizedNoExternals(r). % used when parameter -i is used
robot(r).

#program base.
#include "./input.lp".
#include "./goals-m.lp".
#include "./movement.lp".
#include "./collision.lp".
#include "./action-m.lp".
#include "./optimization.lp".
#include "./outputPrioritized.lp".

:- pos(C,R,_), blockAll(C), robot(R).

:- robot(R), pickup(R,T), move(_,R,T'), T'>T.
