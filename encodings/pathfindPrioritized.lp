#program decentralizedNoExternals(r). % used when parameter -i is used
robot(r).

#program base.
#include "./input.lp".
#include "./movement.lp".
#include "./collision.lp".
#include "./action.lp".
#include "./optimization.lp".
#include "./outputPrioritized.lp".
#include "./horizonPrioritized.lp".

:- pos(C,R,_), blockAll(C), robot(R).

:- robot(R), putdown(R,T), move(_,R,T'), T'>T.

goal(C,R,3) :- goal(C,R,1), goal(_,R,2).
