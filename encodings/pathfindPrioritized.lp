#program decentralizedNoExternals(r). % used when parameter -i is used
robot(r).

#program base.
#include "./input.lp".
#include "./goals.lp".
#include "./movement.lp".
#include "./collision.lp".
#include "./action.lp".
#include "./optimization.lp".
#include "./outputPrioritized.lp".

:- pos(C,R,_), blockAll(C), robot(R).

:- robot(R), putdown(R,T), move(_,R,T'), T'>T.
