#program decentralizedNoExternals(r). % used when parameter -i is used
robot(r).

#program base.
#include "./input.lp".
#include "./goals.lp".
#include "./movement.lp".
#include "./action.lp".
#include "./optimization.lp".
%#include "./output.lp".
#show action/2.
#show putdown/2.
#show deliver/4.
#show pickup/2.
#show goal/3.
#show move(D,R,T) : move(D,R,T), robot(R).
#show pos(C,R,T) : pos(C,R,T), robot(R).
#show carries/2.
#show block/1.
#show chooseShelf/2.

#include "./collision.lp".
