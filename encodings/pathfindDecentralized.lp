% Input:
% avaialable(N)           - shelf N is available
% block((X,Y))            - the node (X,Y) is currently blocked
% start((X,Y),R)          - the robot R is on the node (X, Y) at timestep 0
% order(P,S,O,R)          - the order O for product P to the picking station S is to be performed by robot R
% pickup(R,0)             - the robot R has already picked up the shelf in a previous attempt and is currently resolving
% deliver(P,N,R,0)        - the robot R has already delivered the shelf to a station and is currently resolving

% Output:
% move((X,Y),R,T)         - robot R moves in the direction (X,Y) at timestep T
% pos((X,Y),R,T)          - robot R is on the node (X,Y) at timestep T
% action(R,T)             - robot R performs an action (pickup, deliver, putdown) at timestep T
% putdown(R,T)            - robot R puts the shelf he is carrying down on his position at timestep T
% pickup(R,T)             - robot R picks the shelf on his position up at timestep T
% deliver(P,N,R,T)        - robot R delivers the product P from the shelf he is carrying to statisfy order N at timestep T
% carries(R,T)            - robot R carries a shelf at timestep T
% chooseShelf(N,R)        - Robot R will use shelf N to statisfy his order

#program decentralized(r).	% used when parameter -i is not used
robot(r).
#external available(N) : init(object(shelf,N),value(at,_)).
#external start(C,r) : init(object(node,_),value(at,C)).
#external order(P,S,N,r) : init(object(order,N),value(line,(P,_))), init(object(order,N),value(pickingStation,S)). % external damit bestellungen zugeordnet werden können
#external pickup(r,0).
#external deliver(P,N,r,0) : order(P,S,N,r).
#external block(C) : node(C).

#program decentralizedNoExternals(r). % used when parameter -i is used
robot(r).

#program base.
#include "./input.lp".
#include "./goals.lp".
#include "./movement.lp".
#include "./action.lp".
#include "./optimization.lp".
#include "./output.lp".
