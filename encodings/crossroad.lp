% inputs
#program base.
highway(C) :- init(object(highway,_),value(at,C)).
highway(C) :- init(object(node,_),value(at,C)), init(object(robot,R),value(at,C)).

dir((1,0)). dir((-1,0)). dir((0,1)). dir((0,-1)).

#program external(r).
robot(r).
#external start(C,r) : init(object(node,_),value(at,C)).
#external block(C) : init(object(node,_),value(at,C)).

#program noExternal(r).
robot(r).

% find nearest crossing
#program base.
cross((X,Y)) :- highway((X,Y)), #count{ X2,Y2: highway((X2,Y2)), dir((DX,DY)), X2 = X+DX, Y2 = Y+DY } >= 3, not block((X,Y)).

{ goal(N,(X,Y)) } :- cross((X,Y)), start((X2,Y2),_), N = |X - X2| + |Y - Y2|.
:- #count{ N,C : goal(N,C) } != 1.

#minimize{ N : goal(N,_) }.

% compute path to crossing
step(T) :- goal(N,_), T = 1..N.

{ move(D,R,T) : dir(D) } 1 :- pos(C,R,T-1), highway(C), step(T), robot(R).
move((DX,DY),R,N+1) :- goal(N,(X,Y)), highway((X+DX,Y+DY)), dir((DX,DY)), robot(R).

pos(C,R,0) :- start(C,R).
pos((X+DX,Y+DY),R,T) :- pos((X,Y),R,T-1), move((DX,DY),R,T), robot(R), highway((X,Y)).

:- goal(_,C), not pos(C,_,_).

#show move/3.
#show goal/2.
