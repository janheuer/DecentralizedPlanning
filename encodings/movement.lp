#program base.
pos(C,R,0) :- start(C,R).

{ move(D,R,T) : dir(D) } 1 :- robot(R), time(T), not action(R,T).
% { move(D,T,R) : dir(D) } 1 :- pos(P,T-1,R), goal(P,_,_), time(T), action(T-1,R), robot(R). not needed ?

pos(C,R,T) :- move(D,R,T), pos(C',R,T-1),     nextto(C',D,C), highway(C), robot(R).
pos(C,R,T) :- move(D,R,T), pos(C',R,T-1),     nextto(C',D,C), goal(C,R,_), robot(R).
           :- move(D,R,T), pos(C ,R,T-1), not nextto(C,D,_), robot(R).

pos(C,R,T) :- pos(C,R,T-1), action(R,T), time(T), robot(R).

:- pos(C,R,1), block(C), robot(R). % movement on blocked node only forbidden for t=1

:- pos(C,_,_), not node(C), robot(R).

:- pos(C,R,T), shelf(C,_), not goal(C,R,1), time(T), robot(R). % at t=0 robots are allowed to be on nodes with a shelf
:- pos(C,R,_), station(C,_), not goal(C,R,2), robot(R).

:- move((X,Y),R,T), move((-X,-Y),R,T-1), time(T), robot(R).

:- goal(C,R,2), pos(C,R,T), not carries(R,T), robot(R).
