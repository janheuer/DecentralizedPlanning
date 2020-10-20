shelf(C,N) :- init(object(shelf,N),value(at,C)).
station(C,N) :- init(object(pickingStation,N),value(at,C)).
product(N,S) :- init(object(product,N),value(on,(S,_))).
node(C) :- init(object(node,_),value(at, C)).



1 { chooseShelf(N,R) : available(N), product(P,N) } 1 :- order(P,_,_,R), robot(R).
%:- robot(R), not chooseShelf(_,R).

% The goals are the points the robot has to visit: the chosen shelf, the picking station and then a place to put the shelf back down
goal(C,R,(1;3)) :- chooseShelf(N,R), shelf(C,N).
goal(C,R,2) :- order(_,S,_,R), station(C,S).



dist(D,R,1) :- start((X,Y), R), goal((CX,CY),R,1), D = |CX-X| + |CY-Y|.
dist(D,R,2) :- goal((X, Y), R, 1), goal((CX,CY),R,2), D = |CX-X| + |CY-Y|.
dist(D,R,3) :- goal((X, Y), R, 2), goal((CX,CY),R,3), D = |CX-X| + |CY-Y|.

distT(D,R) :- dist(D1,R,1), dist(D2,R,2), dist(D3,R,3), D = D1+D2+D3. 
distTT(D) :- #sum{DI, R : distT(DI, R)} = D.

% Try to minimize the estimated path length, lower priority
#minimize{D@5 : distTT(D)}.

#show goal/3.
#show chooseShelf/2.

#program centralized.
start(C,R) :- init(object(robot,R),value(at,C)).
robot(R) :- init(object(robot,R),value(at,_)).
available(N) :- init(object(shelf,N),value(at,_)).

:- chooseShelf(N,R1), chooseShelf(N,R2), R1 < R2.