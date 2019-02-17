#program base.
shelf(X, Y, N) :- init(object(shelf,N),value(at,(X,Y))).
station(X, Y, N) :- init(object(pickingStation,N),value(at,(X,Y))).
product(N, S) :- init(object(product,N),value(on,(S,_))).
node(X, Y) :- init(object(node, _), value(at, (X, Y))).
highway(X, Y) :- init(object(highway,_),value(at,(X,Y))).

dir(1, 0). dir(-1, 0). dir(0, 1). dir(0, -1).
robot(1).

#program external.
#external start(X, Y, R) : init(object(node, _), value(at, (X, Y))), robot(R).
%start(1, 4, 1).
%crossings can be blocked (needed for cases d-g)
#external block(X, Y) : init(object(node, _), value(at, (X, Y))).

#program base.
cross(X, Y) :- highway(X, Y), #count{X2, Y2: highway(X2, Y2), dir(DX, DY), X2 = X+DX, Y2 = Y+DY} >= 3, not block(X,Y).

{goal(N, X, Y)} :- cross(X, Y), start(X2, Y2, _), N = |X - X2| + |Y - Y2|.
:- #count{N, X, Y : goal(N, X, Y)} != 1.

#minimize{N : goal(N, _, _)}.

step(T) :- goal(N, X, Y), T = 1..N.


pos(X, Y, 0, R) :- start(X, Y, R).
pos(X+DX, Y+DY, T, R) :- pos(X, Y, T-1, R), move(DX, DY, T, R), robot(R), highway(X, Y).



{move(X, Y, T, R) : dir(X, Y)} <= 1 :- pos(PX, PY, T-1, R), highway(PX, PY), step(T), robot(R).

:- goal(_, X, Y), not pos(X, Y, _, _).

move(DX,DY,N+1,R) :- goal(N,X,Y), highway(X+DX,Y+DY), dir(DX,DY), robot(R).


#show move/4.
#show goal/3.
