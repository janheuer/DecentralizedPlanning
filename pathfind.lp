% Überstzung der spezifizierten Instanzen
#program centralized.
start(X, Y, R) :- init(object(robot,R),value(at,(X,Y))).
robot(R) :- init(object(robot,R),value(at,(X,Y))).
available(N) :- init(object(shelf,N),value(at,(X,Y))).

#program decentralized.
robot(1).


#program base.
%
shelf(X, Y, N) :- init(object(shelf,N),value(at,(X,Y))).
station(X, Y, N) :- init(object(pickingStation,N),value(at,(X,Y))).
product(N, S) :- init(object(product,N),value(on,(S,_))).
node(X, Y) :- init(object(node, _), value(at, (X, Y))).
highway(X, Y) :- init(object(highway,_),value(at,(X,Y))).
% Damit auch Instanzen ohne highway-nodes nutzbar sind. Nur aktivieren, wenn notwendig, wegen performance
% highway(X, Y) :- init(object(node, _), value(at, (X,Y))), not shelf(X, Y, _), not station(X, Y, _).
% Startpositionen der Roboter auch als highway nutzen
highway(X, Y) :- init(object(node, _), value(at, (X,Y))), init(object(robot,R),value(at,(X,Y))).


pos(X, Y, 0, R) :- start(X, Y, R).
dir(1, 0). dir(-1,0). dir(0, 1). dir(0, -1).
pos(X+DX, Y+DY, T, R) :- pos(X, Y, T-1, R), move(DX, DY, T, R), robot(R), highway(X+DX, Y+DY).
pos(X+DX, Y+DY, T, R) :- pos(X, Y, T-1, R), move(DX, DY, T, R), robot(R), goal(X+DX, Y+DY, _, _).


{move(X, Y, T, R) : dir(X, Y)} <= 1 :- pos(PX, PY, T-1, R), node(PX, PY), time(T), not action(T, R), robot(R).
%{move(X, Y, T, R) : dir(X, Y)} <= 1 :- pos(PX, PY, T-1, R), goal(PX, PY, _, _), time(T), not action(T, R), robot(R).

pos(X, Y, T, R) :- pos(X, Y, T-1, R), goal(X, Y, _, _), time(T), action(T, R), robot(R).

{move(X, Y, T, R) : dir(X, Y)} <= 1 :- pos(PX, PY, T-1, R), goal(PX, PY, _, _), time(T), action(T-1, R), robot(R).


action(T, R) :- {pickup(T, R); deliver(T, R, _, _); putdown(T, R)} = 1, time(T), robot(R).

:- pos(X, Y, 1, _), block(X, Y). % bewegung auf blocked feld nur für t=1 nicht erlaubt
:- pos(X, Y, _, _), not node(X, Y).
:- pos(X, Y, T, R), shelf(X, Y, _), not goal(X, Y, 1, R), time(T). % zu t=0 darf roboter auf feld mit shelf stehen
:- pos(X, Y, _, R), station(X, Y, N), not goal(X, Y, 2, R).
:- move(X, Y, T, R), move(-X, -Y, T-1, R), time(T).
:- pos(X, Y, T, R1), pos(X, Y, T, R2), R1 != R2.
:- pos(X1, Y, T, R1), pos(X2, Y, T, R2), pos(X2, Y, T+1, R1), pos(X1, Y, T+1, R2), R1 != R2, X1 != X2.
:- pos(X, Y1, T, R1), pos(X, Y2, T, R2), pos(X, Y2, T+1, R1), pos(X, Y1, T+1, R2), R1 != R2, Y1 != Y2.
:- chooseShelf(N, R1), chooseShelf(N, R2), R1 != R2.

pickup(T, R) :- pos(X, Y, T-1, R), goal(X, Y, 1, R), not putdown(T, R), not pickup(T-1, R), not putdown(T-1, R), not carries(T-1, R), robot(R).
:- not pickup(_, R), robot(R).

carries(T, R) :- pickup(T, R).
carries(T, R) :- carries(T-1, R), not putdown(T, R), time(T).

deliver(T, R, P, N) :- pos(X, Y, T-1, R), goal(X, Y, 2, R), pickup(T2, R), T > T2, not deliver(T-1, R, P, N), robot(R), order(P, S, R, N).
:- not deliver(_, R, _, _), robot(R).
putdown(T, R) :- pos(X, Y, T-1, R), goal(X, Y, 3, R), deliver(T2, R, _, _), T > T2, carries(T-1, R).
:- not putdown(_, R), robot(R).

#minimize{1,T : move(X, Y, T, R)}.


{chooseShelf(N, R) : available(N), product(P, N)}=1 :- order(P, _, R, _), robot(R).
:- robot(R), not chooseShelf(_, R).
goal(X, Y, 1, R) :- chooseShelf(N, R), shelf(X, Y, N).
goal(X, Y, 2, R) :- order(P, S, R, _), station(X, Y, S).
goal(X, Y, 3, R) :- goal(X, Y, 1, R).

#show action/2.
#show putdown/2.
#show deliver/4.
#show pickup/2.
#show goal/4.
#show move/4.
#show pos/4.
#show carries/2.
#show block/2.
#show chooseShelf/2.
