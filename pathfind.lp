% Überstzung der spezifizierten Instanzen
node(X, Y) :- init(object(node, _), value(at, (X, Y))).
% robot(R) :- init(object(robot, R), value(at, (_, _))).
#external available(N) : init(object(shelf,N),value(at,(X,Y))).
shelf(X, Y, N) :- init(object(shelf,N),value(at,(X,Y))).
station(X, Y, N) :- init(object(pickingStation,N),value(at,(X,Y))).
product(N, S) :- init(object(product,N),value(on,(S,_))).
highway(X, Y) :- init(object(highway,_),value(at,(X,Y))).
#external start(X, Y) : init(object(node, _), value(at, (X, Y))).
#external order(P, S) : init(object(order, N), value(line, (P, A))), init(object(order, N), value(pickingStation, S)). % external damit bestellungen zugeordnet werden können
% bei erneutem solven (weil weg blockiert war) muss festgelegt werden was bereits erledigt wurde
#external pickup(0).
#external deliver(0).
#external block(X, Y) : node(X, Y).


time(1..50).

pos(X, Y, 0) :- start(X, Y).
dir(1, 0). dir(-1,0). dir(0, 1). dir(0, -1).
pos(X+DX, Y+DY, T) :- pos(X, Y, T-1), move(DX, DY, T).


{move(X, Y, T) : dir(X, Y)} <= 1 :- pos(PX, PY, T-1), node(PX, PY), time(T), not action(T).

pos(X, Y, T) :- pos(X, Y, T-1), node(X, Y), time(T), action(T).

{move(X, Y, T) : dir(X, Y)} <= 1 :- pos(PX, PY, T-1), node(PX, PY), time(T), action(T-1).


action(T) :- {pickup(T); deliver(T); putdown(T)} = 1, time(T).

:- pos(X, Y, 1), block(X, Y). % bewegung auf blocked feld nur für t=1 nicht erlaubt
:- pos(X, Y, _), not node(X, Y).
:- pos(X, Y, T), shelf(X, Y, _), not goal(X, Y, 1), time(T). % zu t=0 darf roboter auf feld mit shelf stehen
:- pos(X, Y, _), station(X, Y, N), not goal(X, Y, 2).
:- move(X, Y, T), move(-X, -Y, T-1), time(T).

pickup(T) :- pos(X, Y, T-1), goal(X, Y, 1), not putdown(T), not pickup(T-1), not putdown(T-1), not carries(T-1).
:- not pickup(_).

carries(T) :- pickup(T).
carries(T) :- carries(T-1), not putdown(T), time(T).

deliver(T) :- pos(X, Y, T-1), goal(X, Y, 2), pickup(T2), T > T2, not deliver(T-1).
:- not deliver(_).
putdown(T) :- pos(X, Y, T-1), goal(X, Y, 3), deliver(T2), T > T2, carries(T-1).
:- not putdown(_).

#minimize{1,T : move(X, Y, T)}.

%{goal(X, Y, 1) : shelf(X, Y, N), product(P, N)} = 1 :- order(P, _).
{chooseShelf(N) : available(N), product(P, N)} = 1 :- order(P, _).
goal(X, Y, 1) :- chooseShelf(N), shelf(X, Y, N).
%{goal(X, Y, 1) : shelf(X, Y, N), available(N)} = 1. % roboter kann nur aus verfügbaren shelves auswählen
% im moment immer nur ein shelf verfügbar
goal(X, Y, 2) :- order(P, S), station(X, Y, S).
goal(X, Y, 3) :- goal(X, Y, 1).

#show action/1.
#show putdown/1.
#show deliver/1.
#show pickup/1.
#show goal/3.
#show move/3.
#show pos/3.
#show carries/1.
#show block/2.
#show chooseShelf/1.
