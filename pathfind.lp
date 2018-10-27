% kein move und pickup/deliver/putdown zu gleicher zeit
% nach pickup(t) darf roboter nicht mehr über felder mit anderen shelves laufen
% bewegung auf feld mit pickingstation nur wenn dort deliver ausgeführt wird (-> nicht pickingstation für andere roboter blockieren)

#external node(X, Y) : init(object(node, _), value(at, (X, Y))).
#external start(X, Y) : init(object(node, _), value(at, (X, Y))).
#external order(N, P, S) : init(object(order, N), value(line, P)), init(object(order, N), value(pickingStation, S)). % external damit bestellungen zugeordnet werden können
shelf(N, X, Y) :- init(object(shelf, N), value(at, (X, Y))).
station(N, X, Y) :- init(object(pickingStation, N), value(at, (X, Y))).
product(N, S) :- init(object(product, N), value(on, S)).
% bei erneutem solven (weil weg blockiert war) muss festgelegt werden was bereits erledigt wurde
#external pickupdone.
#external deliverdone.

dir(1,0). dir(-1,0). dir(0,1). dir(0,-1).

pos(X, Y, 0) :- start(X, Y).

% Bewegung
{move(X, Y, T) : dir(X, Y)}<=1 :- time(T).
% Positionsaktualisierung
pos(MX+PX, MY+PY, T) :- pos(PX, PY, T-1), move(MX, MY, T).
:- pos(X, Y, T), not node(X, Y).

% Erreichen der 3 Ziele:
pickup(T) :- pos(X, Y, T), goal(X, Y, 1), not putdown(T), not pickupdone.
:- not pickup(_), not pickupdone.
deliver(N, P, T) :- pos(X, Y, T), goal(X, Y, 2), pickup(T2), T2 < T, not deliverdone, order(N, P, S).
:- not deliver(N, P, _), not deliverdone, order(N, P, S).
putdown(T) :- pos(X, Y, T), goal(X, Y, 3), deliver(N, P, T2), T2 < T.
:- not putdown(_).

% Regal reservieren
shelfReserve(N) :- pickup(T), pos(X, Y, T), shelf(N, X, Y).

% Bestimmen der Ziele
goal(X, Y, 1) :- order(N, P, S), shelf(M, X, Y), product(P, M).
goal(X, Y, 2) :- order(N, P, S), station(S, X, Y).
goal(X, Y, 3) :- goal(X, Y, 1).

#minimize{1, T : move(_, _, T)}.

#show putdown/1.
#show deliver/3.
#show pickup/1.
#show move/3.
#show pos/3.
#show shelfReserve/1.


% 3 Schritte:
% 1. Roboter muss ein Shelf finden mit dem gewünschten Produkt, dieses wird dann reserviert
% 2. Roboter nimmt das shelf auf und bringt es an die picking station (Station reservieren?)
% 3. Roboter bringt das shelf:
%	a) Zurück an seinen ursprünglichen Ort
%	b) Irgendwo auf eine gültige Position

% shelf(X, Y, N) :- Das shelf N ist and der Position (X, Y)
% product(N, S) :- Das Produkt N ist an auf shelf S
% station(X, Y, N) :- Es existiert eine picking station N auf Feld (X, Y)
% order(P, S) :- Es gibt eine Order mit dem Produkt P and die picking station S

% ToDo:
% Verhindern, dass mehrere pickups entstehen, wenn der Roboter das Feld mehrmals besucht
% Auswahl eines konkreten shelfs als Ziel und Verbesserung der Programmgeschwindigkeit

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
