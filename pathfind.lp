% Feldgröße
size(5, 5).
% Horizon
time(1..30).

robot(1).
dir(1,0). dir(-1,0). dir(0,1). dir(0,-1).

pos(X, Y, 0, 1) :- start(X, Y).

% Bewegung
{move(X, Y, T, R) : dir(X, Y), robot(R)}<=1 :- time(T).
% Positionsaktualisierung
pos(MX+PX, MY+PY, T, R) :- pos(PX, PY, T-1, R), move(MX, MY, T, R).
:- pos(X, Y, T, R), not node(X, Y).

% Erreichen der 3 Ziele:
pickup(T) :- pos(X, Y, T, R), goal(X, Y, 1), not putdown(T).
:- not pickup(_).
deliver(T) :- pos(X, Y, T, R), goal(X, Y, 2), pickup(T2), T2 < T.
:- not deliver(_).
putdown(T) :- pos(X, Y, T, R), goal(X, Y, 3), deliver(T2), T2 < T.
:- not putdown(_).

% Regal reservieren
shelfReserve(N) :- pickup(T), pos(X, Y, T, _), shelf(X, Y, N).

% Bestimmen der Ziele
goal(X, Y, 1) :- order(PN, S), shelf(X, Y, N), product(PN, N).
goal(X, Y, 2) :- order(P, S), station(X, Y, S).
goal(X, Y, 3) :- goal(X, Y, 1).



#minimize{1, T : move(_, _, T, _)}.

#show putdown/1.
#show deliver/1.
#show pickup/1.

#show move/4.
#show pos/4.

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

% Überstzung der spezifizierten Instanzen
%node(X, Y) :- init(object(node, _), value(at, (X, Y))).
%robot(R) :- init(object(robot, R), value(at, (_, _))).
%shelf(X, Y, N) :- init(object(shelf,N),value(at,(X,Y))).
%station(X, Y, N) :- init(object(pickingStation,N),value(at,(X,Y))).
%order(P, N) :- init(object(order,N),value(line,(P,_))). % Als external?
%product(N, S) :- init(object(product,N),value(on,(X,Y))), shelf(X, Y, S).

% ToDo:
% Verhindern, dass mehrere pickups entstehen, wenn der Roboter das Feld mehrmals besucht
% Auswahl eines konkreten shelfs als Ziel und Verbesserung der Programmgeschwindigkeit

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%*
%Testinstanz:

%[1,1,1,1,1]
%[1,0,1,1,1]
%[0,1,1,0,1]
%[1,1,0,1,1]
%[1,1,0,1,1]

node(1,1). node(1,2). node(1,3). node(1,4). node(1,5).
node(2,1). node(2,3). node(2,4). node(2,5).
node(3,2). node(3,3). node(3,5).
node(4,1). node(4,2). node(4,4). node(4,5).
node(5,1). node(5,2). node(5,4). node(5,5).

start(5,1).
station(1, 1, 1).
shelf(1, 5, 1).
product(1, 1).
order(1, 1).

*%
