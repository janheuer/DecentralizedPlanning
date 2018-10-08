% In:
% size(X, Y) 	- Raum mit der Größe X Y
% block(X, Y) 	- Feld an der pos X Y ist geblockt
% start(X, Y)	- Startfeld
% goal(X, Y)	- Gewolltes Endfeld

% pos(X, Y, Z)		- Roboter an pos X Y zum Zeitpunkt Z
% move(X, Y, Z)	- Roboter bewegt sich in Richtung X Y zum Zeitpunkt Z

#program base.
horizon(20).
size(5, 5).


dir(1, 0). dir(-1, 0). dir(0, 1). dir(0, -1).

% Startposition
pos(X, Y, 0) :- start(X, Y).


% Position nach einem move berechnen und prüfen, ob der Zug möglich ist
pos(X+A, Y+B, N+1) :- pos(X, Y, N), move(A, B, N+1), N < H, horizon(H), X+A > 0, Y+B > 0, size(SX, SY), X+A < SX, Y+B < SY, not block(X, Y).

% Bewegungen machen
{move(DX, DY, N+1) : dir(DX, DY)} <= 1 :- pos(_, _, N), N < H, horizon(H).

% Probleme, wenn ein Feld bei änderungen mehrmals benutzt werden muss? 
%:- pos(X, Y, N), pos(X2, Y2, N), X != X2.
%:- pos(X, Y, N), pos(X2, Y2, N), Y != Y2.
:- pos(X, Y, N1), pos(X, Y, N2), N1 < N2.

% Haben wir das Ziel erreicht?
reached :- pos(X, Y, N), goal(X, Y).
:- not reached.

% Optimize
#minimize{1, N : move(_, _, N)}.


#show move/3.
#show pos/3.






