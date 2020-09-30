% This conversion only works if:
% The top left node is (1,1)
% The stations are at the top row
% There are at least 2 rows between the stations and the first storage areas, meaning the first shelves can stand at Y=4
% The storage areas are rectangular and in a grid
% The instance includes highways
dir((1,0)). dir((-1,0)). dir((0,1)). dir((0,-1)).

% The 2 lanes next to the stations go both ways
lane(2,l).
lane(3,r).
lane(1,u).


% Create shortcut literals
shelf((X, Y), N) :- init(object(shelf,N),value(at,(X,Y))).
station((X, Y), N) :- init(object(pickingStation,N),value(at,(X,Y))).
product(N, S) :- init(object(product,N),value(on,(S,_))).
node((X, Y)) :- init(object(node, _), value(at, (X, Y))).
highway((X, Y)) :- init(object(highway,_),value(at,(X,Y))).


% Create the graph according to lane/2
nextto((AX, AY), (1,0), (BX,AY)) :- lane(AY, r), highway((AX, AY)), highway((BX,AY)), AX-BX==-1.
nextto((AX, AY), (-1,0), (BX,AY)) :- lane(AY, l), highway((AX, AY)), highway((BX,AY)), AX-BX==1.
nextto((AX, AY), (0,1), (AX,BY)) :- lane(AX, d), highway((AX, AY)), highway((AX,BY)), AY-BY==-1.
nextto((AX, AY), (0,-1), (AX,BY)) :- lane(AX, u), highway((AX, AY)), highway((AX,BY)), AY-BY==1.

% Shelves can be put down in rectangular storage areas with lanes inbetween
% way/2 indicates where these lanes are
storage(P) :- node(P), not highway(P), not station(P, _).
way(Y, h) :- highway((X,Y)), storage((X,Y+1)), storage((X,Y-1)).
way(X, v) :- highway((X,Y)), storage((X+1,Y)), storage((X-1,Y)).
way(X, v) :- highway((X,Y)), not node((X+1,Y)), storage((X-1,Y)). % Special case for the last lane at the right edge

% Put the horizontal lanes into an order from top to bottom
{way(Y, N, h) : way(Y,h)} = 1 :- N = 1..Z, #count{Y:way(Y,h)} = Z.
:- way(Y,h), not way(Y,_,h).
:- way(Y,N,h), way(Y2,N2,h), Y < Y2, N > N2.

% put the vertical lanes into an order
{way(X, N, v) : way(X,v)} = 1 :- N = 1..Z, #count{X:way(X,v)} = Z.
:- way(X,v), not way(X,_,v).
:- way(X,N,v), way(X2,N2,v), X < X2, N > N2.

% Create new lane/2 literals that indicate the direction of the lane: up, down, left, right
lane(X, u) :- way(X,N,v), N \ 2 == 0.
lane(X, d) :- way(X,N,v), N \ 2 == 1.
lane(Y, l) :- way(Y,N,h), N \ 2 == 0.
lane(Y, r) :- way(Y,N,h), N \ 2 == 1.


% Allow for free movement in the 2 top lanes, next to the stations
nextto((X,2), (0, 1), (X,3)) :- nextto((X-1,3), (0,-1), (X-1,2)), node((X, _)).
nextto((X,3), (0, -1), (X,2)) :- nextto((X-1,2), (0,1), (X-1,3)), node((X, _)).

% Allow free movement between storage zones and highways
nexttoSt((AX,AY), (DX,DY), (BX,BY)) :- storage((AX,AY)), highway((BX,BY)), dir((DX,DY)), AX+DX=BX, AY+DY=BY.
nextto(P, D, P2) :- nexttoSt(P,D,P2).
nextto(P2, (-DX, -DY), P) :- nexttoSt(P, (DX,DY), P2).

#show time/1.
#show init/2.
#show nextto/3.
%#show storage/1.
%#show lane/2.
