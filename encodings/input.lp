#program base.
dir((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.

% Graphenerstellung am besten immer separat machen
%nextto((X,Y),(DX,DY),(X+DX,Y+DY)) :- node((X,Y)), dir((DX,DY)), node((X+DX,Y+DY)). % ungerichteter Graph
% faster alternative ?
nextto((X,Y),(DX,DY),(X+DX,Y+DY)) :- highway((X,Y)), dir((DX,DY)), highway((X+DX,Y+DY)).
nextto((X,Y),(DX,DY),(X+DX,Y+DY)) :- node((X,Y)), dir((DX,DY)), highway((X+DX,Y+DY)).
nextto((X,Y),(DX,DY),(X+DX,Y+DY)) :- highway((X,Y)), dir((DX,DY)), node((X+DX,Y+DY)).

% Direct translations of the instance for easy use
shelf(C,N) :- init(object(shelf,N),value(at,C)).
station(C,N) :- init(object(pickingStation,N),value(at,C)).
product(N,S) :- init(object(product,N),value(on,(S,_))).
node(C) :- init(object(node,_),value(at, C)).
highway(C) :- init(object(highway,_),value(at,C)).

% prevents the robot from getting stuck on a goal after resolving
action(R,0) :- deliver(_,_,R,0), robot(R).
action(R,0) :- pickup(R,0), robot(R).

% Use the starting position of the robots as highway
highway(C) :- init(object(node,_),value(at,C)),init(object(robot,_),value(at,C)).

#program highways. % optional rule to use when there are no highways defined in the instance
highway(C) :- init(object(node,_),value(at,C)), not shelf(C,_), not station(C,_).
