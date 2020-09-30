dir((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.

node(C) :- init(object(node,_),value(at,C)).
highway(C) :- init(object(highway,_),value(at,C)).

nextto((X,Y),(DX,DY),(X+DX,Y+DY)) :- highway((X,Y)), dir((DX,DY)), highway((X+DX,Y+DY)).
nextto((X,Y),(DX,DY),(X+DX,Y+DY)) :- node((X,Y)), dir((DX,DY)), highway((X+DX,Y+DY)).
nextto((X,Y),(DX,DY),(X+DX,Y+DY)) :- highway((X,Y)), dir((DX,DY)), node((X+DX,Y+DY)).

#show time/1.
#show init/2.
#show nextto/3.
