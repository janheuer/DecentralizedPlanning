% Bewegung mit shelves ist derzeit nicht eingeschränkt


direction((X,Y)) :- X=-1..1, Y=-1..1, |X+Y|=1.

% Graphenerstellung am besten immer separat machen
nextto((X,Y),(X',Y'),(X+X',Y+Y')) :- position((X,Y)), direction((X',Y')), position((X+X',Y+Y')). % ungerichteter Graph

%*
% Alternative. Bewegung von storage zones auf andere storage zones ist verboten
node(P) :- init(object(node, _), value(at, P)).
highway(P) :- init(object(highway, _), value(at, P)).
nextto((AX, AY), (0,1), (AX,BY)) :- highway((AX, AY)), node((AX,BY)), BY-AY=1.
nextto((AX, AY), (0,-1), (AX,BY)) :- highway((AX, AY)), node((AX,BY)), BY-AY=-1.
nextto((AX, AY), (1,0), (BX,AY)) :- highway((AX, AY)), node((BX,AY)), BX-AX=1.
nextto((AX, AY), (-1,0), (BX,AY)) :- highway((AX, AY)), node((BX,AY)), BX-AX=-1.

nextto((AX, AY), (0,1), (AX,BY)) :- node((AX, AY)), highway((AX,BY)), BY-AY=1.
nextto((AX, AY), (0,-1), (AX,BY)) :- node((AX, AY)), highway((AX,BY)), BY-AY=-1.
nextto((AX, AY), (1,0), (BX,AY)) :- node((AX, AY)), highway((BX,AY)), BX-AX=1.
nextto((AX, AY), (-1,0), (BX,AY)) :- node((AX, AY)), highway((BX,AY)), BX-AX=-1.
*%



%#const horizon=1.
%time(1..horizon).


{    move(D,T,R) : direction(D)} 1 :- isRobot(R), time(T), not action(T,R).

% Not necessary, it doesn't make sense for robots to wait
% Unless the telepathic solution is used
% waits(R,T) :- not pickup(R,_,T), not putdown(R,T), not move(_,T,R), isRobot(R), time(T).

% - position/3 ----------------------------------------------------------------------
position(R,P,T) :- move(D,T,R),    position(R,P',T-1),     nextto(P',D,P).
                :- move(D,T,R),    position(R,P ,T-1), not nextto(P, D,_).

% - pickup/3 --------------------------------------------------------------------		
pickup(T, R) 	:- position(R, P, T-1), goal(P, 1, R), not putdown(T, R), not pickup(T-1, R), not putdown(T-1, R), not carries(T-1, R), isRobot(R).
				:- not pickup(_, R), isRobot(R).

carries(T,R) 	:- pickup(T,R).
				:- pickup(T,R),   carries(T-1,R).  
		
% - putdown/3 -------------------------------------------------------------------
putdown(T, R) 	:- position(R, P, T-1), goal(P, 3, R), deliver(T2, R), T > T2, carries(T-1, R).
				:- not putdown(_, R), isRobot(R).

% - deliver/2 --------------------------------------------------------------------

deliver(T, R) :- position(R,POS,T-1), goal(POS, 2, R), pickup(T2, R), T > T2, not deliver(T-1, R), isRobot(R), carries(T, R).
:- not deliver(_, R), isRobot(R).


% - inertia ---------------------------------------------------------------------
position(R,C,T) :- position(R,C,T-1), action(T,R), isRobot(R), time(T).
carries(T, R) 	:- carries(T-1, R), not putdown(T, R), time(T).

% - (in)direct effects ----------------------------------------------------------
% position(S,C,T) :- position(R,C,T  ),     carries(R,S,T).
% position(S,C,T) :- position(S,C,T-1), not carries(_,S,T), isShelf(S), time(T).

% - edge collision --------------------------------------------------------------
moveto(C',C,T) :- nextto(C',D,C), position(R,C',T-1), move(D,T,R).
 :- moveto(C',C,T), moveto(C,C',T), C < C'.

% - vertex collision ------------------------------------------------------------
 :- { position(R,C,T) : isRobot(R) }  > 1, position(C), time(T).

% - auxiliaries -----------------------------------------------------------------
% :- { position(R,C,T) } != 1, isRobot(R), time(T).

 %:- { carries(R,S,T) } > 1, isRobot(R), time(T).

action(T, R) :- deliver(T,R).
action(T, R) :- pickup(T,R).
action(T, R) :- putdown(T,R).

#minimize{1,T : move(_, T, _)}.

#show move/3.
#show action/2.
#show pickup/2.
#show deliver/2.
#show putdown/2.
%#show position/3.



%*
% Testwerte
position((X,Y)) :- init(object(node, _), value(at, (X, Y))).

goal((3,3), 1, 1).
goal((4,6), 2, 1).
goal((3,3), 3, 1).
position(1,(1,1), 0).
isRobot(1).
*%




