% Checks for any conflicts but does not resolve them
%
% In:
% move(D,T,R)
% position(R,P)
% action(T,R)
% waits(R)
%
% Out:
% conflict - 2 robots want to move onto the same position
% swap - 2 robots want to move on each others previous position
% conflictW - robot wants to move on the position of a waiting robot
% conflictWO - robot wants to move on the position of a robot that took the suggested wait
% waitsOut - suggestion that the robot should wait in a conflict
% waitsOutW - suggestion that the robot should wait in a conflictW
% waitsOutWO - suggestion that the robot should wait in a conflictWO
%
% Assisting: 
% predicted(T, R, P)

action(R) :- pickup(R, _).
action(R) :- deliver(_, _, R, _).
action(R) :- putdown(R, _).

move(D, R) :- move(D, R, T).

% Predict the next movement
predicted(R, (PX+DX, PY+DY)) :- position(R, (PX,PY)), move((DX,DY), R).

% The robot with the higher ID takes priority
conflict(R1, R2, P) :- predicted(R1, P), predicted(R2, P), move(_, R1), move(_, R2), R1 < R2.

% check for swapping conflicts
swap(R1, R2) :- predicted(R1, P2), predicted(R2, P1), position(R1, P1), position(R2, P2), R1 < R2. 

% Check if waiting causes another conflict
conflictW(R1, R2, P) :- predicted(R1, P), position(R2, P), waits(R2).
conflictW(R1, R2, P) :- predicted(R1, P), position(R2, P), action(R2).
conflictWO(R1, R2, P) :- predicted(R1, P), position(R2, P), waitsOut(R2).
conflictWO(R1, R2, P) :- predicted(R1, P), position(R2, P), waitsOutW(R2).
conflictWO(R1, R2, P) :- predicted(R1, P), position(R2, P), waitsOutWO(R2).



#show conflict/3.
#show conflictW/3.
#show conflictWO/3.
#show swap/2.