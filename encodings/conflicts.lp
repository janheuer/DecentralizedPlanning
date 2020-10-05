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

action(R) :- pickup(R).
action(R) :- deliver(R).
action(R) :- putdown(R).



% Predict the next movement
predicted(T, R, (PX+DX, PY+DY)) :- position(R, (PX,PY)), move((DX,DY), T, R).

% The robot with the higher ID takes priority
conflict(T, R1, R2, P) :- predicted(T, R1, P), predicted(T, R2, P), move(_, T, R1), move(_, T, R2), R1 < R2.

# check for swapping conflicts
swap(T, R1, R2) :- predicted(T, R1, P2), predicted(T, R2, P1), position(R1, P1), position(R2, P2), R1 < R2. 

% Check if waiting causes another conflict
conflictW(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P), waits(R2).
conflictW(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P), action(R2).
conflictWO(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P), waitsOut(R2).
conflictWO(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P), waitsOutW(R2).
conflictWO(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P), waitsOutWO(R2).


waitsOut(R) :- conflict(_, R, _, _).
waitsOutW(R) :- conflictW(_, R, _, _).
waitsOutWO(R) :- conflictWO(_, R, _, _).


