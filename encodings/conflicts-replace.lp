% Checks for conflicts in the current step and outputs new actions for the robots to resolve all conflicts
%
% In:
% move(D,T,R)
% position(R,P,T)
% action(T,R)
% waits(T,R)
%
% Out:
% waits(T,R)
% moveOut(D,T,R)
% action(T,R)
%
% Assisting: 
% conflict(T, R1, R2, P)
% predicted(T, R, P)



% Predict the next movement
predicted(T, R, (PX+DX, PY+DY)) :- position(R, (PX,PY), T), move((DX,DY), T, R).

% The robot with the higher ID takes priority
conflict(T, R1, R2, P) :- predicted(T, R1, P), predicted(T, R2, P), R1 < R2.

% Check if waiting causes another conflict
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P, T), waits(T, R2).
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P, T), action(T, R2).


waits(T, R) :- conflict(T, R, _, _).
moveOut(D, T, R) :- move(D, T, R), not conflict(T, R, _, _).



#show waits/2.
#show action/2.

#show moveOut/3.
