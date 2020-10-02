% Checks for conflicts in the current step and outputs new actions for the robots to resolve all conflicts
%
% In:
% move(D,R,T)
% position(P,R,T)
% action(R,T)
%
% Out:
% waits(R,T)
% moveOut(D,R,T)
%
% Assisting:
% conflict(T, R1, R2, P)
% predicted(T, R, P)



% Predict the next movement
predicted(T, R, (PX+DX, PY+DY)) :- position((PX,PY),R,T), move((DX,DY),R,T).

% The robot with the higher ID takes priority
conflict(T, R1, R2, P) :- predicted(T, R1, P), predicted(T, R2, P), R1 < R2.

% Check if waiting causes another conflict
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P, T), waits(R2,T).
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P, T), action(R2,T).


waits(R,T) :- conflict(T, R, _, _).
moveOut(D,R,T) :- move(D,R,T), not conflict(T, R, _, _).



#show waits/2.

%#show moveOut/3.
