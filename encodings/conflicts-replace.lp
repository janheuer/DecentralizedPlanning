% Checks for conflicts in the current step and outputs new actions for the robots to resolve all conflicts
%
% In:
% move(D,R,T+1)
% position(P,R,T)
% action(R,T+1)
%
% Out:
% waits(R,T+1)
% moveOut(D,R,T+1)
%
% Assisting:
% conflict(T+1, R1, R2, P)
% predicted(T+1, R, P)



% Predict the next movement
predicted(T+1, R, (PX+DX, PY+DY)) :- position((PX,PY),R,T), move((DX,DY),R,T+1).

% The robot with the higher ID takes priority
conflict(T, R1, R2, P) :- predicted(T, R1, P), predicted(T, R2, P), R1 < R2.

% Check if waiting causes another conflict
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(P,R2,T-1), waits(R2,T).
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(P,R2,T-1), action(R2,T).


waits(R,T) :- conflict(T, R, _, _).
moveOut(D,R,T) :- move(D,R,T), not conflict(T, R, _, _).



#show waits/2.

%#show moveOut/3.
