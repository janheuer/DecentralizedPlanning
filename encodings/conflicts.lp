% Checks for any conflicts but does not resolve them
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
% swap(T, R1, R2)
% conflict(T, R1, R2, P)
%
% Assisting: 
% predicted(T, R, P)



% Predict the next movement
predicted(T, R, (PX+DX, PY+DY)) :- position(R, (PX,PY), T), move((DX,DY), T, R).

% The robot with the higher ID takes priority
conflict(T, R1, R2, P) :- predicted(T, R1, P), predicted(T, R2, P), R1 < R2.

# check for swapping conflicts
swap(T, R1, R2) :- predicted(T, R1, P2), predicted(T, R2, P1), position(R1, P1, T), position(R2, P2, T), R1 < R2. 

% Check if waiting causes another conflict
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P, T), waits(T, R2).
conflict(T, R1, R2, P) :- predicted(T, R1, P), position(R2, P, T), action(T, R2).


#show conflict/4.
#show swap/3.
#show 