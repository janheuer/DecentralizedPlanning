%[1,1,1,1,1]
%[1,0,1,1,1]
%[0,1,1,0,1]
%[1,1,0,1,1]
%[1,1,0,1,1]

%start(5, 1).
%goal(1, 1).

%node(1,1). node(1,2). node(1,3). node(1,4). node(1,5).
%node(2,1). node(2,3). node(2,4). node(2,5).
%node(3,2). node(3,3). node(3,5).
%node(4,1). node(4,2). node(4,4). node(4,5).
%node(5,1). node(5,2). node(5,4). node(5,5).


%%%

size(5, 5).
time(1..30).

robot(1).
dir(1,0). dir(-1,0). dir(0,1). dir(0,-1).

pos(X, Y, 0, 1) :- start(X, Y).

{move(X, Y, T, R) : dir(X, Y), robot(R)}<=1 :- time(T).

pos(MX+PX, MY+PY, T, R) :- pos(PX, PY, T-1, R), move(MX, MY, T, R).
:- pos(X, Y, T, R), not node(X, Y).

reached :- pos(X, Y, T, R), goal(X, Y).
:- not reached.


#minimize{1, T : move(_, _, T, _)}.

#show move/4.
#show pos/4.





