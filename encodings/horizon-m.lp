% Determine the horizon depending on the size of the instance

sizeX(M) :- #max{X : node((X,Y))} = M.
sizeY(M) :- #max{Y : node((X,Y))} = M.
horizon(T) :- sizeX(X), sizeY(Y), X > Y, T = X+Y+X/2.
horizon(T) :- sizeX(X), sizeY(Y), X <= Y, T = X+Y+Y/2.

time(1..T) :- horizon(T).