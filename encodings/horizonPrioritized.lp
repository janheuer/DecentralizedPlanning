
sizeX(M) :- #max{X : node((X,Y))} = M.
sizeY(M) :- #max{Y : node((X,Y))} = M.
horizon(T) :- sizeX(X), sizeY(Y), X > Y, T = 2*X+2*Y+X/2.
horizon(T) :- sizeX(X), sizeY(Y), X <= Y, T = 2*X+2*Y+Y/2.

time(1..T) :- horizon(T).