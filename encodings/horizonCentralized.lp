
sizeX(M) :- #max{X : node((X,Y))} = M.
sizeY(M) :- #max{Y : node((X,Y))} = M.
horizon(T) :- sizeX(X), sizeY(Y), X > Y, T = 3*X+2*Y.
horizon(T) :- sizeX(X), sizeY(Y), X <= Y, T = 3*X+2*Y.

time(1..T) :- horizon(T).
