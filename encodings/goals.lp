#program base.
1 { chooseShelf(N,R) : available(N), product(P,N) } 1 :- order(P,_,_,R), robot(R).
:- robot(R), not chooseShelf(_,R).

% The goals are the points the robot has to visit: the chosen shelf, the picking station and then a place to put the shelf back down
goal(C,R,(1;3)) :- chooseShelf(N,R), shelf(C,N).
goal(C,R,2) :- order(_,S,_,R), station(C,S).
