#program base.
action(R,T) :- { pickup(R,T) ; deliver(_,_,R,T) ; putdown(R,T) } = 1, time(T), robot(R).

pickup(R,T) :- pos(C,R,T-1), goal(C,R,1), not putdown(R,T), not pickup(R,T-1), not putdown(R,T-1), not carries(R,T-1), robot(R).
            :- not pickup(R,_), robot(R).

carries(R,T) :- pickup(R,T), robot(R).
carries(R,T) :- carries(R,T-1), not putdown(R,T), time(T), robot(R).

