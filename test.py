#script (python)

import clingo

def main(prg):
    prg.ground([("base", [])])
    state = [[1,1,1,1,1],
             [1,0,1,1,1],
             [0,1,1,0,1],
             [1,1,0,1,1],
             [1,1,0,1,1]]
    for i in range(len(state)):
        for j in range(len(state[0])):
            if state[i][j] == 1:
                prg.assign_external(clingo.Function("node", [i+1,j+1]), True)
    prg.assign_external(clingo.Function("start", [5,1]), True)
    prg.assign_external(clingo.Function("order", [1,1,1]), True)
    prg.assign_external(clingo.Function("pickupdone", []), False)
    prg.assign_external(clingo.Function("deliverdone", []), False)
    prg.solve()

#end.
