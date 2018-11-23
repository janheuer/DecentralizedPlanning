import clingo

class Robot(object):
    # next_pos = [x,y]
    # next_action = move | pickup | deliver | putdown
    # model
    # order = [orderID,productID,stationID]
    # state
    # pickupdone
    # deliverdone
    # t
    # shelf

    def __init__(self, id, start, encoding, instance):
        self.id = id
        self.start = [-1,-1]
        self.start[0] = start[0]
        self.start[1] = start[1]
        self.pos = [-1,-1]
        self.pos[0] = start[0]
        self.pos[1] = start[1]

        self.next_pos = [-1,-1]

        self.state = []
        self.t=-1

        self.order = [-1,-1,-1]
        self.available_shelves = []
        self.shelf = -1

        self.pickupdone = False
        self.deliverdone = False

        self.prg = clingo.Control()
        self.prg.load(encoding)
        self.prg.load(instance)
        self.prg.ground([("base", []), ("decentralized", [])])

        self.plan_finished = True

    def solve(self):
        self.model = []
        self.prg.assign_external(clingo.Function("start", [self.start[0],self.start[1],1]), False)
        self.prg.assign_external(clingo.Function("start", [self.pos[0],self.pos[1],1]), True)
        self.start[0] = self.pos[0]
        self.start[1] = self.pos[1]
        self.plan_finished = False

        self.prg.assign_external(clingo.Function("pickup", [0,1]), self.pickupdone)
        self.prg.assign_external(clingo.Function("deliver", [0,1, self.order[1], self.order[0]]), self.deliverdone)

        if self.shelf != -1:
            for shelf in self.available_shelves:
                self.prg.assign_external(clingo.Function("available", [shelf]), False)
            self.prg.assign_external(clingo.Function("available", [self.shelf]), True)

        for i in range(len(self.state)):
            for j in range(len(self.state[0])):
                self.prg.assign_external(clingo.Function("block", [i+1,j+1]), not self.state[i][j])

        with self.prg.solve(yield_=True) as h:
            for m in h:
                opt = m
            for atom in opt.symbols(shown=True):
                self.model.append(atom)
                if atom.name == "chooseShelf":
                    self.shelf = atom.arguments[0].number
                    #print("robot"+str(self.id)+" chooseShelf"+str(self.shelf))

        self.t = 0
        self.get_next_action()
        self.t = 1

    def get_next_action(self):
        next_action = False
        for atom in self.model:
            if atom.name == "move" and atom.arguments[2].number == self.t+1:
                self.next_action = atom
                self.next_pos[0] = self.pos[0] + atom.arguments[0].number
                self.next_pos[1] = self.pos[1] + atom.arguments[1].number
                next_action = True
            elif atom.name in ["pickup", "deliver", "putdown"] and atom.arguments[0].number == self.t+1:
                self.next_action = atom
                next_action = True
        if not next_action:
            self.plan_finished = True

    def action(self):
        if self.plan_finished:
            return "",[]
        else:
            action = self.next_action
            name = action.name
            args = []
            if name == "putdown":
                self.pickupdone = False
                self.deliverdone = False
                self.prg.assign_external(clingo.Function("deliver", [0,1, self.order[1], self.order[0]]), self.deliverdone)
                self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], 1, self.order[0]]), False)
                for shelf in self.available_shelves:
                    self.prg.assign_external(clingo.Function("available", [shelf]), False)
            else:
                if name == "move":
                    self.pos[0] = self.next_pos[0]
                    self.pos[1] = self.next_pos[1]
                    args = [action.arguments[0].number, action.arguments[1].number]
                elif name ==  "pickup":
                    self.pickupdone = True
                elif name == "deliver":
                    self.deliverdone = True
                    args = [self.order[0], self.order[1], 1]
            self.get_next_action()
            self.t += 1
            return name, args

    def set_order(self, id, product, station, available_shelves):
        self.shelf = -1

        self.order[0] = id
        self.order[1] = product
        self.order[2] = station
        self.available_shelves = available_shelves
        self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], 1, self.order[0]]), True)
        for shelf in self.available_shelves:
            self.prg.assign_external(clingo.Function("available", [shelf]), True)

    def update_state(self, state):
        if self.state == []:
            for i in range(len(state)):
                self.state.append([])
                for j in range(len(state[0])):
                    self.state[i].append(1)
        for i in range(len(state)):
            for j in range(len(state[0])):
                if ((i == self.pos[0]-1 and abs(j-(self.pos[1]-1)) == 1) or
                    (abs(i-(self.pos[0]-1)) == 1 and j == self.pos[1]-1)):
                    # roboter kann nur auf felder schauen auf die er sich bewegen koennte
                    self.state[i][j] = state[i][j]
                else:
                    # alle anderen positionen als frei angenommen
                    self.state[i][j] = 1

    def action_possible(self):
        # 1 = kann auf feld begehen
        # 0 = kann nicht auf feld begehen
        if self.plan_finished:
            return True
        if self.next_action.name == "move":
            # next_pos ist kein blockiertes feld
            return self.state[self.next_pos[0]-1][self.next_pos[1]-1]
        else:
            # pickup, deliver, putdown koennen immer ausgefuehrt werden
            return True
