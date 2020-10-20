from benchmarker import solve

from typing import List

import clingo
import sys


class Robot(object):
    def __init__(self, rid, start, encoding, domain, instance, external, highways, clingo_arguments, benchmark,
                 benchmarker):
        """Initialize the robot:
        Data structure to save inputs
        Clingo object"""
        # inputs
        self.id = rid
        self.start = list(start)
        self.pos = list(start)

        self.goalA = (-1, -1)
        self.goalB = (-1, -1)
        self.goalC = (-1, -1)

        self.next_pos = [-1, -1]

        self.state = []  # State of the world around the robot as 2D Matrix
        self.t = -1

        self.order = [-1, -1, -1]
        self.available_shelves = []
        self.shelf = -1

        self.pickupdone = False
        self.deliverdone = False

        # solving parameters
        self.encoding = encoding
        self.domain = domain
        self.instance = instance
        self.external = external
        self.highways = highways
        self.clingo_arguments = clingo_arguments
        self.benchmark = benchmark
        self.benchmarker = benchmarker

        # when internal is used clingo object initialized before solve
        if self.external:
            self.prg = clingo.Control(self.clingo_arguments)
            self.prg.load(encoding)
            self.prg.load(instance)
            parts = [("base", []), ("decentralized", [self.id])]
            if self.highways:
                parts.append(("highways", []))
            self.prg.ground(parts)

        self.plan_finished = True
        self.waiting = False  # The robot currently does/does not need to wait

        self.model = []
        self.plan_length = -1

        self.next_action = clingo.Function("", [])

    def solve(self, prg: clingo.Control, type: str) -> List[clingo.Symbol]:
        if self.benchmark:
            return self.benchmarker.solve(prg, type)
        else:
            return solve(prg)

    def generate_goals(self) -> bool:
        self.prg_goals = clingo.Control(self.clingo_arguments)
        self.prg_goals.load(self.instance)
        self.prg_goals.load("./encodings/goals.lp")

        self.prg_goals.add("base", [], "start((" + str(self.pos[0]) + "," + str(self.pos[1]) + ")," + str(self.id) +
                           ").")
        self.prg_goals.add("base", [], "robot(" + str(self.id) + ").")

        for shelf in self.available_shelves:
            self.prg_goals.add("base", [], "available(" + str(shelf) + ").")

        self.prg_goals.add("base", [], "order(" + str(self.order[1]) + ", " + str(self.order[2]) + "," +
                           str(self.order[0]) + "," + str(self.id) + ").")

        self.prg_goals.ground([("base", [])])
        model = self.solve(self.prg_goals, "assignment")
        if model:
            for atom in model:
                if atom.name == "chooseShelf":
                    self.shelf = atom.arguments[0].number
                if atom.name == "goal":
                    if atom.arguments[2].number == 1:
                        self.goalA = (atom.arguments[0].arguments[0].number, atom.arguments[0].arguments[1].number)
                    if atom.arguments[2].number == 2:
                        self.goalB = (atom.arguments[0].arguments[0].number, atom.arguments[0].arguments[1].number)
                    if atom.arguments[2].number == 3:
                        self.goalC = (atom.arguments[0].arguments[0].number, atom.arguments[0].arguments[1].number)
        else:
            return False

        return True

    def add_inputs(self) -> bool:
        if self.external:
            # Assign externals before solving
            self.prg.assign_external(clingo.Function("start", [(self.start[0], self.start[1]), self.id]), False)
            self.prg.assign_external(clingo.Function("start", [(self.pos[0], self.pos[1]), self.id]), True)

            self.prg.assign_external(clingo.Function("pickup", [self.id, 0]), self.pickupdone)
            self.prg.assign_external(clingo.Function("deliver", [self.order[1], self.order[0], self.id, 0]),
                                     self.deliverdone)

            if self.shelf != -1:
                for shelf in self.available_shelves:
                    self.prg.assign_external(clingo.Function("available", [shelf]), False)
                self.prg.assign_external(clingo.Function("available", [self.shelf]), True)

            for i in range(len(self.state)):
                for j in range(len(self.state[0])):
                    self.prg.assign_external(clingo.Function("block", [(i + 1, j + 1)]), not self.state[i][j])
        else:  # if the flag -i is used
            # Add all externals directly as literals instead and then ground
            # assign a shelf and generate the goals
            if self.shelf == -1:
                if not self.generate_goals():
                    # no shelf could be assigned
                    return False

            self.prg = clingo.Control(self.clingo_arguments)
            self.prg.load(self.encoding)
            self.prg.load(self.instance)

            self.prg.add("base", [], "start((" + str(self.pos[0]) + "," + str(self.pos[1]) + ")," + str(self.id) + ").")

            # add the goals
            self.prg.add("base", [], "goal(" + str(self.goalA) + "," + str(self.id) + ", 1).")
            if self.pickupdone:
                self.prg.add("base", [], "pickup(" + str(self.id) + ",0).")
                self.prg.add("base", [], "goal(" + str(self.goalB) + "," + str(self.id) + ", 2).")
            if self.deliverdone:
                self.prg.add("base", [], "deliver(" + str(self.order[1]) + "," + str(self.order[0]) + "," +
                             str(self.id) + ",0).")
                self.prg.add("base", [], "goal(" + str(self.goalC) + "," + str(self.id) + ", 3).")

            for i in range(len(self.state)):
                for j in range(len(self.state[0])):
                    if not (self.state[i][j]):
                        self.prg.add("base", [], "block((" + str(i + 1) + ", " + str(j + 1) + ")).")

            self.prg.add("base", [], "available(" + str(self.shelf) + ").")

            self.prg.add("base", [], "order(" + str(self.order[1]) + ", " + str(self.order[2]) + "," +
                         str(self.order[0]) + "," + str(self.id) + ").")

        return True

    def process_model(self) -> bool:
        found_model = False

        if self.model:
            found_model = True
            for atom in self.model:
                if atom.name == "chooseShelf":
                    self.shelf = atom.arguments[0].number
                elif (atom.name == "putdown" and self.domain == "b") or (atom.name == "pickup"):
                    self.plan_length = atom.arguments[1].number
                elif atom.name == "deliver" and self.domain == "b":
                    if self.plan_length < atom.arguments[3].number:
                        self.plan_length = atom.arguments[3].number

        if not found_model:
            self.plan_length = -1
            self.next_action = clingo.Function("", [])
            if self.shelf == -1:
                if self.external:
                    self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], self.order[0],
                                                                       self.id]), False)
                    for shelf in self.available_shelves:
                        self.prg.assign_external(clingo.Function("available", [shelf]), False)
                self.available_shelves = []

        return found_model

    def find_new_plan(self):
        """Makes the robot solve for a new plan and keeps the old plan saved
        If you dont want to compare the old and new plan use solve() instead"""
        self.old_model = list(self.model)
        self.old_plan_length = self.plan_length

        self.model = []

        if not self.add_inputs():
            print("test", file=sys.stderr)
            return False

        if not self.external:
            parts = [("base", []), ("decentralizedNoExternals", [self.id])]
            if self.highways:
                parts.append(("highways", []))
            self.prg.ground(parts)

        self.start = list(self.pos)
        self.plan_finished = False

        self.model = self.solve(self.prg, "plan")

        return self.process_model()

    def use_new_plan(self):
        """Start using the new plan"""
        self.t = 0
        self.get_next_action()
        self.t = 1

    def plan(self):
        """Finds a new plan and immediately uses it (unless there is a deadlock)"""
        found_model = self.find_new_plan()
        if found_model:
            self.use_new_plan()
        return found_model

    def get_next_action(self):
        """Update the next_action variable"""
        next_action = False
        for atom in self.model:
            if atom.name == "move" and atom.arguments[2].number == self.t + 1:
                self.next_action = atom
                self.next_pos[0] = self.pos[0] + atom.arguments[0].arguments[0].number
                self.next_pos[1] = self.pos[1] + atom.arguments[0].arguments[1].number
                next_action = True
            elif (atom.name in ["pickup", "putdown"] and atom.arguments[1].number == self.t + 1) or (
                    atom.name == "deliver" and atom.arguments[3].number == self.t + 1):
                self.next_action = atom
                next_action = True
        if not next_action:
            self.plan_finished = True
            # if a robot is deadlocked we still need to know its next position to prevent conflicts
            self.next_pos = list(self.pos)  # needed for shortest_replanning strategy
            self.next_action = clingo.Function("", [])

    def action(self):
        """Make the current action
        Update state variables (and externals)"""
        if self.plan_finished:
            return "", []
        else:
            if not self.waiting:
                action = self.next_action
                name = action.name
                args = []
                if name == "putdown":
                    self.pickupdone = False
                    self.deliverdone = False
                    if self.external:
                        self.prg.assign_external(clingo.Function("deliver", [self.order[1], self.order[0], self.id, 0]),
                                                 self.deliverdone)
                else:
                    if name == "move":
                        self.pos = list(self.next_pos)
                        args = [action.arguments[0].arguments[0].number, action.arguments[0].arguments[1].number]
                    elif name == "pickup":
                        self.pickupdone = True
                    elif name == "deliver":
                        self.deliverdone = True
                        args = [self.order[0], self.order[1]]
                self.get_next_action()
                self.t += 1
                return name, args
            else:
                self.waiting = False
                self.waiting_on = []
                self.t -= 1
                self.get_next_action()
                self.t += 1
                return "wait", [self.id]

    def wait(self):
        """Make a wait action in the next action"""
        self.waiting = True
        self.next_action = clingo.Function("wait", [self.id])
        self.next_pos = list(self.pos)

    def set_order(self, order, available_shelves):
        """Set order to be used as the next input in solving"""
        self.shelf = -1

        self.order = list(order)
        self.available_shelves = list(available_shelves)
        if self.external:
            self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], self.order[0], self.id]),
                                     True)
            for shelf in self.available_shelves:
                self.prg.assign_external(clingo.Function("available", [shelf]), True)

    def release_order(self):
        """Deactivate external for current order"""
        if self.external:
            self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], self.order[0], self.id]),
                                     False)
            for shelf in self.available_shelves:
                self.prg.assign_external(clingo.Function("available", [shelf]), False)
        self.shelf = -1

    def update_state(self, state):
        """Update the state matrix
        Robot can only look onto the positions it can move onto
        All other positions are assumed to be free"""
        if not self.state:
            for i in range(len(state)):
                self.state.append([])
                for j in range(len(state[0])):
                    self.state[i].append(1)
        for i in range(len(state)):
            for j in range(len(state[0])):
                if ((i == self.pos[0] - 1 and abs(j - (self.pos[1] - 1)) == 1) or
                        (abs(i - (self.pos[0] - 1)) == 1 and j == self.pos[1] - 1)):
                    # robot can only see at positions that he can look at
                    self.state[i][j] = state[i][j]
                else:
                    # All other positions are considered free
                    self.state[i][j] = 1
                    
    def set_goals(self):
        self.prg_goals = clingo.Control(self.clingo_arguments)
        self.prg_goals.load(self.instance)
        self.prg_goals.load("./encodings/goals.lp")
        
        
        self.prg_goals.add("base", [], "start((" + str(self.pos[0]) + "," + str(self.pos[1]) + ")," + str(self.id) + ").")
        self.prg_goals.add("base", [], "robot(" + str(self.id) + ").")
        for shelf in self.available_shelves:
            self.prg_goals.add("base", [], "available(" + str(shelf) + ").")
        
        self.prg_goals.add("base", [], "order(" + str(self.order[1]) + ", " + str(self.order[2]) + "," + str(
            self.order[0]) + "," + str(self.id) + ").")
        
        self.prg_goals.ground([("base", [])])
        found_model_goals = False
        with self.prg_goals.solve(yield_=True) as h:
            for m in h:
                found_model_goals = True
                opt = m
            if found_model_goals:
                for atom in opt.symbols(shown=True):
                    if atom.name == "chooseShelf":
                        self.shelf = atom.arguments[0].number
                    if atom.name == "goal":
                        if atom.arguments[2].number == 1:
                            self.goalA = (atom.arguments[0].arguments[0].number,atom.arguments[0].arguments[1].number)
                        if atom.arguments[2].number == 2:
                            self.goalB = (atom.arguments[0].arguments[0].number,atom.arguments[0].arguments[1].number)
                        if atom.arguments[2].number == 3:
                            self.goalC = (atom.arguments[0].arguments[0].number,atom.arguments[0].arguments[1].number)


class RobotSequential(Robot):
    def action_possible(self):
        """Determine if next action is possible"""
        # 1 = position is free
        # 0 = position is blocked
        if self.plan_finished:
            return True
        if self.next_action.name == "":
            return False
        if self.next_action.name == "move":
            # next_pos is not a blocked position
            return self.state[self.next_pos[0] - 1][self.next_pos[1] - 1]
        else:
            # pickup, deliver, putdown can always be done
            return True


class RobotShortest(Robot):
    def use_old_plan(self):
        """Continue using the old plan
        Used when new plan of other robot in conflict better
        or there is a deadlock with the new plan"""
        self.model = list(self.old_model)
        self.plan_length = self.old_plan_length

        self.t -= 1
        self.get_next_action()
        self.t += 1


class RobotCrossing(Robot):
    def __init__(self, rid, start, encoding, domain, instance, external, highways, clingo_arguments, benchmark,
                 benchmarker):
        super().__init__(rid, start, encoding, domain, instance, external, highways, clingo_arguments, benchmark,
                         benchmarker)

        """Additional initialization for crossing strategy"""
        self.using_crossroad = False
        self.crossroad = None
        self.cross_model = []
        self.cross_length = -1

        self.in_conflict = False
        self.dodging = False
        self.cross_done = -1
        self.blocked_crossings = []
        self.conflict_partners = {}
        self.waiting_on = []

        self.replanned = False

        self.crossroad_encoding = "./encodings/crossroad.lp"

        if self.external:
            self.crossroad = clingo.Control(self.clingo_arguments)
            self.crossroad.load(self.crossroad_encoding)
            self.crossroad.load(self.instance)
            self.crossroad.ground([("base", []), ("external", [self.id])])

    def action(self):
        if self.cross_done == self.t:
            self.in_conflict = False
            self.dodging = False
            self.cross_done = -1
        new_partners = dict(self.conflict_partners)
        for partner in self.conflict_partners:
            if self.t == self.conflict_partners[partner]:
                new_partners.pop(partner)
        self.conflict_partners = new_partners
        if self.replanned and not self.waiting:
            self.replanned = False

        return super().action()

    def find_crossroad(self):
        """Find the nearest crossroad
        Similar to solve but with crossroad encoding"""
        if self.external:
            self.crossroad.assign_external(clingo.Function("start", [(self.pos[0], self.pos[1]), self.id]), True)
            for cross in self.blocked_crossings:
                self.crossroad.assign_external(clingo.Function("block", [(cross[0], cross[1])]), True)
        else:
            self.crossroad = clingo.Control(self.clingo_arguments)
            self.crossroad.load(self.crossroad_encoding)
            self.crossroad.load(self.instance)
            self.crossroad.add("base", [], "start((" + str(self.pos[0]) + "," + str(self.pos[1]) + ")," + str(self.id) +
                               ").")
            for cross in self.blocked_crossings:
                self.crossroad.add("base", [], "block((" + str(cross[0]) + ", " + str(cross[1]) + ")).")
            self.crossroad.ground([("base", []), ("noExternal", [self.id])])

        self.cross_model = self.solve(self.crossroad, "conflict")
        if self.cross_model:
            for atom in self.cross_model:
                if atom.name == "goal":
                    self.cross_length = atom.arguments[0].number
                    self.cross_model.remove(atom)
        else:
            self.cross_length = -1

        # cross_model needs to be sorted in ascending order
        self.cross_model.sort(key=lambda atom: atom.arguments[2].number)

        if self.external:
            self.crossroad.assign_external(clingo.Function("start", [(self.pos[0], self.pos[1]), self.id]), False)
            for cross in self.blocked_crossings:
                self.crossroad.assign_external(clingo.Function("block", [(cross[0], cross[1])]), False)

    def reset_crossing(self):
        """Reinitialize all crossing related variables"""
        self.cross_model = []
        self.replanned = True
        # set in_conflict flag to False and remove all conflict_partners
        self.in_conflict = False
        self.dodging = False
        self.cross_done = -1
        old_partners = list(self.conflict_partners.keys())
        self.conflict_partners = {}

        return old_partners

    def duplicate_last_move(self):
        """Duplicate the last move of the dodging (used in nested conflicts)"""
        last_move = self.cross_model[self.cross_length - 1]
        self.cross_model.append(clingo.Function(last_move.name,
                                                [(last_move.arguments[0].arguments[0].number,
                                                  last_move.arguments[0].arguments[1].number),
                                                 last_move.arguments[1].number, last_move.arguments[2].number + 1]))
        self.cross_length += 1

    def set_in_conflict(self, t_conflict):
        """Set in_conflict flag and update cross_done"""
        self.in_conflict = True
        if self.cross_done == -1:
            self.cross_done = self.t - 1
        self.cross_done += t_conflict + 1

    def update_partners(self, partner, t_conflict):
        """Add new partner or update already existing partner"""
        # update conflict_partners dict
        # key: robot object (the conflict partner)
        # value: how long is the robot conflict partner (as timestep)
        if partner not in self.conflict_partners:
            self.conflict_partners[partner] = self.t - 1
        self.conflict_partners[partner] += t_conflict

    def use_crossroad(self):
        """Start using the crossroad model
        First the returning part has to be generated
        and then both parts have to be added to the normal model"""
        if not self.cross_model:
            return

        self.in_conflict = True
        if not self.dodging:
            self.cross_done = -1
        self.dodging = True
        if self.cross_done == -1:
            self.cross_done = self.t - 1
        self.cross_done += self.cross_length

        # total time added by dodging
        total_t = 2 * self.cross_length

        # generate part of the model for returning from crossing
        return_model = []
        for atom in self.cross_model:
            if atom.name == "move":
                # invert directions and time
                return_model.append(clingo.Function("move",
                                                    [(-1 * atom.arguments[0].arguments[0].number,
                                                      -1 * atom.arguments[0].arguments[1].number),
                                                     atom.arguments[1].number,
                                                     total_t - (atom.arguments[2].number - 1)]))

        # merge cross_model and model to new_model
        new_model = []
        # first make space for cross_model
        for atom in self.model:
            if atom.name == "move":
                # already done steps can be copied
                if atom.arguments[2].number < self.t:
                    new_model.append(atom)
                # steps in future have to be move back total_t timesteps
                else:
                    new_model.append(clingo.Function(atom.name, [(atom.arguments[0].arguments[0].number,
                                                                  atom.arguments[0].arguments[1].number),
                                                                 atom.arguments[1].number,
                                                                 atom.arguments[2].number + total_t]))
            # same thing for all other action atoms
            elif atom.name in ["pickup", "putdown"]:
                if atom.arguments[1].number < self.t:
                    new_model.append(atom)
                else:
                    new_model.append(
                        clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number + total_t]))
            elif atom.name == "deliver":
                if atom.arguments[3].number < self.t:
                    new_model.append(atom)
                else:
                    new_model.append(clingo.Function(atom.name,
                                                     [atom.arguments[0].number, atom.arguments[1].number,
                                                      atom.arguments[2].number, atom.arguments[3].number + total_t]))
            else:
                new_model.append(atom)

        # add cross_model
        for atom in self.cross_model:
            new_model.append(clingo.Function(atom.name, [(atom.arguments[0].arguments[0].number,
                                                          atom.arguments[0].arguments[1].number),
                                                         atom.arguments[1].number,
                                                         atom.arguments[2].number + self.t - 1]))
        for atom in return_model:
            new_model.append(clingo.Function(atom.name, [(atom.arguments[0].arguments[0].number,
                                                          atom.arguments[0].arguments[1].number),
                                                         atom.arguments[1].number,
                                                         atom.arguments[2].number + self.t - 1]))

        self.model = new_model
        self.t -= 1
        self.get_next_action()
        self.t += 1

    def block_crossings(self, crossings):
        """Save the blocked crossing which will be used as input in the next find_crossroad call"""
        self.blocked_crossings = list(crossings)

    def clear_state(self):
        """Reset the state matrix"""
        # mark all positions as free
        for i in range(len(self.state)):
            for j in range(len(self.state[0])):
                self.state[i][j] = 1


class RobotPrioritized(Robot):
    def __init__(self, rid, start, encoding, domain, instance, external, highways, clingo_arguments, benchmark,
                 benchmarker):
        super().__init__(rid, start, encoding, domain, instance, external, highways, clingo_arguments, benchmark,
                         benchmarker)

        self.additional_inputs = []
        self.blocked_positions = []

    def plan(self):
        # similar to Robot.solve() / Robot.find_new_plan()
        # but needs to add the additional input to the program
        # and clear additional inputs after solving
        self.model = []
        
        self.add_inputs()

        for atom in self.additional_inputs:
            self.prg.add("base", [], atom)

        for pos in self.blocked_positions:
            self.prg.add("base", [], "blockAll(" + str(pos) + ").")

        parts = [("base", []), ("decentralizedNoExternals", [self.id])]
        if self.highways:
            parts.append(("highways", []))
        self.prg.ground(parts)

        self.start = list(self.pos)
        self.plan_finished = False

        self.model = self.solve(self.prg, "plan")

        self.process_model()

        if self.process_model():
            self.t = 0
            self.get_next_action()
            self.t = 1
            return True
        else:
            return False

    def get_plan(self, offset):
        # returns all pos and move atoms from current timestep on
        # and maps the timesteps of these atoms
        # i.e. such that the next action is t=1 and so on
        plan = []
        for atom in self.model:
            if (atom.name == "pos" and atom.arguments[2].number >= self.t + offset - 1) or (
                    atom.name == "move" and atom.arguments[2].number >= self.t + offset):
                plan.append(atom.name + "((" + str(atom.arguments[0].arguments[0]) + "," +
                            str(atom.arguments[0].arguments[1]) + ")," + str(atom.arguments[1].number) + "," +
                            str(atom.arguments[2].number - (self.t + offset - 1)) + ").")

        return plan

    def add_plan(self, plan):
        self.additional_inputs += plan

    def clear_additional_input(self):
        self.additional_inputs = []

    def block_pos(self, pos):
        self.blocked_positions.append(pos)

    def clear_blocked_positions(self):
        self.blocked_positions = []
