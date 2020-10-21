# -*- coding: utf-8 -*-
from benchmarker import Benchmarker, solve
from robot import Robot, RobotSequential, RobotShortest, RobotCrossing, RobotPrioritized

import argparse
import sys
from time import time
from typing import List

import clingo


def print_error(arg: str) -> None:
    print(arg, file=sys.stderr)


class Pathfind(object):
    def __init__(self, instance: str, encoding: str, domain: str, model_output: bool, verbose: bool, benchmark: bool,
                 result_path: str, highways: bool, clingo_arguments: List[str]) -> None:
        self.instance: str = instance
        self.encoding: str = encoding
        self.domain: str = domain
        self.model_output: bool = model_output
        self.verbose: bool = verbose
        self.benchmark: bool = benchmark
        self.highwaysFlag: bool = highways
        self.clingo_arguments: List[str] = clingo_arguments

        self.benchmarker: Benchmarker = None
        if self.benchmark:
            self.init_benchmarker(instance, domain, result_path)

        self.nodes = None
        self.highways = None
        self.robots = None
        self.pickingstations = None
        self.shelves = None
        self.orders = None
        self.products = None

        self.prg = clingo.Control(self.clingo_arguments)
        self.prg.load(instance)

        self.parse_instance()

        self.t: int = 0

        # output of inits
        self.print_inits(self.get_inits())

    def parse_instance(self) -> None:
        """Reads self.instance and saves all information in the according data structures
        Also creates the Robot objects, and saves their initial positions
        """
        self.nodes = []  # [[id,x,y]]
        self.highways = []  # [[id,x,y]]
        self.robots = []  # [Robot]
        self.orders = []  # [[id,product]] in the end format is changed to [[id,product,station]]
        # temporary dict, in the end the pickingstations will be added to the elements of self.orders
        # because the atoms which specify the product for an order and the atom specifying the pickingstation
        # are separate the temporary dict is needed to merge both values later
        order_stations = {}  # key: order id, value: pickingstation id
        self.pickingstations = []  # [[id,x,y]]
        self.shelves = []  # [[id,x,y]]
        self.products = []  # [[id,shelf]]

        self.prg.ground([("base", [])])
        with self.prg.solve(yield_=True) as h:
            for m in h:
                opt = m
            for atom in opt.symbols(shown=True):
                if atom.name == "init":
                    name = atom.arguments[0].arguments[0].name
                    id = atom.arguments[0].arguments[1].number
                    if name == "node":
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.nodes.append([id, x, y])
                    elif name == "highway":
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.highways.append([id, x, y])
                    elif name == "robot":
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.init_robot(id, x, y)
                    elif name == "order":
                        if atom.arguments[1].arguments[0].name == "line":
                            product = atom.arguments[1].arguments[1].arguments[0].number
                            self.orders.append([id, product])
                        else:
                            station = atom.arguments[1].arguments[1].number
                            order_stations[id] = station
                    elif name == "pickingStation":
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.pickingstations.append([id, x, y])
                    elif name == "product":
                        shelf = atom.arguments[1].arguments[1].arguments[0].number
                        # amount
                        self.products.append([id, shelf])
                    elif name == "shelf":
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.shelves.append([id, x, y])

        # assign pickingstations to orders
        for order in self.orders:
            order.append(order_stations[order[0]])
        # now self.orders has format [[id,product,station]]

    def init_robot(self, rid: int, x: int, y: int) -> None:
        # implemented in subclasses in order to use respective robot class
        pass

    def init_benchmarker(self, instance: str, domain: str, result_path: str) -> None:
        # implemented in subclasses as the path to save benchmark results depends on the strategy
        pass

    def get_inits(self) -> List[str]:
        """Format and add all inits to list 'inits' as a string according to asprilo specifications"""
        inits = []
        for node in self.nodes:
            inits.append(
                "init(object(node," + str(node[0]) + "),value(at,(" + str(node[1]) + "," + str(node[2]) + ")))")
        for highway in self.highways:
            inits.append("init(object(highway," + str(highway[0]) + "),value(at,(" + str(highway[1]) + "," + str(
                highway[2]) + ")))")
        for station in self.pickingstations:
            inits.append("init(object(pickingStation," + str(station[0]) + "),value(at,(" + str(station[1]) + "," + str(
                station[2]) + ")))")
        for shelf in self.shelves:
            inits.append(
                "init(object(shelf," + str(shelf[0]) + "),value(at,(" + str(shelf[1]) + "," + str(shelf[2]) + ")))")
        order_stations = []  # used to make sure that only one string for the pickingstation for each order is added
        for order in self.orders:
            inits.append("init(object(order," + str(order[0]) + "),value(line,(" + str(order[1]) + ",1)))")
            # check if string with pickingstation for this order was already added
            if order[0] not in order_stations:
                order_stations.append(order[0])
                inits.append("init(object(order," + str(order[0]) + "),value(pickingStation," + str(order[2]) + "))")
        for product in self.products:
            inits.append("init(object(product," + str(product[0]) + "),value(on,(" + str(product[1]) + ",1)))")
        return inits

    def print_inits(self, inits: List[str]) -> None:
        if self.model_output:
            for atom in inits:
                print(atom + ".")

    def print_action(self, rid: int, name: str, args: List[int], t: int) -> None:
        if self.model_output:
            # for wait no atom is printed
            # in domain m pickups are not printed
            if not ((name == "wait") or (name == "pickup" and self.domain == "m")):
                txt = "occurs(object(robot," + str(rid) + "),action(" + name + ",("
                if (name == "move") or (name == "deliver"):
                    txt += str(args[0]) + "," + str(args[1])
                txt += "))," + str(t) + ")."
                print(txt)

    def print_verbose(self, arg: str) -> None:
        if self.verbose:
            print(arg, file=sys.stderr)

    def run(self) -> int:
        # implements the different strategies in the subclasses
        pass

    def solve(self, prg: clingo.Control, type: str) -> List[clingo.Symbol]:
        # helper function to solve a logic program
        if self.benchmark:
            # finds model and saves the prg stats
            return self.benchmarker.solve(prg, type)
        else:
            # just finds model
            return solve(prg)


class PathfindCentralized(Pathfind):
    def __init__(self, instance: str, encoding: str, domain: str, model_output: bool, verbose: bool, benchmark: bool,
                 result_path: str, highways: bool, clingo_arguments: List[str]) -> None:
        self.assign_prg = clingo.Control(clingo_args)
        self.model = None

        super().__init__(instance, encoding, domain, model_output, verbose, benchmark, result_path, highways,
                         clingo_arguments)

        self.assign_orders()
        self.assign_shelves()

    def init_benchmarker(self, instance: str, domain: str, result_path: str) -> None:
        self.benchmarker = Benchmarker("centralized", instance, domain, result_path)

    def assign_orders(self) -> None:
        for rid, _, _ in self.robots:
            atom = "order(" + str(self.orders[0][1]) + ", " + str(self.orders[0][2]) + ", " + str(self.orders[0][0]) + \
                   ", " + str(rid) + ")."
            self.prg.add("base", [], atom)
            self.assign_prg.add("base", [], atom)
            del self.orders[0]

    def init_robot(self, rid: int, x: int, y: int) -> None:
        self.robots.append((rid, x, y))

    def get_inits(self) -> List[str]:
        inits = super().get_inits()

        for rid, x, y in self.robots:
            inits.append("init(object(robot," + str(rid) + "),value(at,(" + str(x) + "," + str(y) + ")))")

        return inits

    def assign_shelves(self):
        self.assign_prg.load(self.instance)
        self.assign_prg.load("./encodings/goals.lp")

        self.assign_prg.ground([("base", []), ("centralized", [])])

        assignment = self.solve(self.assign_prg, "assignment")

        for atom in assignment:
            if atom.name == "goal":
                if (atom.arguments[2].number == 1) or (self.domain == "b"):
                    self.prg.add("base", [], str(atom) + ".")

    def run(self):
        self.t = 0
        self.prg.load(self.encoding)

        parts = [("base", [])]
        if self.highways:
            parts.append(("highways", []))

        self.prg.ground(parts)

        self.model = self.solve(self.prg, "solve")

        # print actions and save plan length
        for atom in self.model:
            name = atom.name
            args = []
            if self.model_output:
                if name == "move":
                    args.append(atom.arguments[0].arguments[0].number)
                    args.append(atom.arguments[0].arguments[1].number)
                    self.print_action(atom.arguments[1].number, name, args, atom.arguments[2].number)
                if name == "pickup" or name == "putdown":
                    self.print_action(atom.arguments[0].number, name, args, atom.arguments[1].number)
                if name == "deliver":
                    args.append(atom.arguments[1].number)
                    args.append(atom.arguments[0].number)
                    self.print_action(atom.arguments[2].number, name, args, atom.arguments[3].number)
            if (name == "putdown" and self.domain == "b") or (name == "pickup" and self.domain == "m"):
                if self.t < atom.arguments[1].number:
                    self.t = atom.arguments[1].number

        if self.domain == "m":
            self.t -= 1

        return self.t


class PathfindDecentralized(Pathfind):
    def __init__(self, instance: str, encoding: str, domain: str, model_output: bool, verbose: bool, benchmark: bool,
                 result_path: str, external: bool, highways: bool, clingo_arguments: List[str]) -> None:
        """Assigns initial order to the robots and plans it
        Instance is saved in data structures by helper function parse_instance
        (also generates the Robot objects)
        """
        # input parameters (needed in init)
        self.external: bool = external
        super().__init__(instance, encoding, domain, model_output, verbose, benchmark, result_path, highways,
                         clingo_arguments)

        # more initializing
        self.orders_in_delivery = []
        self.used_shelves = []

        self.init_state()

        # initialize robots
        for robot in self.robots:
            self.plan(robot)  # assigns an order and plans it

    def init_state(self) -> None:
        # initialize state matrix, saves which positions are free (1=free, 0=blocked)
        self.state = []
        for i in range(max(self.nodes, key=lambda item: item[1])[1]):  # range(x-size)
            self.state.append([])
            for j in range(max(self.nodes, key=lambda item: item[2])[2]):  # range(y-size)
                self.state[i].append(1)
        # save robot start position in self.state
        for r in self.robots:
            self.state[r.pos[0] - 1][r.pos[1] - 1] = 0

        for robot in self.robots:
            robot.update_state(self.state)

    def get_inits(self) -> List[str]:
        inits = super().get_inits()

        for robot in self.robots:
            inits.append("init(object(robot," + str(robot.id) + "),value(at,(" + str(robot.pos[0]) + "," + str(
                robot.pos[1]) + ")))")
            if robot.pickupdone:
                inits.append("init(object(robot," + str(robot.id) + "),value(carries," + str(robot.shelf) + ")))")

        return inits

    def perform_action(self, robot: Robot):
        """Performs the action of robot
        If the order is finished with this action, a new order is (assigned and) planned
        """
        name: str
        args: List[int]
        name, args = robot.action()
        if name == "":
            # robot doesn't have to do a action in this timestep because
            # 1) there aren't any more orders
            # 2) in the last timestep no order could be assigned
            # 3) robot is in a deadlock
            self.plan(robot)
        elif name == "pickup" or name == "deliver":
            self.print_action(robot.id, name, args, self.t)
            if self.domain == "b":
                self.plan(robot)
        else:
            self.print_action(robot.id, name, args, self.t)

        # check if order is finished
        if (self.domain == "m" and name == "pickup") or (self.domain == "b" and name == "putdown"):
            self.finish_order(robot.order, robot.shelf)
            robot.release_order()
            # in b domain the robot has to plan a new order
            if self.domain == "b":
                self.plan(robot)

    def assign_order(self, robot):
        """Assign the first possible order to the robot
        Return True/False if an order was assigned/wasn't assigned
        """
        # possible_shelves is used to check if an order can be completed
        possible_shelves = []
        o = -1
        possible_order = False
        # no possible order and not all orders checked
        while (not possible_order) and (o != len(self.orders) - 1):
            o += 1
            # list of all shelves which have the needed product
            possible_shelves = [shelf for [sid, shelf] in self.products if sid == self.orders[o][1]]
            # remove already used shelves
            for sid in self.used_shelves:
                if sid in possible_shelves:
                    possible_shelves.remove(sid)
            # if there are shelves the order is possible
            if possible_shelves:
                possible_order = True
        if possible_order:
            # assign the order to the robot and say which shelves can be used
            robot.set_order(self.orders[o], possible_shelves)
            # make sure the order isn't assigned again
            self.reserve_order(self.orders[o])
        return possible_order

    def finish_order(self, order, shelf):
        """Releases the shelf and removes the order from orders_in_delivery"""
        self.release_shelf(shelf)
        self.orders_in_delivery.remove(order)

    def reserve_order(self, order):
        """Adds the order to list of orders which are currently being delivered
        and removes from list of orders which are open
        """
        self.orders_in_delivery.append(order)
        self.orders.remove(order)

    def release_order(self, order):
        """Removes the order from list of orders which are currently being delivered
        and adds it again to the list of order which are open
        This function is needed for situations in which the robot is deadlocked
        in his start position and can't start the delivery of the order
        """
        self.orders_in_delivery.remove(order)
        self.orders.append(list(order))

    def reserve_shelf(self, shelf):
        """Add the shelf to the list of shelves which are already in use"""
        if shelf not in self.used_shelves:
            self.used_shelves.append(shelf)

    def release_shelf(self, shelf):
        """Removes the shelf from the list of shelves which are already in use"""
        if shelf in self.used_shelves:
            self.used_shelves.remove(shelf)

    def plan(self, robot):
        """First checks if robot has an order assigned (if not tries to assign one)
        and then calls robot.solve (if the robot already had an order assigned this
        causes the robot to replan, otherwise the plans the new order)
        """
        if robot.shelf == -1:  # robot doesn't have a order assigned
            if not self.assign_order(robot):  # try to assign a order
                return False  # no order can be assigned
            
            if self.domain == "m":
                self.print_verbose("robot" + str(robot.id) + " planning order id=" + str(robot.order[0]) + " product="
                                   + str(robot.order[1]) + " at t=" + str(self.t))
            else:
                self.print_verbose("robot" + str(robot.id) + " planning step 1 order id=" + str(robot.order[0]) +
                                   " product=" + str(robot.order[1]) + " station=" + str(robot.order[2]) + " at t=" +
                                   str(self.t))
        else:
            if robot.pickupdone and robot.current_goal == 1:
                self.print_verbose("robot" + str(robot.id) + " planning step 2 at t=" + str(self.t))
            elif robot.deliverdone and robot.current_goal == 2:
                self.print_verbose("robot" + str(robot.id) + " planning step 3 at t=" + str(self.t))
            else:
                self.print_verbose("robot" + str(robot.id) + " replanning at t=" + str(self.t))

        found_plan = robot.plan()

        if found_plan:  # if the robot found a plan the shelf has to be reserved
            self.reserve_shelf(robot.shelf)
            return True
        else:  # robot couldn't find a plan
            if robot.shelf == -1:
                # robot couldn't start planning the order (because he deadlocked in his start position)
                # release the order so that other robots can try to plan it
                self.release_order(robot.order)
            return False

    def add_wait(self, r):
        """Add a wait action to the robots plan"""
        self.print_verbose("r" + str(r.id) + " waits")
        r.wait()
        self.state[r.next_pos[0] - 1][r.next_pos[1] - 1] = 0
        
    def check_conflicts(self):
        """Finds all conflicts between robots
        and returns a list of conflicts
        """
        self.prg = clingo.Control(self.clingo_arguments)
        self.prg.load("./encodings/conflicts.lp")
        for r in self.robots:
            if r.next_action != clingo.Function("", []):
                self.prg.add("base", [], str(r.next_action) + ".")
            else:
                self.prg.add("base", [], str("wait(" + str(r.id) + ")."))
                
            self.prg.add("base", [], "position(" + str(r.id) + ",(" + str(r.pos[0]) + "," + str(r.pos[1]) + ")).")
        self.prg.ground([("base", [])])

        return self.solve(self.prg, "conflict")

    def reset_state(self):
        """Sets all values in the state array to 1 (empty)"""
        for i in range(len(self.state)):
            for j in range(len(self.state[0])):
                self.state[i][j] = 1


class PathfindDecentralizedSequential(PathfindDecentralized):
    def init_benchmarker(self, instance: str, domain: str, result_path: str) -> None:
        self.benchmarker = Benchmarker("sequential", instance, domain, result_path)

    def init_robot(self, rid: int, x: int, y: int) -> None:
        self.robots.append(RobotSequential(rid, [x, y], self.encoding, self.domain, self.instance, self.external,
                                           self.highwaysFlag, self.clingo_arguments, self.benchmark, self.benchmarker))

    def run(self):
        """Main function of Pathfind
        Starts execution of all plans and handles possible conflicts
        When robots have completed an order, they get assigned a new order
        Finishes when all orders are delivered
        """
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1
            
            for robot in self.robots:
                robot.update_state(self.state)

                for conflict in self.check_conflicts_robot(robot):
                    if conflict.name in ["swap", "conflict"]:
                        if robot.id == conflict.arguments[0].number or robot.id == conflict.arguments[1].number:
                            self.plan(robot)
                    if conflict.name == "conflictW":
                        self.plan(robot)

                self.state[robot.pos[0] - 1][robot.pos[1] - 1] = 1  # mark old position as free
                self.perform_action(robot)
                self.state[robot.pos[0] - 1][robot.pos[1] - 1] = 0  # mark new position as blocked

        if self.domain == "m":
            self.t -= 1

        return self.t

    def check_conflicts_robot(self, robot):
        """Finds all conflicts between robots
        and returns a list of conflicts
        """
        self.prg = clingo.Control(self.clingo_arguments)
        self.prg.load("./encodings/conflicts.lp")

        for r in self.robots:
            if r != robot:
                self.prg.add("base", [], str("wait(" + str(r.id) + ")."))
            else: 
                if r.next_action != clingo.Function("", []):
                    self.prg.add("base", [], str(r.next_action) + ".")

            self.prg.add("base", [], "position(" + str(r.id) + ",(" + str(r.pos[0]) + "," + str(r.pos[1]) + ")).")
        self.prg.ground([("base", [])])

        return self.solve(self.prg, "conflict")
        

class PathfindDecentralizedShortest(PathfindDecentralized):
    def __init__(self, instance: str, encoding: str, domain: str, model_output: bool, verbose: bool, benchmark: bool,
                 result_path: str, external: bool, highways: bool, clingo_arguments: List[str]) -> None:
        self.resolved = False

        super().__init__(instance, encoding, domain, model_output, verbose, benchmark, result_path, external, highways,
                         clingo_arguments)

    def init_benchmarker(self, instance: str, domain: str, result_path: str) -> None:
        self.benchmarker = Benchmarker("shortest", instance, domain, result_path)

    def init_robot(self, rid: int, x: int, y: int) -> None:
        self.robots.append(RobotShortest(rid, [x, y], self.encoding, self.domain, self.instance, self.external,
                                         self.highwaysFlag, self.clingo_arguments, self.benchmark, self.benchmarker))

    def run(self):
        """Run version using the shortest replanning conflict solving strategy
        In case of a conflict (where both robots move) both robots find a new plan
        but only the robot for which the new plan adds less time uses the new plan
        For conflicts where only one robot moves the other robot waits
        """
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1

            if self.benchmark:
                stime = 0

            # check that every robot has a plan
            for r in self.robots:
                # no order assigned or no plan because in a deadlock
                if (r.shelf == -1) or (r.next_action.name == ""):
                    r.update_state(self.state)
                    self.plan(r)

            # unmark all old positions
            self.reset_state()
            # mark all new positions
            for r in self.robots:
                self.state[r.next_pos[0] - 1][r.next_pos[1] - 1] = 0

            self.resolved = True 
            while self.resolved:  # Needs to recheck for conflicts if a robot replans
                self.resolved = False
                conflicts = self.check_conflicts()

                for conflict in conflicts:  # Conflict detection
                    self.resolved = True
                    if conflict.name in ["conflict", "swap"]:
                        # if there is a conflict the robot with the lower ID needs to replan
                        for r1 in self.robots:
                            if r1.id == conflict.arguments[0].number:
                                for r2 in self.robots:
                                    if r2.id == conflict.arguments[1].number:
                                        if conflict.name == "conflict":
                                            self.print_verbose("conflict between " + str(r1.id) + " and " + str(r2.id) +
                                                               " at t=" + str(self.t))
                                        elif conflict.name == "swap":
                                            self.print_verbose("swapping conflict between " + str(r1.id) + " and " +
                                                               str(r2.id) + " at t=" + str(self.t))
                                        
                                        # both robots move -> both have to find a new plan
                                        # self.replan returns added length of new plan
                                        dr1 = self.replan(r1)
                                        dr2 = self.replan(r2)
            
                                        # choose which robot uses new plan
                                        # case 1: both robots are deadlocked
                                        if (dr1 == -1) and (dr2 == -1):
                                            self.print_verbose("both robots deadlocked")
                                            # both have to use new plan (which causes them to wait next timestep)
                                            self.change_plan(r1)
                                            self.change_plan(r2)
            
                                        # case 2: r1 is deadlocked or the new plan of r1 adds more time
                                        # here dr2 can still be -1 -> then dr2<=dr1 would be true
                                        # therefore the condition dr2!=-1 is needed
                                        elif (dr1 == -1) or (dr2 <= dr1 and dr2 != -1):
                                            self.print_verbose("r" + str(r1.id) + " deadlocked or dr" + str(r2.id) +
                                                               "<=dr" + str(r1.id))
                                            # r1 continues using the old plan
                                            r1.use_old_plan()
                                            # r2 uses the new plan
                                            self.change_plan(r2)
            
                                        # case 3: r2 is deadlocked or the new plan of r2 adds more time
                                        elif (dr2 == -1) or (dr1 < dr2):
                                            self.print_verbose("r" + str(r2.id) + " deadlocked or dr" + str(r1.id) +
                                                               "<=dr" + str(r2.id))
                                            # r1 uses new plan
                                            self.change_plan(r1)
                                            # r2 continues using the old plan
                                            r2.use_old_plan()

                    elif conflict.name in ["conflictW", "conflictWO"]:
                        # if another robots waits or performs an action the robot should wait
                        for r in self.robots:
                            if r.id == conflict.arguments[0].number:
                                for r2 in self.robots:
                                    if r2.id == conflict.arguments[1].number:
                                        if r2.shelf != -1:
                                            self.add_wait(r)
                                        else:
                                            self.plan(r)

            # perform all next actions
            for robot in self.robots:
                if robot.next_action.name != "":
                    self.perform_action(robot)

        if self.domain == "m":
            self.t -= 1

        return self.t

    def replan(self, robot):
        """Helper function used in shortest replanning strategy
        finds a new plan and returns the added length
        Argument out for benchmarking output (to know which replannigs belong together)
        out = 0 for replannigs where only one robot replans
        out = 1/2 for first/second robot when two robots replan
        """
        robot.update_state(self.state)

        robot.find_new_plan()

        # compute how many timesteps the new plan added compared to the old plan
        # if deadlocked this is set to -1
        d = robot.plan_length - (robot.old_plan_length - (robot.t - 1)) if robot.plan_length != -1 else -1
        return d

    def change_plan(self, robot):
        """Changes the plan of the robot to the new plan
        Also adds robot to the list of robots which have to be checked for conflicts
        and marks the new next_pos of the robot
        """
        robot.use_new_plan()
        self.resolved = True
        self.state[robot.next_pos[0] - 1][robot.next_pos[1] - 1] = 0


class PathfindDecentralizedCrossing(PathfindDecentralized):
    def __init__(self, instance: str, encoding: str, domain: str, model_output: bool, verbose: bool, benchmark: bool,
                 result_path: str, external: bool, highways: bool, clingo_arguments: List[str]) -> None:
        self.resolved = False

        super().__init__(instance, encoding, domain, model_output, verbose, benchmark, result_path, external, highways,
                         clingo_arguments)

    def init_benchmarker(self, instance: str, domain: str, result_path: str) -> None:
        self.benchmarker = Benchmarker("crossing", instance, domain, result_path)

    def init_robot(self, rid: int, x: int, y: int) -> None:
        self.robots.append(RobotCrossing(rid, [x, y], self.encoding, self.domain, self.instance, self.external,
                                         self.highwaysFlag, self.clingo_arguments, self.benchmark, self.benchmarker))

    def run(self):
        """Run function using the crossing strategy
        In case of a conflict both robots look for their nearest crossing
        (a crossing is a highway connected to three other highways)
        The robot closest to a crossing dodges the other robot using the crossing
        This method is only used for swapping conflict, all other conflicts
        are solved by making one of the robots wait"""
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1
            
            for r in self.robots:
                # if the robot doesn't have a order or was in a deadlock it needs to find a new plan
                if (r.shelf == -1) or (r.next_action.name == ""):
                    r.update_state(self.state)
                    self.plan(r)
                else:
                    self.next_action_possible(r, r.next_action)

            self.resolved = True 
            while self.resolved:  # Needs to recheck for conflicts if a robot replans
                self.resolved = False
                conflicts = self.check_conflicts()
            
                for conflict in conflicts:
                    if conflict.name == "swap":  # if there is a conflict the robot with the lower ID needs to replan
                        r1 = 0
                        r2 = 0
                        for r in self.robots:
                            if r.id == conflict.arguments[0].number:
                                r1 = r
                            if r.id == conflict.arguments[1].number:
                                r2 = r
                        
                        self.print_verbose("swapping conflict between " + str(r1.id) + " and " + str(r2.id) + " at t=" +
                                           str(self.t))
                        # if r1 isn't in a conflict but r2 is already dodging another robot
                        self.resolved = True
                        if (not r1.in_conflict) and r2.dodging and (not r1.replanned):
                            # r1 will "copy" the dodging from r2 but has to make an additional step to dodge r2
                            self.add_nested_crossroad(r1, r2)
                        # analogous to the first case
                        elif r1.dodging and (not r2.in_conflict) and (not r2.replanned):
                            self.add_nested_crossroad(r2, r1)
                        # if both are already dodging another robot
                        elif r1.dodging and r2.dodging:
                            # whoever is further away from his crossing will nested dodge the other one
                            if (r1.cross_done - r1.t) < (r2.cross_done - r2.t):
                                self.add_nested_crossroad(r2, r1)
                            else:
                                self.add_nested_crossroad(r1, r2)
                        else:
                            # in all other cases both robots will look for the nearest crossroad
                            # and whoever is closest will dodge

                            # if any of the robots are already in conflict their positions will be blocked
                            # thus they can't be used as a crossing
                            # (positions of possible partner also have to be blocked)
                            self.block_crossings(r1, r2)

                            # both find their nearest crossroad
                            r1.find_crossroad()
                            r2.find_crossroad()
                            # choose who is closer to a crossroad
                            # or when one of the robots couldn't find a crossroad the other has to dodge
                            # when a robot couldn't find a crossroad its cross_length  will be -1
                            if (r1.cross_length < r2.cross_length and r1.cross_length != -1) or (r2.cross_length == -1):
                                self.add_crossroad(r1, r2)
                            else:
                                self.add_crossroad(r2, r1)
                                
                    else:  # if another robots waits or performs an action the robot should wait
                        for r in self.robots:
                            if r.id == conflict.arguments[0].number:
                                r1 = r
                            if r.id == conflict.arguments[1].number:
                                r2 = r

                        self.print_verbose("conflict between " + str(r1.id) + " and " + str(r2.id) + " at t=" +
                                           str(self.t))
                        # the robot which moves and is not already in a conflict will wait
                        if (r1.next_action.name == "move") and (not r1.in_conflict):
                            self.add_wait(r1)
                        elif (r2.next_action.name == "move") and (not r2.in_conflict):
                            self.add_wait(r2)
                        # if non fit this criterion an arbitrary robot waits
                        else:
                            # but we have to make sure that the robot actually moves
                            if r1.next_action.name == "move":
                                self.add_wait(r1)
                            # no need to check if r2 is moving
                            # (because if r1 isn't moving, r2 has to move as there is a conflict)
                            else:
                                self.add_wait(r2)

            # then perform the actions
            for robot in self.robots:
                self.perform_action(robot)
                self.state[robot.pos[0] - 1][robot.pos[1] - 1] = 1
                self.state[robot.next_pos[0] - 1][robot.next_pos[1] - 1] = 0

        if self.domain == "m":
            self.t -= 1

        return self.t

    def add_crossroad(self, r1, r2):
        """Add crossroad to plan of r1 to dodge r2
        First get in which direction to dodge
        Possibly add this to conflict partners of r1
        Change r1 (and partners) to use the crossroad"""
        self.print_verbose("r" + str(r1.id) + " dodges")

        # determine in which direction to dodge
        self.get_dodge(r1, r2)

        r1_old_partners = self.update_conflict_partners(r1, r2)

        # generate cross_model for all old_partners of r1
        # process is different depending on if r1 moves in direction of its partners (so moves on some partner) or not
        # but this is only needed if r1 had partners previously
        if r1_old_partners:
            self.print_verbose("all conflict partners of r" + str(r1.id) + " also dodge")
            changed = []
            # get the next pos of r1
            # cant use r1.next_pos because r1 doesn't use the crossroad yet
            # r1 can't use the crossroad yet because there could be the possibility that moves still have to be added
            next_pos = list(r1.pos)
            for atom in r1.cross_model:
                if atom.name == "move":
                    if atom.arguments[2].number == 1:
                        next_pos[0] += atom.arguments[0].arguments[0].number
                        next_pos[1] += atom.arguments[0].arguments[1].number
                        break

            partner_dir = False
            # check if r1 moves onto any of its partners
            for p in r1_old_partners:
                if p.pos == next_pos:
                    # in this case add_nested_dodge1 is used
                    partner_dir = True
                    if p == r2:
                        break
                    changed = self.get_nested_dodge1(p, r1, r1.cross_length, [])
                    break
            # if r1 doesn't move onto any of its partners get_nested_dodge2 will be used
            if not partner_dir:
                for p in r1_old_partners:
                    if p.next_pos == r1.pos:
                        if p == r2:
                            break
                        changed = self.get_nested_dodge2(p, r1, [], [])
                        break

            # add crossroad recursively to all partner of r1
            for p in changed:
                self.change_crossroad(p)
        # add crossroad to the model, also sets in_conflict flag for r1
        self.change_crossroad(r1)

    def get_dodge(self, r1, r2):
        """determines in which direction r1 has to dodge and add the needed move to the cross_model
        also handles the special case that r1 doesn't actually have to dodge all the way"""
        t_finished = None  # save time when r2 changes from path to crossing, r1 only has to dodge that long
        # r1.cross_model contains all possible moves off the crossing
        # the directions from which r2 is going to the crossing and off the crossing have to be removed
        move_to_cross = None
        move_from_cross = None
        for atom in r2.model:
            if atom.name == "move":
                # r2.t (the next move r2 makes) + r1.cross_length (how many steps it is to the crossing)
                if atom.arguments[2].number == r2.t + r1.cross_length:
                    move_to_cross = atom
                    if move_from_cross is not None:
                        break  # we have all directions we need so we can stop the loop
                # after going to the crossing r2 will leave the crossing in the next step -> + 1
                elif atom.arguments[2].number == r2.t + r1.cross_length + 1:
                    move_from_cross = atom
                    if move_to_cross is not None:
                        break  # we have all directions we need so we can stop the loop
                # to make sure that r2 is actually going to the crossing we have to check
                # if its moves are the same as those of r1
                # all other moves (after the first and before going off the crossing)
                # have to be the same as the previous move of r1
                if (atom.arguments[2].number > r2.t) and (atom.arguments[2].number <= (r2.t + r1.cross_length)):
                    # r1.cross_model is sorted so r1.cross_model[t-1] will be the move at time t
                    if ((atom.arguments[0].arguments[0].number !=
                         r1.cross_model[atom.arguments[2].number - r2.t - 1].arguments[0].arguments[0].number) or
                            (atom.arguments[0].arguments[1].number !=
                             r1.cross_model[atom.arguments[2].number - r2.t - 1].arguments[0].arguments[1].number)):
                        # if the move isn't the same we record the time
                        # the earliest time determines how long r1 has to dodge
                        if t_finished is None:
                            t_finished = atom.arguments[2].number - r2.t
                        elif atom.arguments[2].number - r2.t < t_finished:
                            t_finished = atom.arguments[2].number - r2.t
            elif atom.name in ["putdown", "pickup"]:
                if (atom.arguments[1].number > r2.t) and (atom.arguments[1].number <= (r2.t + r1.cross_length)):
                    if t_finished is None:
                        t_finished = atom.arguments[1].number - r2.t
                    elif atom.arguments[1].number - r2.t < t_finished:
                        t_finished = atom.arguments[1].number - r2.t
            elif atom.name == "deliver":
                if (atom.arguments[3].number > r2.t) and (atom.arguments[3].number <= (r2.t + r1.cross_length)):
                    if t_finished is None:
                        t_finished = atom.arguments[3].number - r2.t
                    elif atom.arguments[3].number - r2.t < t_finished:
                        t_finished = atom.arguments[3].number - r2.t

        filtered_model = []
        # r1 has to dodge all the way
        if t_finished is None:
            # r1.cross_model can contain up to 4 moves off the crossing
            # 2 of those will be removed because of move_to_cross and move_from_cross
            # if after that there are still 2 moves one has to be chosen, first_move keeps track of that
            first_move = True
            for atom in r1.cross_model:
                if atom.name == "move":
                    # because r1.cross_length is the time to the crossing, +1 will be the moves off the crossing
                    if atom.arguments[2].number == r1.cross_length + 1:
                        # the direction from which r2 is coming isn't added to the model
                        if ((atom.arguments[0].arguments[0].number ==
                             -1 * move_to_cross.arguments[0].arguments[0].number) and
                                (atom.arguments[0].arguments[1].number ==
                                 -1 * move_to_cross.arguments[0].arguments[1].number)):
                            continue
                        # the direction in which the r2 is moving isn't added to the model
                        elif ((atom.arguments[0].arguments[0].number ==
                               move_from_cross.arguments[0].arguments[0].number) and
                              (atom.arguments[0].arguments[1].number ==
                               move_from_cross.arguments[0].arguments[1].number)):
                            continue
                        # now at least 1 direction is left but there could be a second one
                        # the first direction will be used
                        elif first_move:
                            first_move = False
                            filtered_model.append(atom)
                    # possible other direction won't be added to the model
                    # all moves before going of the crossing stay the same
                    else:
                        filtered_model.append(atom)
        # r1 only has to dodge for t_finished steps
        else:
            for atom in r1.cross_model:
                if atom.name == "move":
                    # only steps before t_finished are added to the model
                    if atom.arguments[2].number <= t_finished:
                        filtered_model.append(atom)

        r1.cross_model = list(filtered_model)
        r1.cross_length = len(r1.cross_model)

    def add_nested_crossroad(self, r1, r2):
        """Add crossroad in case of a nested conflict
        changed saves all partners that have to do a nested dodge
        for all changed conflict partners are update and crossroad is used"""
        changed = self.get_nested_dodge1(r1, r2, None, [])

        if changed:
            r1_old_partners = self.update_conflict_partners(r1, r2)
            # add crossroad for all robots (this includes r1)
            for p in changed:
                self.change_crossroad(p)

    def get_nested_dodge1(self, r1, r2, steps_to_do, changed):
        """helper function for add_nested_crossroad
        dodging robot (r2) moves onto one of its partners (r1)
        partner (r1) has to dodge the same way as r2
        but without the first step and duplicating the last step"""
        self.print_verbose("r" + str(r1.id) + " recursively dodges")
        r1.cross_model = []
        # how many steps does r2 still have to do from its cross_model
        if steps_to_do is None:
            steps_to_do = r2.cross_done - (r2.t - 1)
        # if only one step
        if steps_to_do == 1:
            # this is then the last step, this will just be added to r1.cross_model but as the first move
            for atom in r2.cross_model:
                if atom.arguments[2].number == r2.cross_length:
                    r1.cross_model.append(clingo.Function(atom.name,
                                                          [(atom.arguments[0].arguments[0].number,
                                                            atom.arguments[0].arguments[1].number), r1.id, 1]))
        else:
            # the first move will be removed
            # all other moves will be moved to one timestep earlier
            # and the last move has to duplicated
            offset = r2.cross_length - steps_to_do + 1
            for atom in r2.cross_model:
                # the first move doesn't have to be added to r1.cross_model
                if atom.arguments[2].number > offset:
                    # last is duplicated, and moved to one timestep earlier
                    if atom.arguments[2].number == r2.cross_length:
                        r1.cross_model.append(clingo.Function(atom.name,
                                                              [(atom.arguments[0].arguments[0].number,
                                                                atom.arguments[0].arguments[1].number),
                                                               r1.id, atom.arguments[2].number - offset]))
                        r1.cross_model.append(clingo.Function(atom.name,
                                                              [(atom.arguments[0].arguments[0].number,
                                                                atom.arguments[0].arguments[1].number),
                                                               r1.id, atom.arguments[2].number - offset + 1]))
                    # other moves are just moved to one timestep earlier
                    else:
                        r1.cross_model.append(clingo.Function(atom.name,
                                                              [(atom.arguments[0].arguments[0].number,
                                                                atom.arguments[0].arguments[1].number),
                                                               r1.id, atom.arguments[2].number - offset]))
        r1.cross_length = len(r1.cross_model)

        if not self.next_action_possible(r1, r1.cross_model[0]):
            # next_action wasn't possible
            # r1 replanned
            return changed

        changed.append(r1)

        # compute the next_pos of r1
        next_pos = list(r1.pos)
        for atom in r1.cross_model:
            if atom.name == "move":
                if atom.arguments[2].number == 1:
                    next_pos[0] += atom.arguments[0].arguments[0].number
                    next_pos[1] += atom.arguments[0].arguments[1].number
                    break

        # recursively add crossroad to the next robot
        for p in r1.conflict_partners:
            if p.pos == next_pos:
                return self.get_nested_dodge1(p, r1, r1.cross_length, changed)

        return changed

    def get_nested_dodge2(self, r1, r2, prev, changed):
        """helper function for add_crossroad
        dodging robot (r2) doesn't move onto any of its partner
        the partner next to r2 is r1
        r1 has to do the same dodge
        r2 gets an additional last step (duplicate previous last step)"""
        self.print_verbose("r" + str(r1.id) + " recursively dodges2")
        # r1 will copy the cross_model of r2
        # but a new first move is added (so all other moves have to be moved one timestep back)
        # and the last move has to be duplicated for r2 (not for r1)
        # compute the first move
        move_x = r2.pos[0] - r1.pos[0]
        move_y = r2.pos[1] - r1.pos[1]
        r1.cross_model = [clingo.Function("move", [(move_x, move_y), r1.id, 1])]
        for atom in r2.cross_model:
            # copy all move with t+=1
            if atom.arguments[2].number <= r2.cross_length:
                r1.cross_model.append(clingo.Function(atom.name, [(atom.arguments[0].arguments[0].number,
                                                                   atom.arguments[0].arguments[1].number),
                                                                  r1.id, atom.arguments[2].number + 1]))
        r1.cross_length = len(r1.cross_model)

        changed.append(r1)

        # duplicate last move for r2 and all previous partners
        prev.append(r2)
        for r in prev:
            r.duplicate_last_move()

        # recursively add crossroad for the next robot
        for p in r1.conflict_partners:
            if p.next_pos == r1.pos:
                return self.get_nested_dodge2(p, r1, prev, changed)
        return changed

    def update_conflict_partners(self, r1, r2):
        """Update the conflict partners of r1 and r2 (and do so recursively for their partners)"""
        # if r2 previously not in conflict, in_conflict flag needs to be set
        if not r2.in_conflict:
            r2.set_in_conflict(r1.cross_length)

        # r1 and r2 aren't already in conflict
        if r1 not in r2.conflict_partners:
            # old conflict partners of r1 and r2 have to be saved
            r1_old_partners = dict(r1.conflict_partners)
            r2_old_partners = dict(r2.conflict_partners)
            # first update all partner of r1 and do this recursively for all partners of r1
            for p in r1_old_partners:
                r1.update_partners(p, r1.cross_length)
                for p2 in p.conflict_partners:
                    p.update_partners(p2, r1.cross_length)
            # then add all partners of r2 to r1 and do so recursively for all partners of r1
            for p in r2_old_partners:
                r1.update_partners(p, r1.cross_length)
                for p2 in r1_old_partners:
                    p2.update_partners(p, r1.cross_length)
            # add all old_partners of r1 to r2 and recursively to all partners of r2
            for p in r1_old_partners:
                r2.update_partners(p, r1.cross_length)
                for p2 in r2_old_partners:
                    p2.update_partners(p, r1.cross_length)
            # finally add each other as partners
            r1.update_partners(r2, r1.cross_length)
            for p in r1_old_partners:
                p.update_partners(r2, r1.cross_length)
            r2.update_partners(r1, r1.cross_length)
            for p in r2_old_partners:
                p.update_partners(r1, r1.cross_length)
        else:
            r1_old_partners = dict(r1.conflict_partners)
            for p in r1_old_partners:
                r1.update_partners(p, r1.cross_length)
                for p2 in p.conflict_partners:
                    p.update_partners(p2, r1.cross_length)

        return r1_old_partners

    def change_crossroad(self, r):
        """Make the robot use the crossroad
        Update position, robot has to be checked for possible new conflicts"""
        # add the crossroad to the model, also generates the returning
        self.state[r.next_pos[0] - 1][r.next_pos[1] - 1] = 1
        r.use_crossroad()
        self.resolved = True
        self.state[r.next_pos[0] - 1][r.next_pos[1] - 1] = 0

    def block_crossings(self, r1, r2):
        """In certain cases the positions of r1 and r2 can't be used as crossings"""
        blocked = []
        if r1.in_conflict or r1.replanned:
            blocked.append(list(r1.pos))
            for partner in r1.conflict_partners:
                blocked.append(list(partner.pos))
        if r2.in_conflict or r2.replanned:
            blocked.append(list(r2.pos))
            for partner in r2.conflict_partners:
                blocked.append(list(partner.pos))
        r1.block_crossings(blocked)
        r2.block_crossings(blocked)

    def next_action_possible(self, r, action):
        """Check if the next action is possible
        if not also replan"""
        possible = True
        if action.name != "move":
            return possible

        # calculate next_pos
        next_pos = [r.pos[0] + action.arguments[0].arguments[0].number,
                    r.pos[1] + action.arguments[0].arguments[1].number]

        # check is next_pos is a node
        is_node = False
        for [_, x, y] in self.nodes:
            if (next_pos[0] == x) and (next_pos[1] == y):
                is_node = True
                break
        if not is_node:
            self.print_verbose(
                "r" + str(r.id) + " would move off the field with move(" + str(action.arguments[0].arguments[0].number)
                + "," + str(action.arguments[0].arguments[1].number) + ") at t=" + str(self.t))
            possible = False

        # check for moving onto a shelf which isn't the shelf of the robot
        if possible:
            for [sid, x, y] in self.shelves:
                if (next_pos[0] == x) and (next_pos[1] == y) and (r.shelf != sid):
                    self.print_verbose("r" + str(r.id) + " would go onto shelf with move("
                                       + str(action.arguments[0].arguments[0].number) + ","
                                       + str(action.arguments[0].arguments[1].number) + ") at t=" + str(self.t))
                    possible = False
                    break

        # check for moving onto a station which isn't the one the robot needs to go to
        if possible:
            for [pid, x, y] in self.pickingstations:
                if (next_pos[0] == x) and (next_pos[1] == y) and (r.order[2] != pid):
                    self.print_verbose("r" + str(r.id) + " would go onto station with move("
                                       + str(action.arguments[0].arguments[0].number) + ","
                                       + str(action.arguments[0].arguments[1].number) + ") at t=" + str(self.t))
                    possible = False
                    break

        if not possible:
            self.state[r.next_pos[0] - 1][r.next_pos[1] - 1] = 1
            r.clear_state()
            self.plan(r)
            old_partners = r.reset_crossing()
            # delete r from conflict_partners of all partners
            for p in old_partners:
                p.conflict_partners.pop(r)
            self.state[r.next_pos[0] - 1][r.next_pos[1] - 1] = 0
        return possible


class PathfindDecentralizedPrioritized(PathfindDecentralized):
    def __init__(self, instance: str, encoding: str, domain: str, model_output: bool, verbose: bool, benchmark: bool,
                 result_path: str, external: bool, highways: bool, clingo_arguments: List[str]) -> None:
        self.performed_action: [int] = []

        super().__init__(instance, encoding, domain, model_output, verbose, benchmark, result_path, external, highways,
                         clingo_arguments)

    def init_benchmarker(self, instance: str, domain: str, result_path: str) -> None:
        self.benchmarker = Benchmarker("prioritized", instance, domain, result_path)

    def init_robot(self, rid: int, x: int, y: int) -> None:
        self.robots.append(RobotPrioritized(rid, [x, y], self.encoding, self.domain, self.instance, self.external,
                                            self.highwaysFlag, self.clingo_arguments, self.benchmark, self.benchmarker))

    def plan(self, robot: RobotPrioritized):
        self.print_verbose("planning for robot" + str(robot.id))
        # collect plans from all other robots
        for r in self.robots:
            if robot != r:
                self.print_verbose("collecting plan from robot" + str(r.id))
                # r.get_plan() get an offset as the argument
                # the offset is 1 if r has not yet performed an action in this timestep
                # (as in this case r is one step behind robot)
                # during initialisation the offset is always 0
                plan = r.get_plan(1 if (r.id not in self.performed_action and self.t > 0) else 0)
                if plan:
                    robot.add_plan(plan)
                else:
                    self.print_verbose("robot" + str(r.id) + " does not have a plan, current position (" +
                                       str(r.pos[0]) + "," + str(r.pos[1]) + ") will be blocked")
                    robot.block_pos((r.pos[0], r.pos[1]))

        super().plan(robot)

        robot.clear_additional_input()
        robot.clear_blocked_positions()

    def run(self):
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1

            for robot in self.robots:
                # perform action or find new plan
                self.perform_action(robot)
                # mark that robot performed an action
                self.performed_action.append(robot.id)

            self.performed_action = []

        if self.domain == "m":
            self.t -= 1

        return self.t


class PathfindDecentralizedTraffic(PathfindDecentralized):
    def init_benchmarker(self, instance: str, domain: str, result_path: str) -> None:
        self.benchmarker = Benchmarker("traffic", instance, domain, result_path)

    def init_robot(self, rid: int, x: int, y: int) -> None:
        self.robots.append(Robot(rid, [x, y], self.encoding, self.domain, self.instance, self.external,
                                 self.highwaysFlag, self.clingo_arguments, self.benchmark, self.benchmarker))

    def resolve_conflicts(self):
        # load encoding which computes conflicts and solves them by adding wait actions
        conflicts = self.check_conflicts()

        for conflict in conflicts:  # Conflict detection
            if conflict.name == "swap":
                print_error("Error: illegal swapping conflict detected: " + str(conflict))
                sys.exit()
            else:
                for r1 in self.robots:
                    if r1.id == conflict.arguments[0].number:
                        self.add_wait(r1)
                        
    def run(self):
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1

            for robot in self.robots:
                action = robot.next_action
                if action.name == "":
                    self.plan(robot)

            # find and solve all conflicts for the current timestep
            self.resolve_conflicts()

            # perform all actions
            for robot in self.robots:
                self.perform_action(robot)

        if self.domain == "m":
            self.t -= 1

        return self.t


if __name__ == "__main__":
    # command line arguments
    parser = argparse.ArgumentParser(description="A program for decentralized planning in the asprilo framework",
                                     usage='%(prog)s [options] instance')
    parser.add_argument("instance", help="the instance to be loaded")
    parser.add_argument("-s", "--strategy", help="conflict solving strategy to be used (default: sequential)",
                        choices=['sequential', 'shortest', 'crossing', 'prioritized', 'traffic', 'centralized'],
                        default='sequential', type=str)
    parser.add_argument("-d", "--domain", help="asprilo domain to be used (default: b)", type=str, default="b",
                        choices=["b", "m"])
    parser.add_argument("-n", "--nomodel", help="disables output of the model", default=False, action="store_true")
    parser.add_argument("-v", "--verbose", help="outputs additional information (printed to stderr)", default=False,
                        action="store_true")
    parser.add_argument("-b", "--benchmark",
                        help="benchmarking mode output will be written to './results/strategy/instance/' where strategy"
                             " is the used strategy and instance is the instance (without .lp), to use a different "
                             "directory than './results', use the option --results",
                        default=False, action="store_true")
    parser.add_argument("-r", "--results", help="use custom directory for the benchmarking results (default: "
                                                "'./results')", default='./results/', type=str)
    # external not supported anymore
    # parser.add_argument("-e", "--external", help="enables use of external atoms (only for sequential, shortest and "
    #                                              "crossing strategy)",
    #                     default=False, action="store_true")
    parser.add_argument("-H", "--Highways", help="generate highway tuples if they are not given in the instance",
                        default=False, action="store_true")
    parser.add_argument("--debug", help="enables clingo warnings", default=False, action="store_true")
    args = parser.parse_args()

    clingo_args = []
    if not args.debug:
        clingo_args.append("-Wnone")

    if args.domain == "m":
        encoding = "./encodings/pathfindDecentralized-m.lp"
    else:
        encoding = "./encodings/pathfindDecentralized.lp"

    if args.benchmark:
        ts = time()

    # Initialize the Pathfind object
    if args.strategy == 'sequential':
        pathfind = PathfindDecentralizedSequential(args.instance, encoding, args.domain, not args.nomodel, args.verbose,
                                                   args.benchmark, args.results, False, args.Highways,
                                                   clingo_args)
    elif args.strategy == 'shortest':
        pathfind = PathfindDecentralizedShortest(args.instance, encoding, args.domain, not args.nomodel, args.verbose,
                                                 args.benchmark, args.results, False, args.Highways,
                                                 clingo_args)
    elif args.strategy == 'crossing':
        pathfind = PathfindDecentralizedCrossing(args.instance, encoding, args.domain, not args.nomodel, args.verbose,
                                                 args.benchmark, args.results, False, args.Highways,
                                                 clingo_args)
    elif args.strategy == 'prioritized':
        if args.domain == "m":
            encoding = "./encodings/pathfindPrioritized-m.lp"
        else:
            encoding = "./encodings/pathfindPrioritized.lp"
        pathfind = PathfindDecentralizedPrioritized(args.instance, encoding, args.domain, not args.nomodel,
                                                    args.verbose, args.benchmark, args.results, False, args.Highways,
                                                    clingo_args)
    elif args.strategy == 'traffic':
        print_error("Warning: traffic strategy needs special instance in order to work correctly")
        pathfind = PathfindDecentralizedTraffic(args.instance, encoding, args.domain, not args.nomodel, args.verbose,
                                                args.benchmark, args.results, False, args.Highways, clingo_args)
    elif args.strategy == 'centralized':
        if args.domain == "m":
            encoding = "./encodings/pathfindCentralized-m.lp"
        else:
            encoding = "./encodings/pathfindCentralized.lp"
        pathfind = PathfindCentralized(args.instance, encoding, args.domain, not args.nomodel, args.verbose,
                                       args.benchmark, args.results, args.Highways, clingo_args)

    if args.benchmark:
        plan_length = pathfind.run()
        tf = time()
        run_time = tf - ts
        pathfind.benchmarker.output({"plan_length": plan_length, "run_time": run_time}, "main")
    else:
        pathfind.run()
