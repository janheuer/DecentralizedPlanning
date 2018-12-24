# -*- coding: utf-8 -*-

import clingo
import simulatorCostum # needs simulator.py and network.py from visualizer/scripts
import printer
import robot
from time import time
import argparse

class Pathfind(object):
    def __init__(self, instance, encoding, output, benchmark, solve_output, port):
        self.output = output
        self.benchmark = benchmark
        self.solve_output = solve_output
        self.ground_times = []
        self.solve_times = []
        self.resolve_times = []
        # save instance in the data strctres and create robot objects
        self.instance = instance
        self.encoding = encoding
        self.prg = clingo.Control()
        self.prg.load(instance)
        if self.benchmark:
            ts = time()
        self.prg.ground([("base", [])])
        if self.benchmark:
            tf = time()
            t = tf-ts
            self.ground_times.append(t)
            print("PGt=%s," %(t)), # Parsing Ground time
        self.state = [] # to save the positions of the robots
        if self.benchmark:
            ts = time()
        self.parse_instance()
        if self.benchmark:
            tf = time()
            t = tf-ts
            print("Pt=%s," %(t)), # Parse time
        # nodes: [[id, x, y]]
        # highways: [[id, x, y]]
        # robots: [robot]
        # orders: [[id, product, pickingStation]]
        # pickingstations: [[id, x, y]]
        # shelves: [[id, x, y]]
        # products: [[id, shelf]]
        self.orders_in_delivery = []
        self.used_shelves = []

        self.t = 0

        # initialize output
        if self.output == "viz":
            self.sim = simulatorCostum.SimulatorCostum(port)
        elif self.output == "print":
            self.sim = printer.Printer()

        if self.output is not None:
            self.sim.add_inits(self.get_inits())

        # initialize robots
        for robot in self.robots:
            robot.update_state(self.state)
            self.plan(robot)

    def run(self):
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1
            for robot in self.robots:
                robot.update_state(self.state)
                if robot.action_possible():
                    self.perform_action(robot)
                else:
                    if self.plan(robot):
                        self.perform_action(robot)

        # start visualizer
        if self.output == "viz":
            self.sim.run(self.t)
        if benchmark:
            print("Tpl="+str(self.t)+","), # Total plan length

    # new run version for different conflict handling strategies
    def run2(self):
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1
            # get list of all conflicts
            conflicts = self.get_conflicts()

            # TODO: implement a method for each strategy
            self.solve_conflicts(conflicts)
                # handle all conflicts
                # (i.e. choose one of robot that has to replan for each conflict pair)
                # at the end of the function the next action of every robot can be performed without conflicts
                # possible problems: -conflicts with more than two robots
                #                    -new conflicts after replanning

            for robot in self.robots:
                self.perform_action(robot)

        # start visualizer
        if self.output == "viz":
            self.sim.run(self.t)
        if benchmark:
            print("Tpl="+str(self.t)+","), # Total plan length

    def get_conflicts(self):
        # returns list of tuples of robots which have a conflict
        conflicts = []
        for i,r1 in enumerate(self.robots[:-1]):
            for r2 in self.robots[i+1:]:
                # robots want to move onto same pos or robots want to swap pos
                if (r1.next_pos == r2.next_pos) or
                   ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
                    conflicts.append((r1,r2))
        return conflicts

    def parse_instance(self):
        self.nodes = []
        self.highways = []
        self.robots = []
        self.orders = []
        order_stations = {}
        self.pickingstations = []
        self.shelves = []
        self.products = []

        if self.benchmark:
                ts = time()
        with self.prg.solve(yield_=True) as h:
            # choose optimal model
            for m in h:
                opt = m
            for atom in opt.symbols(shown=True):
                if atom.name == "init":
                    name = atom.arguments[0].arguments[0].name
                    if name == "node":
                        id = atom.arguments[0].arguments[1].number
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.nodes.append([id, x, y])
                    elif name == "highway":
                        id = atom.arguments[0].arguments[1].number
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.highways.append([id, x, y])
                    elif name == "robot":
                        id = atom.arguments[0].arguments[1].number
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        if self.benchmark:
                            ts = time()
                        self.robots.append(robot.Robot(id, [x,y], self.encoding, self.instance))
                        if self.benchmark:
                            tf = time()
                            t = tf-ts
                            self.ground_times.append(t)
                            print("Igt=%s," %(t)), # Init Ground time
                    elif name == "order":
                        id = atom.arguments[0].arguments[1].number
                        if atom.arguments[1].arguments[0].name == "line":
                            product = atom.arguments[1].arguments[1].arguments[0].number
                            self.orders.append([id,product])
                        else:
                            station = atom.arguments[1].arguments[1].number
                            order_stations[id] = station
                    elif name == "pickingStation":
                        id = atom.arguments[0].arguments[1].number
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.pickingstations.append([id,x,y])
                    elif name == "product":
                        id = atom.arguments[0].arguments[1].number
                        shelf = atom.arguments[1].arguments[1].arguments[0].number
                        # amount
                        self.products.append([id,shelf])
                    elif name == "shelf":
                        id = atom.arguments[0].arguments[1].number
                        x = atom.arguments[1].arguments[1].arguments[0].number
                        y = atom.arguments[1].arguments[1].arguments[1].number
                        self.shelves.append([id,x,y])

        if self.benchmark:
            tf = time()
            t = tf-ts
            print("PSt=%s," %(t)), # Parsing solve time

        # assign pickingstations to orders
        for order in self.orders:
            order.append(order_stations[order[0]])

        # update self.state
        for i in range(max(self.nodes, key=lambda item:item[1])[1]):
            self.state.append([])
            for j in range(max(self.nodes, key=lambda item:item[2])[2]):
                self.state[i].append(1)

        for r in self.robots:
            self.state[r.pos[0]-1][r.pos[1]-1] = 0

    def assign_order(self, robot):
        # choose first order which can be completed
        possible_shelves = []
        o = -1
        possible_order = False
        # no possible order and not all orders checked
        while (not possible_order) and (o != len(self.orders)-1):
            o += 1
            # list of all shelves which have the needed product
            possible_shelves = [shelf for [id, shelf] in self.products if id == self.orders[o][1]]
            # remove already used shelves
            for id in self.used_shelves:
                if id in possible_shelves:
                    possible_shelves.remove(id)
            # if there are shelves the order is possible
            if possible_shelves != []:
                possible_order = True
        if possible_order:
            robot.set_order(self.orders[o], possible_shelves)
            self.reserve_order(self.orders[o])
        return possible_order

    def perform_action(self, robot):
        # mark old position as free
        self.state[robot.pos[0]-1][robot.pos[1]-1] = 1
        name, args = robot.action()
        if name == "":
            # robot doesn't have to do a action in this timestep because
            # 1) there aren't any more orders
            # 2) in the last timestep no order could be assigned
            # 3) robot was in a deadlock in the last timestep
            self.plan(robot)
        if (self.output is not None) and (name != ""):
            self.sim.add(robot.id, name, args, self.t)
        if name == "putdown":
            # order is finished
            self.finish_order(robot.order, robot.shelf)
            robot.release_order()
            # plan a new order
            self.plan(robot)
        # mark new position
        self.state[robot.pos[0]-1][robot.pos[1]-1] = 0

    def finish_order(self, order, shelf):
        # shelf can be used again
        self.release_shelf(shelf)
        # order was delivered
        self.orders_in_delivery.remove(order)

    def release_order(self, order):
        # order not in delivery
        self.orders_in_delivery.remove(order)
        # order wasn't completed
        self.orders.append(list(order))

    def reserve_order(self, order):
        # order now in delivery
        self.orders_in_delivery.append(order)
        # remove from still open orders
        self.orders.remove(order)

    def reserve_shelf(self, shelf):
        if shelf not in self.used_shelves:
            self.used_shelves.append(shelf)

    def release_shelf(self, shelf):
        if shelf in self.used_shelves:
            self.used_shelves.remove(shelf)

    def plan(self, robot):
        if robot.shelf == -1: # robot doesn't have a order assigned
            resolve = False
            if not self.assign_order(robot): # try to assign a order
                return False
            if self.solve_output:
                print("robot"+str(robot.id)+" planning order id="+str(robot.order[0])+" product="+str(robot.order[1])+" station="+str(robot.order[2])+" at t="+str(self.t))
        else:
            resolve = True
            if self.solve_output:
                print("robot"+str(robot.id)+" replanning at t="+str(self.t))

        if self.benchmark:
            ts = time()
        found_plan = robot.solve()
        if self.benchmark:
            tf = time()
            t = tf-ts
            self.solve_times.append(t)
            if resolve:
                self.resolve_times.append(t)
                print("Rst=%s," %(t)),
            else:
                print("St=%s," %(t)),
            print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ","),

        if found_plan: # if the robot found a plan the shelf has to be reserved
            self.reserve_shelf(robot.shelf)
            return True
        else: # robot couldn't find a plan
            if robot.shelf == -1:
                # robot couldn't start planning the order (because he is in a deadlock)
                # release the order so that other robots can try to plan it
                self.release_order(robot.order)
            return False

    def get_inits(self):
        # alle inits werden als string zur liste inits hinzugefuegt, diese wird dem simulator uebergebn
        inits = []
        for node in self.nodes:
            inits.append("init(object(node,"+str(node[0])+"),value(at,("+str(node[1])+","+str(node[2])+")))")
        for highway in self.highways:
            inits.append("init(object(highway,"+str(highway[0])+"),value(at,("+str(highway[1])+","+str(highway[2])+")))")
        for robot in self.robots:
            inits.append("init(object(robot,"+str(robot.id)+"),value(at,("+str(robot.pos[0])+","+str(robot.pos[1])+")))")
            if robot.pickupdone:
                inits.append("init(object(robot,"+str(robot.id)+"),value(carries,"+str(robot.shelf)+")))")
        for station in self.pickingstations:
            inits.append("init(object(pickingStation,"+str(station[0])+"),value(at,("+str(station[1])+","+str(station[2])+")))")
        for shelf in self.shelves:
            inits.append("init(object(shelf,"+str(shelf[0])+"),value(at,("+str(shelf[1])+","+str(shelf[2])+")))")
        order_stations = []
        for order in self.orders:
            inits.append("init(object(order,"+str(order[0])+"),value(line,("+str(order[1])+",1)))")
            # zuordnung zu pickingstation nur einmal uebergeben
            if order[0] not in order_stations:
                order_stations.append(order[0])
                inits.append("init(object(order,"+str(order[0])+"),value(pickingStation,"+str(order[2])+"))")
        for product in self.products:
            inits.append("init(object(product,"+str(product[0])+"),value(on,("+str(product[1])+",1)))")
        return inits

if __name__ == "__main__":
    # zu messen:
    # zeit init + run + gesmatzeit
    # zeit grouden in pathfind + grounden in robot + gesmatzeit
    # zeit initiales solven + gesamtzeit
    # zeit resolving + gesamtzeit + anzahl resolving
    # gesamtzeit des plans

    parser = argparse.ArgumentParser()
    parser.add_argument("instance", help="the instance to be loaded")
    parser.add_argument("-b", "--benchmark", help="use benchmark output (turns off all other output)", default=False, action="store_true")
    parser.add_argument("-o", "--output", help="output mode (default=print)", choices=["print", "viz"], default="print")
    parser.add_argument("-n", "--nosolve", help="turns off output when a robot solves/resolves", default=False, action="store_true")
    parser.add_argument("-p", "--port", help="port to be used when connecting to the visualizer (default=5001)", default=5001, type=int)
    args = parser.parse_args()

    benchmark = args.benchmark
    output = args.output if not benchmark else None
    solve_output = not args.nosolve if not benchmark else False

    if benchmark:
        t1 = time()
    pathfind = Pathfind(args.instance, './pathfind.lp', output, benchmark, solve_output, args.port)
    if benchmark:
        t2 = time()
        initTime = t2-t1
        print("It=%s," %(initTime)), # Initial time

    if benchmark:
        groundTime = 0
        for t in pathfind.ground_times:
            groundTime += t
        print("Tgt=%s," %(groundTime)), #Total ground time

        solveTimeInit = 0
        for t in pathfind.solve_times:
            solveTimeInit += t
        print("TstI=%s," %(solveTimeInit)), # Total solve time in Init
        pathfind.solve_times = []

    if benchmark:
        t1 = time()
    pathfind.run()
    if benchmark:
        t2 = time()
        runTime = t2-t1
        print("Rt=%s," %(runTime)), # Run time

    if benchmark:
        solveTimeRun = 0
        for t in pathfind.solve_times:
            solveTimeRun += t
        print("TstR=%s," %(solveTimeRun)), # Total solve time in run
        print("Tst=%s," %(solveTimeInit+solveTimeRun)), # Total solve time

        resolveTime = 0
        for t in pathfind.resolve_times:
            resolveTime += t
        print("Trst=%s," %(resolveTime)), # Total resolve time
        print("Tt=%s," %(initTime+runTime)), # Total time
