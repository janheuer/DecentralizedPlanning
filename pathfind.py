# -*- coding: utf-8 -*-

# Programm aufgeteielt in pathfind.py + robot.py
# -> nicht mehr über clingo aufrufen sondern nur mit python
# Fuer anzeigen im visualizer: simulatorCostum.py (-> braucht die Dateien simulator.py und network.py aus visualizer/scripts)
# Im visualizer starten ueber: Network->Initialize simulator, dort kommando zum starten des programms eingeben (also python ./pathfind.py) + port 5001
# Alternativ: Ausgaben durch print mit printer.py (-> dafuer zeile 29 auskommentieren und dafuer zeile 30 nutzen)

import clingo
import simulatorCostum
import printer
import robot
from time import time
import argparse

class Pathfind(object):
    def __init__(self, instance, encoding, output, benchmark):
        self.output = output
        self.benchmark = benchmark
        self.ground_times = []
        self.solve_times = []
        self.resolve_times = []
        # speichern der instanz in entsprechenden datenstrukturen + erstellen der roboter
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
        self.state = [] # zum speichern auf welchen positionen roboter stehen
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

        # initialisieren des simulators
        if self.output == "viz":
            self.sim = simulatorCostum.SimulatorCostum()
        elif self.output == "print":
            self.sim = printer.Printer() # keine verwendung des visualizers, statt dessen print ausgaben

        if not benchmark:
            if self.output is not None:
                self.sim.add_inits(self.get_inits())

        # initialisieren der roboter
        for robot in self.robots:
            robot.update_state(self.state)
            if self.assign_order(robot):
                if not self.benchmark:
                    print("robot"+str(robot.id)+" planning order id="+str(robot.order[0])+" product="+str(robot.order[1])+" station="+str(robot.order[2])+" at t="+str(self.t))
                if self.benchmark:
                    ts = time()
                robot.solve()
                if self.benchmark:
                    tf = time()
                    t = tf-ts
                    self.solve_times.append(t)
                    print("Ist=%s," %(t)), # Init Solve time
                    print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ","),
                self.used_shelves.append(robot.shelf)

    def run(self):
        while self.orders != [] or self.orders_in_delivery != []:
            self.t += 1
            for robot in self.robots:
                robot.update_state(self.state)
                if robot.action_possible():
                    self.perform_action(robot)
                else:
                    # todo: wenn roboter auf pickingstation bewegen will, neu solven nicht sinnvoll
                    # statdessen warten bis pickingstation wieder frei
                    if not self.benchmark:
                        print("robot"+str(robot.id)+" replanning at t="+str(self.t))
                    if self.benchmark:
                        ts = time()
                    robot.solve()
                    if self.benchmark:
                        tf = time()
                        t = tf-ts
                        self.solve_times.append(t)
                        print("Rst=%s," %(t)), # Resolve time
                        print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ","),
                    self.perform_action(robot)
        # visualization starten
        if self.output == "viz":
            self.sim.run(self.t)
        if benchmark:
            print("Tpl="+str(self.t)+","), # Total plan length

    def parse_instance(self):
        self.nodes = []
        self.highways = []
        self.robots = []
        self.orders = []
        order_stations = {} # zum zuordnen der orders zu pickingstation benoetigt
        self.pickingstations = []
        self.shelves = []
        self.products = []

        if self.benchmark:
                ts = time()
        with self.prg.solve(yield_=True) as h:
            # optimales modell auswaehlen
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

        # pickingstations zu orders zuteilen
        for order in self.orders:
            order.append(order_stations[order[0]])

        # self.state initialisieren
        for i in range(max(self.nodes, key=lambda item:item[1])[1]):
            self.state.append([])
            for j in range(max(self.nodes, key=lambda item:item[2])[2]):
                self.state[i].append(1)

        for r in self.robots:
            self.state[r.pos[0]-1][r.pos[1]-1] = 0

    def assign_order(self, robot):
        possible_shelves = []

        # auswählen der ersten ausführbaren order
        o = -1
        possible_order = False
        while not possible_order:
            o += 1
            # alle shleves die das produkt haben
            possible_shelves = [shelf for [id, shelf] in self.products if id == self.orders[o][1]]
            # entfernen der shelves die bereits benutzt werden
            for id in self.used_shelves:
                if id in possible_shelves:
                    possible_shelves.remove(id)
            # wenn es mögliche shleves gibt ist order ausführbar
            if possible_shelves != []:
                possible_order = True
            # order ist nicht ausführbar -> gibt es noch nicht überprüfte orders?
            elif o == len(self.orders)-1:
                # falls keine weiteren orders vorhanden sind kann keine order zegeordnet werden
                break
        if possible_order:
            robot.set_order(self.orders[o][0], self.orders[o][1], self.orders[o][2], possible_shelves)
            self.orders_in_delivery.append(self.orders[o])
            self.orders.remove(self.orders[o])
        return possible_order

    def perform_action(self, robot):
        self.state[robot.pos[0]-1][robot.pos[1]-1] = 1
        name, args = robot.action()
        if name == "":
            # roboter hat in diesem timestep nichts zu tun:
            # 1) es sind keine orders mehr vorhanden
            # oder
            # 2) im letzen timestep konnte keine order zugeteielt werden
            if self.orders != []:
                if self.assign_order(robot): # falls order assignd wurde neuen plan finden
                    if not self.benchmark:
                        print("robot"+str(robot.id)+" planning order id="+str(robot.order[0])+" product="+str(robot.order[1])+" station="+str(robot.order[2])+" at t="+str(self.t))
                    if self.benchmark:
                        ts = time()
                    robot.solve()
                    self.used_shelves.append(robot.shelf)
                    if self.benchmark:
                        tf = time()
                        t = tf-ts
                        self.solve_times.append(t)
                        print("St=%s," %(t)), # solve time
        if (self.output is not None) and (name!=""):
            if not benchmark:
                self.sim.add(robot.id, name, args, self.t)
        if name == "putdown":
            self.update_orders(robot.order, robot.shelf)
            if self.orders != []:
                if self.assign_order(robot):
                    if not self.benchmark:
                        print("robot"+str(robot.id)+" planning order id="+str(robot.order[0])+" product="+str(robot.order[1])+" station="+str(robot.order[2])+" at t="+str(self.t))
                    if self.benchmark:
                        ts = time()
                    robot.solve()
                    self.used_shelves.append(robot.shelf)
                    if self.benchmark:
                        tf = time()
                        t = tf-ts
                        self.solve_times.append(t)
                        print("St=%s," %(t)), # Solve Time
        self.state[robot.pos[0]-1][robot.pos[1]-1] = 0

    def update_orders(self, order, shelf):
        self.used_shelves.remove(shelf)
        self.orders_in_delivery.remove(order)

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
    output = "print" # "viz" | "print" | None
    benchmark = False # True | False
    # zu messen:
    # zeit init + run + gesmatzeit
    # zeit grouden in pathfind + grounden in robot + gesmatzeit
    # zeit initiales solven + gesamtzeit
    # zeit resolving + gesamtzeit + anzahl resolving
    # gesamtzeit des plans

    parser = argparse.ArgumentParser()
    parser.add_argument("instance", help = "The instance to be loaded")
    parser.add_argument("-b","--benchmark", help="Use benchmark output",
                    action="store_true")
    args = parser.parse_args()
    if args.benchmark:
        benchmark = True
        output = None
    
    if benchmark:
        t1 = time()
    pathfind = Pathfind(args.instance, './pathfind.lp', output, benchmark)
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
