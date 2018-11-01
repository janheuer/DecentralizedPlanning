# Programm aufgeteielt in pathfind.py + robot.py
# -> nicht mehr über clingo aufrufen sondern nur mit python
# Fuer anzeigen im visualizer: simulatorCostum.py (-> braucht die Dateien simulator.py und network.py aus visualizer/scripts)
# Im visualizer starten über: Network->Initialize simulator, dort kommando zum starten des programms eingeben (also python ./pathfind.py) + port 5001 
# Alternativ: Ausgaben durch print mit printer.py (-> dafür zeile 29 auskommentieren und dafür zeile 30 nutzen)

import clingo
import simulatorCostum
import printer
import robot

class Pathfind(object):
    def __init__(self, instance, encoding):
        # speichern der instanz in entsprechenden datenstrukturen + erstellen der roboter
        self.instance = instance
        self.encoding = encoding
        self.prg = clingo.Control()
        self.prg.load(instance)
        self.prg.ground([("base", [])])
        self.state = [] # zum speichern auf welchen positionen roboter stehen
        self.parse_instance()
        # nodes: [[id, x, y]]
        # highways: [[id, x, y]]
        # robots: [robot]
        # orders: [[id, product, pickingStation]]
        # pickingstations: [[id, x, y]]
        # shelves: [[id, x, y]]
        # products: [[id, shelf]]

        # initialisieren des simulators
        self.sim = simulatorCostum.SimulatorCostum()
        #self.sim = printer.Printer() # keine verwendung des visualizers, statt dessen print ausgaben
        self.sim.add_inits(self.get_inits())

        # initialisieren der roboter
        for robot in self.robots:
            self.state[robot.pos[0]-1][robot.pos[1]-1] = 0
            robot.update_state(self.state)
            self.assign_order(robot)
            robot.solve()

        self.t = 0

    def run(self):
        while self.orders != []:#or zur zeit bearbeitete orders != []
            self.t += 1
            # blockierte felder zum testen
            if (5>=self.t>=2):
                self.state[4-1][6-1] = 0
            if (16>=self.t>=13):
                self.state[4-1][2-1] = 0
            if (39>=self.t>=22):
                self.state[8-1][2-1] = 0
            for robot in self.robots:
                robot.update_state(self.state)
                if robot.action_possible():
                    self.perform_action(robot)
                else:
                    # todo: wenn roboter auf pickingstation bewegen will, neu solven nicht sinnvoll
                    # statdessen warten bis pickingstation wieder frei
                    robot.solve()
                    self.perform_action(robot)
        # visualization starten
        self.sim.run(self.t)

    def parse_instance(self):
        self.nodes = []
        self.highways = []
        self.robots = []
        self.orders = []
        order_stations = {} # zum zuordnen der orders zu pickingstation benoetigt
        self.pickingstations = []
        self.shelves = []
        self.products = []

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
                        self.robots.append(robot.Robot(id, [x,y], self.encoding, self.instance))
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

        # pickingstations zu orders zuteilen
        for order in self.orders:
            order.append(order_stations[order[0]])

        # self.state initialisieren
        for i in range(max(self.nodes, key=lambda item:item[1])[1]):
            self.state.append([])
            for j in range(max(self.nodes, key=lambda item:item[2])[2]):
                self.state[i].append(1)

    def assign_order(self, robot):
        # todo: keine ueberschneidungen in zugeteielten orders
        robot.set_order(self.orders[0][0], self.orders[0][1], self.orders[0][2])
        # aus orders loeschen und in liste mit zur zeit bearbeiteten orders speichern

    def perform_action(self, robot):
        prev_pos = robot.pos
        name, args = robot.action()
        self.sim.add(robot.id, name, args, self.t)
        if name == "move":
            self.state[prev_pos[0]-1][prev_pos[1]-1] = 1
            self.state[robot.pos[0]-1][robot.pos[1]-1] = 0
        elif name == "putdown":
            self.update_orders(robot.order)
            if self.orders != []:
                self.assign_order(robot)
                robot.solve()

    def update_orders(self, order):
        # aus liste mit zur zeit bearbeiteten order speichern
        self.orders.remove(order)

    def get_inits(self):
        # alle inits werden als string zur liste inits hinzugefuegt, diese wird dem simulator uebergebn
        inits = []
        for node in self.nodes:
            if node not in [[59,4,6], [15,4,2], [19,8,2]]: # blockierte felder zum testen
                inits.append("init(object(node,"+str(node[0])+"),value(at,("+str(node[1])+","+str(node[2])+")))")
        for highway in self.highways:
            if highway not in [[59,4,6], [15,4,2], [19,8,2]]: # blockierte felder zum testen
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
    pathfind = Pathfind('./instance.lp', './pathfind.lp')
    pathfind.run()
