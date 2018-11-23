import clingo
import robot
import printer
import simulatorCostum
from time import time
import sys

class Pathfind(object):
	def __init__(self, instance, encoding, benchmark):
		self.instance = instance
		self.encoding = encoding
		self.prg = clingo.Control()
		self.prg.load(instance)
		if benchmark:
			ts = time()
		self.prg.ground([("base", [])])
		if benchmark:
			tf = time()
			groundTime = tf-ts
			print("IGt=%s," %(groundTime)), # Initial Ground time
		self.state = []
		if benchmark:
			ts = time()
		self.parse_instance()
		if benchmark:
			tf = time()
			t = tf-ts
			print("Pt=%s," %(t)), # Parse Time
		self.benchmark = benchmark
		
		self.sim = printer.Printer()
		#self.sim = simulatorCostum.SimulatorCostum()
		if not self.benchmark:
			self.sim.add_inits(self.get_inits())

		for robot in self.robots:
			self.state[robot.pos[0]-1][robot.pos[1]-1] = 0
			robot.update_state(self.state)
			self.assign_order(robot)
		
		
		self.prg.load(encoding)
		if benchmark:
			ts = time()
		self.prg.ground([("base", []),("centralized", [])])
		if benchmark:
			tf = time()
			t = tf-ts
			print("Gt=%s," %(t)), # Ground time
		
		self.t = 0	
		
		
	def run(self):
		
		self.model = []
		if benchmark:
			ts = time()
		with self.prg.solve(yield_=True) as h:
			for m in h:
				opt = m
			for atom in opt.symbols(shown=True):
				self.model.append(atom)
		if benchmark:
			tf = time()
			t = tf-ts
			print("St=%s," %(t)), # Solve time
			
		for atom in self.model:
			name = atom.name
			args = []
			if not self.benchmark:
				if name == "move":
					args.append(atom.arguments[0])
					args.append(atom.arguments[1])
					self.sim.add(atom.arguments[3], name, args, atom.arguments[2])
				if name == "pickup" or name == "putdown":
					self.sim.add(atom.arguments[1], name, args, atom.arguments[0])
				if name == "deliver":
					args.append(atom.arguments[2])
					args.append(atom.arguments[3])
					self.sim.add(atom.arguments[1], name, args, atom.arguments[0])
			
		self.sim.run(self.t)
		
		
	
	def assign_order(self, robot):
		# todo: keine ueberschneidungen in zugeteielten orders
		robot.set_order(self.orders[0][0], self.orders[0][1], self.orders[0][2], self.prg)
		# aus orders loeschen und in liste mit zur zeit bearbeiteten orders speichern
		del self.orders[0]
	
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
	
	def parse_instance(self):
		self.nodes = []
		self.highways = []
		self.robots = []
		self.orders = []
		order_stations = {} # zum zuordnen der orders zu pickingstation benoetigt
		self.pickingstations = []
		self.shelves = []
		self.products = []

		if benchmark:
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

		if benchmark:
			tf = time()
			t = tf-ts
			print("ISt=%s," %(t)), # Initial Solve Time
		# pickingstations zu orders zuteilen
		for order in self.orders:
			order.append(order_stations[order[0]])

		# self.state initialisieren
		for i in range(max(self.nodes, key=lambda item:item[1])[1]):
			self.state.append([])
			for j in range(max(self.nodes, key=lambda item:item[2])[2]):
				self.state[i].append(1)
				
				
				
				
if __name__ == "__main__":
	
	benchmark = True
	
	if benchmark:
		ts = time()
	pathfind = Pathfind(str(str(sys.argv[1])), './pathfind.lp', benchmark)
	if benchmark:
		tf = time()
		initTime = tf-ts
		print("It=%s," %(initTime)), # Initial time
	if benchmark:
		ts = time()
	pathfind.run()
	if benchmark:
		tf = time()
		runTime = tf-ts
		print("Rt=%s," %(runTime)), # Run time
		print("Tt=%s" %(initTime+runTime)) # Total time
	
	
	
	