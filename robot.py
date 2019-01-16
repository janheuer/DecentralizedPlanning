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
		self.start = list(start)
		self.pos = list(start)

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
		self.wait = False # The robot currently does/does not need to wait

		self.model = []
		self.plan_length = -1

		self.using_crossroad = False

	def find_new_plan(self):
		self.old_model = list(self.model)
		self.old_plan_length = self.plan_length

		self.model = []
		self.prg.assign_external(clingo.Function("start", [self.start[0],self.start[1],1]), False)
		self.prg.assign_external(clingo.Function("start", [self.pos[0],self.pos[1],1]), True)
		self.start = list(self.pos)
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

		found_model = False
		with self.prg.solve(yield_=True) as h:
			for m in h:
				found_model = True
				opt = m
			if found_model:
				for atom in opt.symbols(shown=True):
					self.model.append(atom)
					if atom.name == "chooseShelf":
						self.shelf = atom.arguments[0].number
					if atom.name == "putdown":
						self.plan_length = atom.arguments[0].number

		if not found_model:
			self.plan_length = -1
			self.next_action = None
			if self.shelf == -1:
				# order freigeben
				self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], 1, self.order[0]]), False)
				for shelf in self.available_shelves:
					self.prg.assign_external(clingo.Function("available", [shelf]), False)
				self.available_shelves = []

		return found_model

	def use_old_plan(self):
		self.model = list(self.old_model)
		self.plan_length = self.old_plan_length

		self.t -= 1
		self.get_next_action()
		self.t += 1

	def use_new_plan(self):
		self.t = 0
		self.get_next_action()
		self.t = 1

	def solve(self):
		found_model = self.find_new_plan()
		#if found_model:
		self.use_new_plan()
		return found_model

	def find_crossroad():
		if self.crossroad is None:
			self.crossroad = clingo.Control()
			self.crossroad.load("./crossroad.lp")
			self.crossroad.ground([("base", [])])
		self.crossroad.assign_external(clingo.Function("start", [self.pos[0],self.pos[1],1]), True)
		self.cross_model = []

		found_model = False
		with self.crossroad.solve(yield_=True) as h:
			for m in h:
				found_model = True
				opt = m
			if found_model:
				for atom in opt.symbols(shown=True):
					if atom.name == "goal":
						self.cross_length = atom.arguments[0].number
						#self.cross_pos = [atom.arguments[1].number,atom.arguments[2]]
					else:
						self.cross_model.append(atom)

		self.crossroad.assign_external(clingo.Function("start", [self.pos[0],self.pos[1],1]), False)

	def use_crossroad():
		self.using_crossroad = True
		self.t_model_done = self.t -1 # save how many steps of plan are already completed

		# total corss_model length will be corss_length (time to corssing) +1 (to actually dodge other robot) *2 (for returning)
		# construct the dodging move in pathfind ???
		total_t = 2*(self.cross_length+1)
		return_model = []
		for atom in self.cross_model:
			if atom.name == "move":
				return_model.append(clingo.Function("move", [-1*atom.arguments[0].number, -1*atom.arguments[1].number, total_t-(atom.arguments[2].number-1), 1]))
		self.cross_model += return_model # add all returning moves to the model
		self.cross_length = total_t

		self.t = 0
		self.get_next_action()
		self.t = 1

	def get_next_action(self):
		next_action = False
		if not self.using_crossroad:
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
				self.next_pos = list(self.pos) # needed for shortest_replanning strategy
				self.next_action = clingo.Function("wait", [])
				# if a robot is deadlocked we still need to know its next position to prevent conflicts
		else:
			for atom in self.cross_model:
				if atom.name == "move" and atom.arguments[2].number == self.t+1:
					self.next_action = atom
					self.next_pos[0] = self.pos[0] + atom.arguments[0].number
					self.next_pos[1] = self.pos[1] + atom.arguments[1].number
					next_action = True
			if not next_action:
				self.using_crossroad = False
				# restore to old plan
				self.t = self.t_model_done
				self.get_next_action()
				self.t += 1

			# TODO: continue with normal plan

	def action(self):
		if self.plan_finished:
			return "",[]
		else:
			if self.wait == False:
				action = self.next_action
				name = action.name
				args = []
				if name == "putdown":
					self.pickupdone = False
					self.deliverdone = False
					self.prg.assign_external(clingo.Function("deliver", [0,1, self.order[1], self.order[0]]), self.deliverdone)
				else:
					if name == "move":
						self.pos = list(self.next_pos)
						args = [action.arguments[0].number, action.arguments[1].number]
					elif name ==  "pickup":
						self.pickupdone = True
					elif name == "deliver":
						self.deliverdone = True
						args = [self.order[0], self.order[1], 1]
				self.get_next_action()
				self.t += 1
				return name, args
			else:
				#print ("%s is waiting at %d" %(self.id, self.t) )
				self.wait = False
				return "wait",[]

	def set_order(self, order, available_shelves):
		self.shelf = -1

		self.order = list(order)
		self.available_shelves = available_shelves
		self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], 1, self.order[0]]), True)
		for shelf in self.available_shelves:
			self.prg.assign_external(clingo.Function("available", [shelf]), True)

	def release_order(self):
		self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], 1, self.order[0]]), False)
		for shelf in self.available_shelves:
			self.prg.assign_external(clingo.Function("available", [shelf]), False)
		self.shelf = -1

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
		if self.next_action is None:
			return False
		if self.next_action.name == "move":
			# next_pos ist kein blockiertes feld
			return self.state[self.next_pos[0]-1][self.next_pos[1]-1]
		else:
			# pickup, deliver, putdown koennen immer ausgefuehrt werden
			return True
