from __future__ import print_function # only needed for debugging
import sys # only needed for debugging
import clingo

class Robot(object):
	def __init__(self, id, start, encoding, instance, external, highways):
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

		self.encoding = encoding
		self.instance = instance
		self.external = external
		self.highways = highways

		if self.external:
			self.prg = clingo.Control()
			self.prg.load(encoding)
			self.prg.load(instance)
			parts = []
			parts.append(("base", []))
			parts.append(("decentralized", []))
			if self.highways:
				parts.append(("highways", []))
			self.prg.ground(parts)

		self.plan_finished = True
		self.waiting = False # The robot currently does/does not need to wait

		self.model = []
		self.plan_length = -1

		self.next_action = clingo.Function("", [])

		self.crossing_strategy = False

	def init_crossing(self):
		self.crossing_strategy = True

		self.using_crossroad = False
		self.crossroad = None

		self.in_conflict = False
		self.dodging = False
		self.cross_done = -1
		self.blocked_crossings = []
		self.conflict_partners = {}
		self.waiting_on = []

		self.replanned = False

		if self.external:
			self.crossroad = clingo.Control()
			self.crossroad.load("./crossroad.lp")
			self.crossroad.load(self.instance)
			self.crossroad.ground([("base", []), ("external", [])])

	def find_new_plan(self):
		self.old_model = list(self.model)
		self.old_plan_length = self.plan_length

		self.model = []

		if self.external:
			self.prg.assign_external(clingo.Function("start", [self.start[0],self.start[1],1]), False)
			self.prg.assign_external(clingo.Function("start", [self.pos[0],self.pos[1],1]), True)

			self.prg.assign_external(clingo.Function("pickup", [0,1]), self.pickupdone)
			self.prg.assign_external(clingo.Function("deliver", [0,1, self.order[1], self.order[0]]), self.deliverdone)

			if self.shelf != -1:
				for shelf in self.available_shelves:
					self.prg.assign_external(clingo.Function("available", [shelf]), False)
				self.prg.assign_external(clingo.Function("available", [self.shelf]), True)

			for i in range(len(self.state)):
				for j in range(len(self.state[0])):
					self.prg.assign_external(clingo.Function("block", [i+1,j+1]), not self.state[i][j])
		else:
			self.prg = clingo.Control()
			self.prg.load(self.encoding)
			self.prg.load(self.instance)
			self.prg.add("start", ["pos0", "pos1"], "start(pos0, pos1, 1).")
			if (self.pickupdone):
				self.prg.add("base", [], "pickup(0, 1).")
			if (self.deliverdone):
				self.prg.add("base", [], "deliver(0, 1, "+str(self.order[1])+", "+str(self.order[0])+").")

			self.prg.add("base", [], "block(-1, -1).")
			for i in range(len(self.state)):
				for j in range(len(self.state[0])):
					if not (self.state[i][j]):
						self.prg.add("base", [], "block("+str(i+1)+", "+str(j+1)+").")
			if self.shelf != -1:
				self.prg.add("base", [], "available("+str(self.shelf)+").")
			else:
				for shelf in self.available_shelves:
					self.prg.add("base", [], "available("+str(shelf)+").")

			self.prg.add("base", [], "order("+str(self.order[1])+", "+str(self.order[2])+", 1, "+str(self.order[0])+").")

			parts = []
			parts.append(("base", []))
			parts.append(("decentralizedNoExternals", []))
			parts.append(("start", [self.pos[0], self.pos[1]]))
			if (self.highways):
				parts.append(("highways", []))
			self.prg.ground(parts)

		self.start = list(self.pos)
		self.plan_finished = False

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
			self.next_action = clingo.Function("", [])
			if self.shelf == -1:
				if self.external:
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
		if found_model:
			self.use_new_plan()
		return found_model

	def block_crossings(self, crossings):
		self.blocked_crossings = list(crossings)

	def update_partners(self, partner, t_conflict):
		# update conflict_partners dict
		# key: robot object (the conflict partner)
		# value: how long is the robot conflict partner (as timestep)
		if partner not in self.conflict_partners:
			self.conflict_partners[partner] = self.t-1
		self.conflict_partners[partner] += t_conflict

	def find_crossroad(self):
		if self.external:
			self.crossroad.assign_external(clingo.Function("start", [self.pos[0],self.pos[1],1]), True)
			for cross in self.blocked_crossings:
				self.crossroad.assign_external(clingo.Function("block", [cross[0],cross[1]]), True)
		else:
			self.crossroad = clingo.Control()
			self.crossroad.load("./crossroad.lp")
			self.crossroad.load(self.instance)
			self.crossroad.add("start", ["pos0", "pos1"], "start(pos0, pos1, 1).")
			for cross in self.blocked_crossings:
				self.crossroad.add("base", [], "block("+str(cross[0])+", "+str(cross[1])+").")
			self.crossroad.ground([("base", []), ("start", [self.pos[0], self.pos[1]])])

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
					else:
						self.cross_model.append(atom)
			else:
				self.cross_length = -1

		# cross_model needs to be sorted in ascending order
		self.cross_model.sort(key=lambda atom:atom.arguments[2].number)

		if self.external:
			self.crossroad.assign_external(clingo.Function("start", [self.pos[0],self.pos[1],1]), False)
			for cross in self.blocked_crossings:
				self.crossroad.assign_external(clingo.Function("block", [cross[0],cross[1]]), False)

	def set_in_conflict(self, t_conflict):
		self.in_conflict = True
		if self.cross_done == -1:
			self.cross_done = self.t-1
		self.cross_done += t_conflict+1

	def use_crossroad(self):
		if self.cross_model == []:
			return

		self.in_conflict = True
		if not self.dodging:
			self.cross_done = -1
		self.dodging = True
		if self.cross_done == -1:
			self.cross_done = self.t-1
		self.cross_done += len(self.cross_model)

		# total time added by dodging
		total_t = 2*len(self.cross_model)
		self.cross_length = total_t

		# generate part of the model for returning from crossing
		return_model = []
		for atom in self.cross_model:
			if atom.name == "move":
				# invert directions and time
				return_model.append(clingo.Function("move", [-1*atom.arguments[0].number, -1*atom.arguments[1].number, total_t-(atom.arguments[2].number-1), 1]))

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
					new_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number+total_t, atom.arguments[3].number]))
			# same thing for all other action atoms
			elif atom.name in ["pickup", "putdown"]:
				if atom.arguments[0].number < self.t:
					new_model.append(atom)
				else:
					new_model.append(clingo.Function(atom.name, [atom.arguments[0].number+total_t, atom.arguments[1].number]))
			elif atom.name == "deliver":
				if atom.arguments[0].number < self.t:
					new_model.append(atom)
				else:
					new_model.append(clingo.Function(atom.name, [atom.arguments[0].number+total_t, atom.arguments[1].number, atom.arguments[2].number, atom.arguments[3].number]))
			else:
				new_model.append(atom)

		# add cross_model
		for atom in self.cross_model:
			new_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number+self.t-1, atom.arguments[3].number]))
		for atom in return_model:
			new_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number+self.t-1, atom.arguments[3].number]))

		self.model = new_model
		self.t -= 1
		self.get_next_action()
		self.t += 1

	def duplicate_last_move(self):
		last_move = self.cross_model[len(self.cross_model)-1]
		self.cross_model.append(clingo.Function(last_move.name, [last_move.arguments[0].number, last_move.arguments[1].number, last_move.arguments[2].number+1, last_move.arguments[3].number]))

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
			# if a robot is deadlocked we still need to know its next position to prevent conflicts
			self.next_pos = list(self.pos) # needed for shortest_replanning strategy
			self.next_action = clingo.Function("", [])

	def action(self):
		# special stuff for when crossing strategy is used
		if self.crossing_strategy:
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

		if self.plan_finished:
			return "",[]
		else:
			if not self.waiting:
				action = self.next_action
				name = action.name
				args = []
				if name == "putdown":
					self.pickupdone = False
					self.deliverdone = False
					if self.external:
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
				self.waiting = False
				self.waiting_on = []
				self.t -= 1
				self.get_next_action()
				self.t += 1
				return "wait",[]

	def wait(self):
		self.waiting = True
		self.next_action = clingo.Function("wait", [])
		self.next_pos = list(self.pos)

	def set_order(self, order, available_shelves):
		self.shelf = -1

		self.order = list(order)
		self.available_shelves = list(available_shelves)
		if self.external:
			self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], 1, self.order[0]]), True)
			for shelf in self.available_shelves:
				self.prg.assign_external(clingo.Function("available", [shelf]), True)

	def release_order(self):
		if self.external:
			self.prg.assign_external(clingo.Function("order", [self.order[1], self.order[2], 1, self.order[0]]), False)
			for shelf in self.available_shelves:
				self.prg.assign_external(clingo.Function("available", [shelf]), False)
		self.shelf = -1

	def clear_state(self):
		# mark all positions as free
		for i in range(len(self.state)):
			for j in range(len(self.state[0])):
				self.state[i][j] = 1

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
		if self.next_action.name == "":
			return False
		if self.next_action.name == "move":
			# next_pos ist kein blockiertes feld
			return self.state[self.next_pos[0]-1][self.next_pos[1]-1]
		else:
			# pickup, deliver, putdown koennen immer ausgefuehrt werden
			return True
