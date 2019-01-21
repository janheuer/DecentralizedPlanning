# -*- coding: utf-8 -*-

from __future__ import print_function
import clingo
import simulatorCostum # needs simulator.py and network.py from visualizer/scripts
import printer
import robot
from time import time
import argparse
import sys

class Pathfind(object):
	def __init__(self, instance, encoding, model_output, verbose, benchmark, verbose_out):
		self.model_output = model_output
		self.benchmark = benchmark
		self.verbose = verbose
		self.verbose_out = verbose_out
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
			print("PGt=%s," %(t), file=sys.stderr, end='') # Parsing Ground time
		self.state = [] # to save the positions of the robots
		if self.benchmark:
			ts = time()
		self.parse_instance()
		if self.benchmark:
			tf = time()
			t = tf-ts
			print("Pt=%s," %(t), file=sys.stderr, end='') # Parse time
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
		if self.model_output:
			self.sim = printer.Printer()
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
		if self.model_output:
			self.sim.run(self.t)
		if benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length

	def run_shortest_replanning(self):
		while self.orders != [] or self.orders_in_delivery != []:

			self.t += 1
			# mark new potential new position in temp_state
			temp_state = list(self.state)
			for r in self.robots:
				temp_state[r.pos[0]-1][r.pos[1]-1] = 1
			for r in self.robots:
				temp_state[r.next_pos[0]-1][r.next_pos[1]-1] = 0

			r_replanned = []
			for i,r1 in enumerate(self.robots):
				temp_robots = self.robots[0:i]
				temp_robots += self.robots[i+1:]
				for r2 in temp_robots:
					if (r1.next_pos == r2.next_pos) or ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
						if r1.next_pos == r2.next_pos:
							if self.verbose:
								print("conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
						else:
							if self.verbose:
								print("swapping conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
						if (r1.next_action.name == "move") and (r2.next_action.name == "move"): # method only works if both move
							# both robots find a new plan
							r1.update_state(temp_state)
							if self.benchmark:
								ts = time()
							r1.find_new_plan()
							if self.benchmark:
								tf = time()
								t = tf-ts
								self.solve_times.append(t)
								self.resolve_times.append(t)
								print("Rst1=%s," %(t), file=sys.stderr, end='') # Rst1 for robot 1 of 2
								print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')

							if self.benchmark:
								ts = time()
							r2.update_state(temp_state)
							if self.benchmark:
								tf = time()
								t = tf-ts
								self.solve_times.append(t)
								self.resolve_times.append(t)
								print("Rst2=%s," %(t), file=sys.stderr, end='') # Rst2 for robot 2 of 2
								print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')

							r2.find_new_plan()
							# compute added length of new plan
							#print(r1.plan_length)
							dr1 = -1 if r1.plan_length == -1 else r1.plan_length - (r1.old_plan_length - (r1.t - 1))
							#print(dr1)
							#print(r2.plan_length)
							dr2 = -1 if r2.plan_length == -1 else r2.plan_length - (r2.old_plan_length - (r2.t - 1))
							#print(dr2)
							# choose which robot uses new plan
							if ((dr1 == -1) and (dr2 == -1)):
								if self.verbose:
									print("both robots deadlocked", file=verbose_out)
								# both robot deadlocked
								# both use new plan (which causes them to wait next timestep and replan)
								r1.use_new_plan()
								# whenever a robot uses a new plan we have to check for possible new conflicts
								# the checking of conflicts with robots with a higher id than r1 is already done by the loop
								# this however does not detect conflicts of the following kind:
								# initially no conflict between 1 and 2
								# after a conflict with 3 the robot 2 uses a new plan
								# this new plan could introduce a conflict between 1 and 2
								# to check for these all robots which use a new plan are saved in a list and will be checked again after the loop
								# TODO: possibly rewrite this so this loop already uses a list and initially all robots are in it ?
								if r1 not in r_replanned:
									r_replanned.append(r1)
								r2.use_new_plan()
								if r2 not in r_replanned:
									r_replanned.append(r2)
							if (dr1 == -1) or (dr2 <= dr1 and dr2!=-1): # here dr2 can still be -1 -> then dr2<=dr1 would be true therefore the condition dr2!=-1 is needed
								if self.verbose:
									print("r1 deadlocked or dr2<=dr1", file=verbose_out)
								r1.use_old_plan()
								#print(r1.plan_length)
								#print(r1.model)
								r2.use_new_plan()
								if r2 not in r_replanned:
									r_replanned.append(r2)
								#print(r2.plan_length)
								#print(r2.model)
								temp_state[r2.next_pos[0]-1][r2.next_pos[1]-1] = 0
							elif (dr2 == -1) or (dr1 < dr2):
								if self.verbose:
									print("r2 deadlocked or dr1<dr2", file=verbose_out)
								r1.use_new_plan()
								if r1 not in r_replanned:
									r_replanned.append(r1)
								#print(r1.model)
								r2.use_old_plan()
								#print(r2.model)
								temp_state[r1.next_pos[0]-1][r1.next_pos[1]-1] = 0
						else:
							if r1.next_action.name != "move":
								if self.verbose:
									print("only r2 moves", file=verbose_out)
								r2.update_state(temp_state)
								if self.benchmark:
									ts = time()
								r2.find_new_plan()
								if self.benchmark:
									tf = time()
									t = tf-ts
									self.solve_times.append(t)
									self.resolve_times.append(t)
									print("Rst0=%s," %(t), file=sys.stderr, end='') # Rst0 because no second robot
									print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')
								r2.use_new_plan()
								if r2 not in r_replanned:
									r_replanned.append(r2)
							if r2.next_action.name != "move":
								if self.verbose:
									print("only r1 moves", file=verbose_out)
								r1.update_state(temp_state)
								if self.benchmark:
									ts = time()
								r1.find_new_plan()
								if self.benchmark:
									tf = time()
									t = tf-ts
									self.solve_times.append(t)
									self.resolve_times.append(t)
									print("Rst0=%s," %(t), file=sys.stderr, end='') # Rst0 because no second robot
									print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')
								r1.use_new_plan()
								if r1 not in r_replanned:
									r_replanned.append(r1)

			# TODO rewrite *****************************************************
			while r_replanned != []:
				r1 = r_replanned.pop()
				i = self.robots.index(r1)
				temp_robots = self.robots[0:i]
				temp_robots += self.robots[i+1:]
				for r2 in temp_robots:
					if (r1.next_pos == r2.next_pos) or ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
						if r1.next_pos == r2.next_pos:
							if self.verbose:
								print("conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
						else:
							if self.verbose:
								print("swapping conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
						if (r1.next_action.name == "move") and (r2.next_action.name == "move"): # method only works if both move
							# both robots find a new plan
							r1.update_state(temp_state)
							if self.benchmark:
								ts = time()
							r1.find_new_plan()
							if self.benchmark:
								tf = time()
								t = tf-ts
								self.solve_times.append(t)
								self.resolve_times.append(t)
								print("Rst1=%s," %(t), file=sys.stderr, end='') # Rst1 for robot 1 of 2
								print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')

							if self.benchmark:
								ts = time()
							r2.update_state(temp_state)
							if self.benchmark:
								tf = time()
								t = tf-ts
								self.solve_times.append(t)
								self.resolve_times.append(t)
								print("Rst2=%s," %(t), file=sys.stderr, end='') # Rst2 for robot 2 of 2
								print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')

							r2.find_new_plan()
							# compute added length of new plan
							#print(r1.plan_length)
							dr1 = -1 if r1.plan_length == -1 else r1.plan_length - (r1.old_plan_length - (r1.t - 1))
							#print(dr1)
							#print(r2.plan_length)
							dr2 = -1 if r2.plan_length == -1 else r2.plan_length - (r2.old_plan_length - (r2.t - 1))
							#print(dr2)
							# choose which robot uses new plan
							if ((dr1 == -1) and (dr2 == -1)):
								if self.verbose:
									print("both robots deadlocked", file=verbose_out)
								# both robot deadlocked
								# both use new plan (which causes them to wait next timestep and replan)
								r1.use_new_plan()
								# whenever a robot uses a new plan we have to check for possible new conflicts
								# the checking of conflicts with robots with a higher id than r1 is already done by the loop
								# this however does not detect conflicts of the following kind:
								# initially no conflict between 1 and 2
								# after a conflict with 3 the robot 2 uses a new plan
								# this new plan could introduce a conflict between 1 and 2
								# to check for these all robots which use a new plan are saved in a list and will be checked again after the loop
								# TODO: possibly rewrite this so this loop already uses a list and initially all robots are in it ?
								if r1 not in r_replanned:
									r_replanned.append(r1)
								r2.use_new_plan()
								if r2 not in r_replanned:
									r_replanned.append(r2)
							if (dr1 == -1) or (dr2 <= dr1 and dr2!=-1): # here dr2 can still be -1 -> then dr2<=dr1 would be true therefore the condition dr2!=-1 is needed
								if self.verbose:
									print("r1 deadlocked or dr2<=dr1", file=verbose_out)
								r1.use_old_plan()
								#print(r1.plan_length)
								#print(r1.model)
								r2.use_new_plan()
								if r2 not in r_replanned:
									r_replanned.append(r2)
								#print(r2.plan_length)
								#print(r2.model)
								temp_state[r2.next_pos[0]-1][r2.next_pos[1]-1] = 0
							elif (dr2 == -1) or (dr1 < dr2):
								if self.verbose:
									print("r2 deadlocked or dr1<dr2", file=verbose_out)
								r1.use_new_plan()
								if r1 not in r_replanned:
									r_replanned.append(r1)
								#print(r1.model)
								r2.use_old_plan()
								#print(r2.model)
								temp_state[r1.next_pos[0]-1][r1.next_pos[1]-1] = 0
						else:
							if r1.next_action.name != "move":
								if self.verbose:
									print("only r2 moves", file=verbose_out)
								r2.update_state(temp_state)
								if self.benchmark:
									ts = time()
								r2.find_new_plan()
								if self.benchmark:
									tf = time()
									t = tf-ts
									self.solve_times.append(t)
									self.resolve_times.append(t)
									print("Rst0=%s," %(t), file=sys.stderr, end='') # Rst0 because no second robot
									print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')
								r2.use_new_plan()
								if r2 not in r_replanned:
									r_replanned.append(r2)
							if r2.next_action.name != "move":
								if self.verbose:
									print("only r1 moves", file=verbose_out)
								r1.update_state(temp_state)
								if self.benchmark:
									ts = time()
								r1.find_new_plan()
								if self.benchmark:
									tf = time()
									t = tf-ts
									self.solve_times.append(t)
									self.resolve_times.append(t)
									print("Rst0=%s," %(t), file=sys.stderr, end='') # Rst0 because no second robot
									print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')
								r1.use_new_plan()
								if r1 not in r_replanned:
									r_replanned.append(r1)
			# ******************************************************************


			for robot in self.robots:
				self.perform_action(robot)

		# start visualizer
		if self.model_output:
			self.sim.run(self.t)
		if benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length


	def run_crossing(self):
		while self.orders != [] or self.orders_in_delivery != []:

			self.t += 1
			# mark new potential new position in temp_state
			temp_state = list(self.state)
			for r in self.robots:
				temp_state[r.pos[0]-1][r.pos[1]-1] = 1
			for r in self.robots:
				temp_state[r.next_pos[0]-1][r.next_pos[1]-1] = 0

			for i,r1 in enumerate(self.robots):
				temp_robots = self.robots[0:i]
				temp_robots += self.robots[i+1:]
				for r2 in temp_robots:
					if r1.next_pos == r2.next_pos:
						if self.verbose:
							print("conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
							print("r"+str(r1.id)+" waits", file=verbose_out)
						r1.wait = True
						r1.next_pos = list(r1.pos) # next_pos needs to be changed or the conflict will be detected again
					if ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
						if self.verbose:
							print("swapping conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
						if (r1.next_action.name == "move") and (r2.next_action.name == "move"): # method only works if both move
							r1.update_state(temp_state)
							r1.find_crossroad()
							r2.update_state(temp_state)
							r2.find_crossroad()

							if r1.cross_length < r2.cross_length:
								print("r"+str(r1.id)+" dodges", file=verbose_out)
								# detemine in which direction r1 has to dodge
								# get direction of r2 moving onto crossing and direction of r2 moving from crossing
								for atom in r2.model:
									if atom.name == "move":
										#r2.t-1 = timesteps r2 has completed, r1.cross_length = time r1 takes to crossing +1 because r2 takes one step more to crossing +1 beacuse we need the next move atom
										if atom.arguments[2].number == r2.t-1 + r1.cross_length +1:
											move_to_cross = atom
										if atom.arguments[2].number == r2.t-1 + r1.cross_length +1 +1:
											move_from_cross = atom
								# cross_model contains moves for all possible direction off the crossing
								# one needs to be chosen
								second_move = False
								for atom in r1.cross_model:
									if atom.name == "move":
										if atom.arguments[2].number == r1.cross_length+1:
											# remove the direction from which the other robot is coming
											if ((atom.arguments[0].number == -1*move_to_cross.arguments[0].number) and
												(atom.arguments[1].number == -1*move_to_cross.arguments[1].number)):
												r1.cross_model.remove(atom)
											# remove the direction in which the other robot is moving
											elif ((atom.arguments[0].number == move_from_cross.arguments[0].number) and
												(atom.arguments[1].number == move_from_cross.arguments[1].number)):
												r1.cross_model.remove(atom)
											# now atleast 1 direction is left but there could be a second one
											# the first direction will be used
											elif not second_move:
												second_move = True
											# the possible other direction will be removed
											else:
												r1.cross_model.remove(atom)
								r1.use_crossroad() # this also generate rest of plan (returning to start)

							if r1.cross_length < r2.cross_length:
								print("r"+str(r2.id)+" dodges", file=verbose_out)
								for atom in r1.model:
									if atom.name == "move":
										if atom.arguments[2].number == r1.t-1 + r2.cross_length +1:
											move_to_cross = atom
										if atom.arguments[2].number == r1.t-1 + r2.cross_length +1 +1:
											move_from_cross = atom
								second_move = False
								for atom in r2.cross_model:
									if atom.name == "move":
										if atom.arguments[2].number == r2.cross_length+1:
											if ((atom.arguments[0].number == move_to_cross.arguments[0].number) and
												(atom.arguments[1].number == move_to_cross.arguments[1].number)):
												r2.cross_model.remove(atom)
											elif ((atom.arguments[0].number == move_from_cross.arguments[0].number) and
												(atom.arguments[1].number == move_from_cross.arguments[1].number)):
												r2.cross_model.remove(atom)
											elif not second_move:
												second_move = True
											else:
												r2.cross_model.remove(atom)
								r2.use_crossroad()

						else:
							if r1.next_action.name != "move":
								if self.verbose:
									print("only r2 moves", file=verbose_out)
								r2.update_state(temp_state)
								r2.find_new_plan()
								r2.use_new_plan()
							if r2.next_action.name != "move":
								if self.verbose:
									print("only r1 moves", file=verbose_out)
								r1.update_state(temp_state)
								r1.find_new_plan()
								r1.use_new_plan()

			for robot in self.robots:
				self.perform_action(robot)

		# start visualizer
		if self.model_output:
			self.sim.run(self.t)
		if benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length



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
				#					 -new conflicts after replanning

			for robot in self.robots:
				self.perform_action(robot)

		# start visualizer
		if self.model_output:
			self.sim.run(self.t)
		if benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length

	def solve_conflicts(self, conflicts):
		for conflict in conflicts:
			if conflict[0].next_pos == conflict[1].next_pos:
				conflict[0].wait = True

	def get_conflicts(self):
		# returns list of tuples of robots which have a conflict
		conflicts = []
		for i,r1 in enumerate(self.robots[:-1]):
			for r2 in self.robots[i+1:]:
				# robots want to move onto same pos or robots want to swap pos
				if (r1.next_pos == r2.next_pos) or ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
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
							print("Igt=%s," %(t), file=sys.stderr, end='') # Init Ground time
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
			print("PSt=%s," %(t), file=sys.stderr, end='') # Parsing solve time

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
		if (self.model_output) and (name != ""):
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
			if self.verbose:
				print("robot"+str(robot.id)+" planning order id="+str(robot.order[0])+" product="+str(robot.order[1])+" station="+str(robot.order[2])+" at t="+str(self.t), file=verbose_out)
		else:
			resolve = True
			if self.verbose:
				print("robot"+str(robot.id)+" replanning at t="+str(self.t), file=verbose_out)

		if self.benchmark:
			ts = time()
		found_plan = robot.solve()
		if self.benchmark:
			tf = time()
			t = tf-ts
			self.solve_times.append(t)
			if resolve:
				self.resolve_times.append(t)
				print("Rst=%s," %(t), file=sys.stderr, end='')
			else:
				print("St=%s," %(t), file=sys.stderr, end='')
			print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')

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
	parser.add_argument("-n", "--nomodel", help="disables output of the model", default=False, action="store_true")
	parser.add_argument("-v", "--verbose", help="outputs additional information (printed to stderr)", default=False, action="store_true")
	parser.add_argument("-b", "--benchmark", help="use benchmark output (possible verbose output will be redirected to stdout)", default=False, action="store_true")
	parser.add_argument("-e", "--encoding", help="encoding to be used. default: ./pathfind.lp", default = './pathfind.lp', type = str)
	args = parser.parse_args()

	model_output = not args.nomodel
	verbose = args.verbose
	benchmark = args.benchmark
	encoding = args.encoding
	verbose_out = sys.stderr if not benchmark else sys.stdout

	if benchmark:
		t1 = time()
	pathfind = Pathfind(args.instance, encoding, model_output, verbose, benchmark, verbose_out)
	if benchmark:
		t2 = time()
		initTime = t2-t1
		print("It=%s," %(initTime), file=sys.stderr, end='') # Initial time

	if benchmark:
		groundTime = 0
		for t in pathfind.ground_times:
			groundTime += t
		print("Tgt=%s," %(groundTime), file=sys.stderr, end='') #Total ground time

		solveTimeInit = 0
		for t in pathfind.solve_times:
			solveTimeInit += t
		print("TstI=%s," %(solveTimeInit), file=sys.stderr, end='') # Total solve time in Init
		pathfind.solve_times = []

	if benchmark:
		t1 = time()
	#pathfind.run()
	#pathfind.run_shortest_replanning()
	pathfind.run_crossing()
	if benchmark:
		t2 = time()
		runTime = t2-t1
		print("Rt=%s," %(runTime), file=sys.stderr, end='') # Run time

	if benchmark:
		solveTimeRun = 0
		for t in pathfind.solve_times:
			solveTimeRun += t
		print("TstR=%s," %(solveTimeRun), file=sys.stderr, end='') # Total solve time in run
		print("Tst=%s," %(solveTimeInit+solveTimeRun), file=sys.stderr, end='') # Total solve time

		resolveTime = 0
		for t in pathfind.resolve_times:
			resolveTime += t
		print("Trst=%s," %(resolveTime), file=sys.stderr, end='') # Total resolve time
		print("Tt=%s" %(initTime+runTime), file=sys.stderr) # Total time
