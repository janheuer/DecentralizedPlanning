# -*- coding: utf-8 -*-

from __future__ import print_function
import clingo
import printer
import robot
from time import time
import argparse
import sys

class Pathfind(object):
	def __init__(self, instance, encoding, model_output, verbose, verbose_out, benchmark, external, highways):
		"""Assigns initial order to the robots and plans it
		Instance is saved in data structures by helper function parse_instance
		(also generates the Robot objects)
		"""
		self.instance = instance
		self.encoding = encoding
		self.model_output = model_output
		self.verbose = verbose
		self.verbose_out = verbose_out
		self.benchmark = benchmark
		self.external = external
		self.highwaysFlag = highways

		self.prg = clingo.Control()
		self.prg.load(instance)
		if self.benchmark:
			self.ground_times = []
			self.solve_times = []
			self.resolve_times = []
			self.real_time = 0
			ts = time()
		self.prg.ground([("base", [])])
		if self.benchmark:
			tf = time()
			t = tf-ts
			self.ground_times.append(t)
			print("PGt=%s," %(t), file=sys.stderr, end='') # Parsing Ground time
		if self.benchmark:
			ts = time()
		# reads instance and saves it in the datastructures
		self.parse_instance()
		if self.benchmark:
			tf = time()
			t = tf-ts
			print("Pt=%s," %(t), file=sys.stderr, end='') # Parse time

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
			self.plan(robot) # assigns an order and plans it

	"""helper functions"""

	def print_verbose(self, arg):
		if self.verbose:
			print(arg, file=verbose_out)

	def parse_instance(self):
		"""Reads self.instance and saves all information in the according data structures
		Also creates the Robot objects, and saves their initial positions
		"""
		self.nodes = [] # [[id,x,y]]
		self.highways = [] # [[id,x,y]]
		self.robots = [] # [Robot]
		self.orders = [] # [[id,product]] in the end format is changed to [[id,product,station]]
		# temporary dict, in the end the pickingstations will be added to the elements of self.orders
		# because the atoms which specify the product for an order and the atom specifing the pickingstation
		# are separate the temporary dict is needed to merge both informations later
		order_stations = {} # key: order id, value: pickingstation id
		self.pickingstations = [] # [[id,x,y]]
		self.shelves = [] # [[id,x,y]]
		self.products = [] # [[id,shelf]]

		if self.benchmark:
				ts = time()
		with self.prg.solve(yield_=True) as h:
			for m in h:
				opt = m
			if self.benchmark:
				tf = time()
				t = tf-ts
				print("PSt=%s," %(t), file=sys.stderr, end='') # Parsing solve time
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
						self.robots.append(robot.Robot(id, [x,y], self.encoding, self.instance, self.external, self.highwaysFlag))
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

		# assign pickingstations to orders
		for order in self.orders:
			order.append(order_stations[order[0]])
		# now self.orders has format [[id,product,station]]

		# intialize state matrix, saves which positions are free (1=free, 0=blocked)
		self.state = []
		for i in range(max(self.nodes, key=lambda item:item[1])[1]): # range(x-size)
			self.state.append([])
			for j in range(max(self.nodes, key=lambda item:item[2])[2]): # range(y-size)
				self.state[i].append(1)
		# save robot start position in self.state
		for r in self.robots:
			self.state[r.pos[0]-1][r.pos[1]-1] = 0

	def get_inits(self):
		"""All inits are added to inits as a string according to asprillo specifications"""
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
		order_stations = [] # used to make sure that only one string for the pickingstation for each order is added
		for order in self.orders:
			inits.append("init(object(order,"+str(order[0])+"),value(line,("+str(order[1])+",1)))")
			# check if string with pickingstation for this order was already added
			if order[0] not in order_stations:
				order_stations.append(order[0])
				inits.append("init(object(order,"+str(order[0])+"),value(pickingStation,"+str(order[2])+"))")
		for product in self.products:
			inits.append("init(object(product,"+str(product[0])+"),value(on,("+str(product[1])+",1)))")
		return inits

	def perform_action(self, robot):
		"""Performs the action of robot
		If the order is finished with this action, a new order is planned
		"""
		name, args = robot.action()
		if name == "":
			# robot doesn't have to do a action in this timestep because
			# 1) there aren't any more orders
			# 2) in the last timestep no order could be assigned
			# 3) robot was in a deadlock in the last timestep
			self.plan(robot)
		elif self.model_output:
			self.sim.add(robot.id, name, args, self.t)
		if name == "putdown":
			# order is finished
			self.finish_order(robot.order, robot.shelf)
			robot.release_order()
			# plan a new order
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
		and adds it to the list of order which are open
		This funtion is needed for situations in which the robot is deadlocked
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
		if robot.shelf == -1: # robot doesn't have a order assigned
			resolve = False
			if not self.assign_order(robot): # try to assign a order
				return False # no order can be assigned
			if self.verbose:
				print("robot"+str(robot.id)+" planning order id="+str(robot.order[0])+" product="+str(robot.order[1])+" station="+str(robot.order[2])+" at t="+str(self.t), file=verbose_out)
		else:
			resolve = True
			if self.verbose:
				print("robot"+str(robot.id)+" replanning at t="+str(self.t), file=verbose_out)

		robot.update_state(self.state) # not needed for use in standard run() but needed in other versions
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
			print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "),t=" + str(self.t) + ",", file=sys.stderr, end='')

		if found_plan: # if the robot found a plan the shelf has to be reserved
			self.reserve_shelf(robot.shelf)
			return True
		else: # robot couldn't find a plan
			if robot.shelf == -1:
				# robot couldn't start planning the order (because he deadlocked in his start position)
				# release the order so that other robots can try to plan it
				self.release_order(robot.order)
			return False

	"""main function for sequential strategy"""

	def run_sequential(self):
		"""Main function of Pathfind
		Starts execution of all plans and handles possible conflicts
		When robots have completed an order, they get assigned a new order
		Finishes when all orders are delivered
		"""
		while self.orders != [] or self.orders_in_delivery != []:
			self.t += 1
			for robot in self.robots:
				robot.update_state(self.state)
				if not robot.action_possible():
					if not self.plan(robot):
						# action not possible and couldn't find a new plan -> deadlocked, no action
						continue
				self.state[robot.pos[0]-1][robot.pos[1]-1] = 1 # mark old position as free
				self.perform_action(robot)
				self.state[robot.pos[0]-1][robot.pos[1]-1] = 0 # mark new position as blocked

		if self.benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length

	"""main function for shortest replanning strategy and all helper functions"""

	def replan(self, robot, out=0):
		"""Helper function used in shortest replanning strategy
		finds a new plan and returns the added length
		Argument out for benchmarking output (to know which replannigs belong together)
		out = 0 for replannigs where only one robot replans
		out = 1/2 for first/second robot when two robots replan
		"""
		robot.update_state(self.state)

		if self.benchmark:
			ts = time()
		robot.find_new_plan()
		if self.benchmark:
			tf = time()
			t = tf-ts
			self.solve_times.append(t)
			self.resolve_times.append(t)
			print("Rst"+str(out)+"=%s," %(t), file=sys.stderr, end='')
			print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')

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
		if robot not in self.to_check:
			self.to_check.append(robot)
		self.state[robot.next_pos[0]-1][robot.next_pos[1]-1] = 0

	def add_wait(self, r):
		self.print_verbose("r"+str(r.id)+" waits")
		r.wait()
		if r not in self.to_check:
			self.to_check.append(r)
		self.state[r.next_pos[0]-1][r.next_pos[1]-1] = 0

	def run_shortest_replanning(self):
		"""Run version using the shortest replanning conflict solving strategy
		In case of a conflict (where both robots move) both robots find a new plan
		but only the robot for which the new plan adds less time uses the new plan
		For conflicts where only one robot moves the other robot waits
		"""
		# self.to_check is a list of robots which still have to be checked for conflicts
		self.to_check = []

		# setup variables to express realtime in the benchmark
		if self.benchmark:
			rltime = []		# tracks the time for resolves
			stime = 0		# tracks the time for non-conflict solves
			for r in self.robots:
				rltime.append(0)

		while self.orders != [] or self.orders_in_delivery != []:
			self.t += 1

			if self.benchmark:
				stime = 0

			# check that every robot has a plan
			for r in self.robots:
				if self.benchmark:
					rltime[r.id - 1] = 0
				# no order asssigned or no plan because in a deadlock
				if (r.shelf == -1) or (r.next_action.name == ""):
					self.plan(r)

			# unmark all old positions
			for r in self.robots:
				self.state[r.pos[0]-1][r.pos[1]-1] = 1
			# mark all new positions
			for r in self.robots:
				self.state[r.next_pos[0]-1][r.next_pos[1]-1] = 0

			# initially all robots have to be checked
			self.to_check = list(self.robots)
			while self.to_check != []:
				r1 = self.to_check.pop(0) # get first robot and remove from list
				# check for possible conflicts with every other robot
				for r2 in self.robots:
					if r1.id == r2.id:
						continue
					# robots want to move onto same position or robots want to swap positions
					if (r1.next_pos == r2.next_pos) or ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
						if r1.next_pos == r2.next_pos:
							self.print_verbose("conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t))
						else:
							self.print_verbose("swapping conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t))

						#check if both robots move
						if (r1.next_action.name == "move") and (r2.next_action.name == "move"):
							# both robots move -> both have to find a new plan
							# self.replan returns added length of new plan
							if self.benchmark:
								if (rltime[r1.id - 1] > rltime[r2.id - 1]):
									rltime[r2.id - 1] = rltime[r1.id - 1]
								else:
									rltime[r1.id - 1] = rltime[r2.id - 1]
								r1start = time()
							dr1 = self.replan(r1, 1)
							if self.benchmark:
								r1end = time()
								rltime[r1.id - 1] += r1end - r1start
								r2start = time()
							dr2 = self.replan(r2, 2)
							if self.benchmark:
								r2end = time()
								rltime[r2.id - 1] += r2end - r2start
								if (rltime[r1.id - 1] > rltime[r2.id - 1]):
									rltime[r2.id - 1] = rltime[r1.id - 1]
								else:
									rltime[r1.id - 1] = rltime[r2.id - 1]

							# choose which robot uses new plan
							# case 1: both robots are deadlocked
							if ((dr1 == -1) and (dr2 == -1)):
								self.print_verbose("both robots deadlocked")
								# both have to use new plan (which causes them to wait next timestep)
								self.change_plan(r1)
								self.change_plan(r2)

							# case 2: r1 is deadlocked or the new plan of r1 adds more time
							elif (dr1 == -1) or (dr2 <= dr1 and dr2!=-1): # here dr2 can still be -1 -> then dr2<=dr1 would be true therefore the condition dr2!=-1 is needed
								self.print_verbose("r"+str(r1.id)+" deadlocked or dr"+str(r2.id)+"<=dr"+str(r1.id))
								# r1 continues using the old plan
								r1.use_old_plan()
								# r2 uses the new plan
								self.change_plan(r2)

							# case 3: r2 is deadlocked or the new plan of r2 adds more time
							elif (dr2 == -1) or (dr1 < dr2):
								self.print_verbose("r"+str(r2.id)+" deadlocked or dr"+str(r1.id)+"<=dr"+str(r2.id))
								# r1 uses new plan
								self.change_plan(r1)
								# r2 continues using the old plan
								r2.use_old_plan()

						else:
							# only one robot moves
							if r1.next_action.name == "move":
								self.print_verbose("r"+str(r2.id)+" delivers")
								self.add_wait(r1)
							elif r2.next_action.name == "move":
								self.print_verbose("r"+str(r1.id)+" delivers")
								self.add_wait(r2)

			if self.benchmark:
				self.real_time += max(rltime)
				#print("t" + str(self.t) + "rlt=" + str(ttime), file=sys.stderr, end='')

			# perform all next actions
			for robot in self.robots:
				if robot.next_action.name != "":
					self.perform_action(robot)

		if self.benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length

	"""main function for the crossing strategy and all helper functions"""

	def get_dodging_dir(self, r1, r2):
		"""determines in which direction r1 has to dodge
		also handels the special case that r1 doesn't actually have to dodge all the way"""
		t_finished = None # save time when r2 changes from path to crossing, r1 only has to dodge that long
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
						break # we have all directions we need so we can stop the loop
				# after going to the crossing r2 will leave the crossing in the next step -> + 1
				elif atom.arguments[2].number == r2.t + r1.cross_length + 1:
					move_from_cross = atom
					if move_to_cross is not None:
						break # we have all directions we need so we can stop the loop
				# to make sure that r2 is actually going to the crossing we have if its moves are the same as those of r1
				# all other moves (after the first and before going off the crossing) have to be the same as the previous move of r1
				if (atom.arguments[2].number > r2.t) and (atom.arguments[2].number <= (r2.t + r1.cross_length)):
					# r1.cross_model is sorted so r1.cross_model[t-1] will be the move at time t
					if ((atom.arguments[0].number != r1.cross_model[atom.arguments[2].number-r2.t-1].arguments[0].number) or
						(atom.arguments[1].number != r1.cross_model[atom.arguments[2].number-r2.t-1].arguments[1].number)):
						# if the move isn't the same we record the time, the earliest time determines how long r1 has to dodge
						if t_finished is None:
							t_finished = atom.arguments[2].number - r2.t
						elif atom.arguments[2].number - r2.t < t_finished:
							t_finished = atom.arguments[2].number - r2.t

		filtered_model = []

		# r1 has to dodge all the way
		if t_finished is None:
			# r1.cross_model can contain up to 4 moves off the crossing
			# 2 of those will be removed because of move_to_cross and move_from_cross
			# if after that there are still 2 moves one has to be chosen, first_move keeps track of that
			first_move = True
			for atom in r1.cross_model:
					if atom.name == "move":
						# because r1.cross_length is the time to the corssing, +1 will be the moves off the crossing
						if atom.arguments[2].number == r1.cross_length+1:
							# the direction from which r2 is coming isn't added to the model
							if ((atom.arguments[0].number == -1*move_to_cross.arguments[0].number) and
								(atom.arguments[1].number == -1*move_to_cross.arguments[1].number)):
								continue
							# the direction in which the r2 is moving isn't added to the model
							elif ((atom.arguments[0].number == move_from_cross.arguments[0].number) and
								(atom.arguments[1].number == move_from_cross.arguments[1].number)):
								continue
							# now atleast 1 direction is left but there could be a second one
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
					# only setps before t_finished are added to the model
					if atom.arguments[2].number <= t_finished:
						filtered_model.append(atom)

		r1.cross_model = list(filtered_model)

	def update_conflict_partners(self, r1, r2):
		# if r2 previously not in conflict, in_conflict flag needs to be set
		if not r2.in_conflict:
			r2.set_in_conflict(len(r1.cross_model))

		# update the conflict partners
		# old conflict partners of r1 have to be saved
		r1_old_partners = dict(r1.conflict_partners)
		# first update all partner of r1 and do this recursively for all partners of r1
		for p in r1.conflict_partners:
			r1.update_partners(p, len(r1.cross_model))
			for p2 in p.conflict_partners:
				p.update_partners(p2, len(r1.cross_model))
		# then add all partners of r2 to r1 and do so recursively for all partners of r1
		for p in r2.conflict_partners:
			r1.update_partners(p, len(r1.cross_model))
			for p2 in r1.conflict_partners:
				p2.update_partners(p, len(r1.cross_model))
		# lastly add all old_partners of r1 to r2 and recursively to all partners of r2
		for p in r1_old_partners:
			r2.update_partners(p, len(r1.cross_model))
			for p2 in r2.conflict_partners:
				p2.update_partners(p, len(r1.cross_model))

		return r1_old_partners

	def add_crossroad(self, r1, r2):
		self.print_verbose("r"+str(r1.id)+" dodges")

		# determine in which direction to dodge
		self.get_dodging_dir(r1,r2)

		r1_old_partners = self.update_conflict_partners(r1, r2)

		# generate cross_model for all old_partners of r1
		# process is different depending on if r1 moves in direction of its partners (so moves on some partner) or not
		# but this is only needed if r1 had partners previously
		if r1_old_partners:
			self.print_verbose("all conflict partners of r"+str(r1.id)+"also dodge")
			# get the next pos of r1
			# cant use r1.next_pos because r1 doesn't use the crossroad yet
			# r1 can't use the crossroad yet because there could be the possibility that moves still have to be added
			next_pos = list(r1.pos)
			for atom in r1.cross_model:
				if atom.name == "move":
					if atom.arguments[2].number == 1:
						next_pos[0] += atom.arguments[0].number
						next_pos[1] += atom.arguments[1].number
						break

			partner_dir = False
			# check if r1 moves onto any of its partners
			for p in r1_old_partners:
				if p.pos == next_pos:
					# in this case add_nested_dodge1 is used
					partner_dir = True
					self.add_nested_dodge12(p, r1)
					break
			# if r1 doesn't move onto any of its partners add_nested_dodge2 will be used
			if not partner_dir:
				for p in r1_old_partners:
					if p.next_pos == r1.pos:
						self.add_nested_dodge2(p, r1)
						break

		# add crossroad to the model, also sets in_conflict flag for r1
		self.change_crossroad(r1)
		# add crossroad recursively to all partner of r1
		for p in r1_old_partners:
			self.change_crossroad(p)

	def add_nested_dodge1(self, r1, r2):
		"""helper function for add_crossroad"""
		# r1 will copy the cross_model of r2
		# but first step will be removed
		# all other steps will be moved to one timestep earlier
		# and the last move is duplicated
		self.print_verbose("r"+str(r1.id)+" recursively dodges1")
		r1.cross_model = []
		for atom in r2.cross_model:
			# last is also duplicated and moved to one timestep earlier
			if atom.arguments[2].number == len(r2.cross_model):
				r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number-1, atom.arguments[3].number]))
				r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number, atom.arguments[3].number]))
			# all other moves but the first moved to one timestep earlier
			elif atom.arguments[2].number != 1:
				r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number-1, atom.arguments[3].number]))

		# get the next position of r1
		next_pos = list(r1.pos)
		for atom in r1.cross_model:
			if atom.name == "move":
				if atom.arguments[2].number == 1:
					next_pos[0] += atom.arguments[0].number
					next_pos[1] += atom.arguments[1].number
					break

		# recursively add crossroad to the next robot
		for p in r1.conflict_partners:
			if p.pos == next_pos:
				self.add_nested_dodge1(p, r1) """replaced by add_nested_dodge12, still needs testing"""

	def add_nested_dodge2(self, r1, r2, prev=[]):
		"""helper function for add_crossroad"""
		self.print_verbose("r"+str(r1.id)+" recursively dodges2")
		# r1 will copy the cross_model of r2
		# but a new first move is added (so all other moves have to be moved one timestep back)
		# and the last move has to be duplicated for r2 (not for r1)
		# compute the first move
		move_x = r2.pos[0] - r1.pos[0]
		move_y = r2.pos[1] - r1.pos[1]
		r1.cross_model = [clingo.Function("move", [move_x, move_y, 1, 1])]
		for atom in r2.cross_model:
			# copy all move with t+=1
			if atom.arguments[2].number <= len(r2.cross_model):
				r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number +1, atom.arguments[3].number]))

		# duplicate last move for r2 and all previous partners
		prev.append(r2)
		for r in prev:
			r.duplicate_last_move()

		# recursively add crossroad for the next robot
		for p in r1.conflict_partners:
			if p.next_pos == r1.pos:
				self.add_nested_dodge1(p, r1, prev)

	def add_nested_crossroad(self, r1, r2):
		r1_old_partners = self.update_conflict_partners(r1, r2)

		self.add_nested_dodge12(r1, r2)

		# add crossroad to the model, also sets in_conflict flag for r1
		self.change_crossroad(r1)
		# add crossroad recursively to all partner of r1
		for p in r1_old_partners:
			self.change_crossroad(p)

	def add_nested_dodge12(self, r1, r2):
		"""helper function for add_nested_crossroad"""
		# same functionality as add_nested_dodge1 but for a more general case
		# in add_nested_dodge1 r2 has to actually start with the cross_model
		# here r2 can already have made some moves from the cross_model
		self.print_verbose("r"+str(r1.id)+" recursively dodges12")
		r1.cross_model = []
		# how many steps does r2 still have to do from its cross_model
		steps_to_do = r2.cross_done - (r2.t-1)
		# if only one step
		if steps_to_do == 1:
			# this is then the last step, this will just be added to r1.cross_model but as the first move
			for atom in r2.cross_model:
				if atom.arguments[2].number == len(r2.cross_model):
					r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, 1, atom.arguments[3].number]))
		else:
			# the first move will be removed
			# all other moves will be moved to one timestep earlier
			# and the last move has to duplicated
			for atom in r2.cross_model:
				# the first move doesn't have to be added to r1.cross_model
				if atom.arguments[2].number > (len(r2.cross_model)-steps_to_do+1):
					# last is duplicated, and moved to one timestep earlier
					if atom.arguments[2].number == len(r2.cross_model):
						r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number-(len(r2.cross_model)-steps_to_do+1), atom.arguments[3].number]))
						r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number-(len(r2.cross_model)-steps_to_do+1)+1, atom.arguments[3].number]))
					# other moves are just moved to one timestep earlier
					else:
						r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number-(len(r2.cross_model)-steps_to_do+1), atom.arguments[3].number]))

		# compute the next_pos of r1
		next_pos = list(r1.pos)
		for atom in r1.cross_model:
			if atom.name == "move":
				if atom.arguments[2].number == 1:
					next_pos[0] += atom.arguments[0].number
					next_pos[1] += atom.arguments[1].number
					break

		# recursively add crossroad to the next robot
		for p in r1.conflict_partners:
			if p.pos == next_pos:
				self.add_nested_dodge12(p, r1)

	def add_wait2(self, r1, r2):
		# modification of add_wait function
		# here the robots have a list of all robots which they are waiting on
		# this is used to detect circular waiting, so situations where a robot is waiting on itself
		self.print_verbose("r"+str(r1.id)+" waits")
		r1.wait()
		if r1 not in self.to_check:
			self.to_check.append(r1)

		"""
		r1.waiting_on = [r2] + r2.waiting_on

		# check for circular waiting
		if r1.waiting_on[len(r1.waiting_on)-1] == r1:
			self.print_verbose("circular waiting")
			# the first wait will be reversed
			new_wait_partner = r1.waiting_on[len(r1.waiting_on)-2]
			# all waits are removed
			for r in r1.waiting_on:
				self.remove_wait(r)
			# inverse first waiting order
			self.print_verbose("order of first waiting reversed")
			self.add_wait2(r1, new_wait_partner)
		"""

	def remove_wait(self, r):
		self.print_verbose("r"+str(r.id)+" doesn't wait")
		r.action() # performs the action (=wait=nothing)
		if r not in self.to_check:
			self.to_check.append(r)

	def change_crossroad(self, r):
		# add the crossroad to the model, also generates the returning
		r.use_crossroad()
		if r not in self.to_check:
			self.to_check.append(r)

	def run_crossing(self):
		# for the crossing strategy the crossroad encoding has to be loaded
		# also some variables have to initialised
		for r in self.robots:
			r.init_crossing()

		# keep track of which robots we still have to check for conflicts
		self.to_check = []

		while self.orders != [] or self.orders_in_delivery != []:
			self.t += 1

			for r in self.robots:
				# if the robot doesn't have a order or was in a deadlock it needs to find a new plan
				if (r.shelf == -1) or (r.next_action.name == ""):
					self.plan(r)

			# first check for swapping conflicts and solve them
			self.to_check = list(self.robots)
			while self.to_check != []:
				r1 = self.to_check.pop(0)

				for r2 in self.robots:
					if r1.id == r2.id:
						continue

					if ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
						self.print_verbose("swapping conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t))
						# if r1 isn't in a conflict but r2 is already dodging another robot
						if (not r1.in_conflict) and (r2.dodging):
							# r1 will "copy" the dodging from r2 but has to make an additional step to dodge r2
							self.add_nested_crossroad(r1, r2)
						# analogous to the first case
						elif (r1.dodging) and (not r2.in_conflict):
							self.add_nested_crossroad(r2, r1)
						# if both are already dodging another robot
						elif (r1.dodging) and (r2.dodging):
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
							blocked = []
							if r1.in_conflict:
								blocked.append(list(r1.pos))
								for partner in r1.conflict_partners:
									blocked.append(list(r1.conflict_partners[partner]))
							if r2.in_conflict:
								blocked.append(list(r2.pos))
								for partner in r2.conflict_partners:
									blocked.append(list(r2.conflict_partners[partner]))
							r1.block_crossings(blocked)
							r2.block_crossings(blocked)

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

			# second check for non swapping conflicts and solve them by waiting
			self.to_check = list(self.robots)
			while self.to_check != []:
				r1 = self.to_check.pop(0)
				for r2 in self.robots:
					if r1.id == r2.id:
						continue

					if r1.next_pos == r2.next_pos:
						self.print_verbose("conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t))
						# the robot which moves and is not already in a conflict will wait
						if (r1.next_action.name == "move") and (not r1.in_conflict):
							self.add_wait2(r1, r2)
						elif (r2.next_action.name == "move") and (not r2.in_conflict):
							self.add_wait2(r2, r1)
						# if non fit this criterium an arbitrary robot waits
						else:
							# but we have to make sure that the robot actually moves
							if r1.next_action.name == "move":
								self.add_wait2(r1, r2)
							# no need to check if r2 is moving
							# (because r1 isn't moving but one robot has to move for there to be a conflict)
							else:
								self.add_wait2(r2, r1)

			# then the action are actually performed in self.state is updated with the new positions
			# first unmark all old positions
			for robot in self.robots:
				self.state[robot.pos[0]-1][robot.pos[1]-1] = 1
			# then perform the actions and mark new positions
			for robot in self.robots:
				# in case of a nested dodge it can happen that a robot would go off the field
				if (robot.next_pos[0]-1 < 0) or (robot.next_pos[1]-1 < 0) or (robot.next_pos[0]-1 > len(self.state)-1) or (robot.next_pos[1]-1 > len(self.state[0])-1):
					# in this case the program is exited with printing an error message
					sys.exit("r"+str(robot.id)+" would go off the field with move("+str(robot.next_action.arguments[0].number)+","+str(robot.next_action.arguments[1].number)+")")
				self.perform_action(robot)
				self.state[robot.pos[0]-1][robot.pos[1]-1] = 0

		if self.benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length

if __name__ == "__main__":
	parser = argparse.ArgumentParser()
	parser.add_argument("instance", help="the instance to be loaded")
	parser.add_argument("-n", "--nomodel", help="disables output of the model", default=False, action="store_true")
	parser.add_argument("-v", "--verbose", help="outputs additional information (printed to stderr)", default=False, action="store_true")
	parser.add_argument("-b", "--benchmark", help="use benchmark output (possible verbose output will be redirected to stdout)", default=False, action="store_true")
	parser.add_argument("-s", "--strategy", help="conflict solving strategy to be used (default: sequential)", choices = ['sequential','shortest','crossing'],default = 'sequential', type = str)
	parser.add_argument("-i", "--internal", help="disables use of external atoms", default=False, action="store_true")
	parser.add_argument("-H", "--Highways", help="generate highway tuples if they are not given in the instance", default = False, action = "store_true")
	parser.add_argument("-e", "--encoding", help="encoding to be used (default: ./pathfind.lp)", default = './pathfind.lp', type = str)
	args = parser.parse_args()
	benchmark = args.benchmark
	verbose_out = sys.stderr if not benchmark else sys.stdout

	# Initialize the Pathfind object
	if benchmark:
		t1 = time()
	pathfind = Pathfind(args.instance, args.encoding, not args.nomodel, args.verbose, verbose_out, benchmark, not args.internal, args.Highways)
	if benchmark:
		t2 = time()
		initTime = t2-t1
		print("It=%s," %(initTime), file=sys.stderr, end='') # Initial time

		groundTime = 0
		for t in pathfind.ground_times:
			groundTime += t
		print("Tgt=%s," %(groundTime), file=sys.stderr, end='') #Total ground time

		solveTimeInit = 0
		for t in pathfind.solve_times:
			solveTimeInit += t
		print("TstI=%s," %(solveTimeInit), file=sys.stderr, end='') # Total solve time in Init
		pathfind.solve_times = []

	# Start the execution of the plans
	# choose which run method is used according to args.strategy
	if benchmark:
		t1 = time()
	if args.strategy == 'sequential':
		pathfind.run_sequential()
	elif args.strategy == 'shortest':
		pathfind.run_shortest_replanning()
	elif args.strategy == 'crossing':
		pathfind.run_crossing()
	if benchmark:
		t2 = time()
		runTime = t2-t1
		print("Rt=%s," %(runTime), file=sys.stderr, end='') # Run time

		solveTimeRun = 0
		for t in pathfind.solve_times:
			solveTimeRun += t
		print("TstR=%s," %(solveTimeRun), file=sys.stderr, end='') # Total solve time in run
		print("Tst=%s," %(solveTimeInit+solveTimeRun), file=sys.stderr, end='') # Total solve time
		print("Trlt=%s," %(solveTimeInit+solveTimeRun+pathfind.real_time), file=sys.stderr, end='') # Total real time

		resolveTime = 0
		for t in pathfind.resolve_times:
			resolveTime += t
		print("Trst=%s," %(resolveTime), file=sys.stderr, end='') # Total resolve time
		print("Tt=%s" %(initTime+runTime), file=sys.stderr) # Total time
