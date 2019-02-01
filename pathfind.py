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

	def run_shortest_replanning(self):
		"""Run version using the shortest replanning conflict solving strategy
		In case of a conflict (where both robots move) both robots find a new plan
		but only the robot for which the new plan adds less time uses the new plan
		For conflicts where only one robot moves the other robot waits
		"""
		# self.to_check is a list of robots which still have to be checked for conflicts
		self.to_check = []

		while self.orders != [] or self.orders_in_delivery != []:
			self.t += 1

			# check that every robot has a plan
			for r in self.robots:
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
						if self.verbose:
							if r1.next_pos == r2.next_pos:
								print("conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
							else:
								print("swapping conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)

						#check if both robots move
						if (r1.next_action.name == "move") and (r2.next_action.name == "move"):
							# both robots move -> both have to find a new plan
							# self.replan returns added length of new plan
							dr1 = self.replan(r1, 1)
							dr2 = self.replan(r2, 2)

							# choose which robot uses new plan
							# case 1: both robots are deadlocked
							if ((dr1 == -1) and (dr2 == -1)):
								if self.verbose:
									print("both robots deadlocked", file=verbose_out)
								# both have to use new plan (which causes them to wait next timestep)
								self.change_plan(r1)
								self.change_plan(r2)

							# case 2: r1 is deadlocked or the new plan of r1 adds more time
							elif (dr1 == -1) or (dr2 <= dr1 and dr2!=-1): # here dr2 can still be -1 -> then dr2<=dr1 would be true therefore the condition dr2!=-1 is needed
								if self.verbose:
									print("r"+str(r1.id)+" deadlocked or dr"+str(r2.id)+"<=dr"+str(r1.id), file=verbose_out)
								# r1 continues using the old plan
								r1.use_old_plan()
								# r2 uses the new plan
								self.change_plan(r2)

							# case 3: r2 is deadlocked or the new plan of r2 adds more time
							elif (dr2 == -1) or (dr1 < dr2):
								if self.verbose:
									print("r"+str(r2.id)+" deadlocked or dr"+str(r1.id)+"<=dr"+str(r2.id), file=verbose_out)
								# r1 uses new plan
								self.change_plan(r1)
								# r2 continues using the old plan
								r2.use_old_plan()

						else:
							# only one robot moves
							if r1.next_action.name == "move":
								if self.verbose:
									print("r"+str(r2.id)+" delivers and r"+str(r1.id)+" will wait", file=verbose_out)
								r1.wait()
								self.state[r1.next_pos[0]-1][r1.next_pos[1]-1] = 0
							elif r2.next_action.name == "move":
								if self.verbose:
									print("r"+str(r1.id)+" delivers and r"+str(r2.id)+" will wait", file=verbose_out)
								r2.wait()
								self.state[r2.next_pos[0]-1][r2.next_pos[1]-1] = 0

			# perform all next actions
			for robot in self.robots:
				if robot.next_action.name != "":
					self.perform_action(robot)

		if self.benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length

	def get_dodging_dir(self, r1, r2):
		# detemine in which direction r1 has to dodge
		# get direction of r2 moving onto crossing and direction of r2 moving from crossing -> new function ***************

		# if the other robot doesn't actually go to the crossing (for example moves to a pickingstation)
		# the robot doesn't need to dodge completely
		t_finished = None # save time when robot changes from path to crossing
		for atom in r2.model:
			if atom.name == "move":
				#r2.t-1 = timesteps r2 has completed, r1.cross_length = time r1 takes to crossing +1 because r2 takes one step more to crossing +1 beacuse we need the next move atom
				if atom.arguments[2].number == r2.t-1 + r1.cross_length +1:
					move_to_cross = atom
				if atom.arguments[2].number == r2.t-1 + r1.cross_length +1 +1:
					move_from_cross = atom
					break # we have all directions we need so we can stop the loop
			else:
				if (t_finished is None) and (atom.arguments[0].number > self.t):
					# we didn't reach the crossing but have a action atom
					t_finished = atom.arguments[0].number-1 - r2.t # r2.t-1 ???????
					break # the robot has left the path to the crossing so we don't need to look at the next atoms
		# cross_model contains moves for all possible direction off the crossing
		# one needs to be chosen
		first_move = True
		filtered_model = []
		if t_finished is None:
			for atom in r1.cross_model:
					if atom.name == "move":
						if atom.arguments[2].number == r1.cross_length+1:
							# remove the direction from which the other robot is coming
							if ((atom.arguments[0].number == -1*move_to_cross.arguments[0].number) and
								(atom.arguments[1].number == -1*move_to_cross.arguments[1].number)):
								continue
							# remove the direction in which the other robot is moving
							elif ((atom.arguments[0].number == move_from_cross.arguments[0].number) and
								(atom.arguments[1].number == move_from_cross.arguments[1].number)):
								continue
							# now atleast 1 direction is left but there could be a second one
							# the first direction will be used
							elif first_move:
								first_move = False
								filtered_model.append(atom)
							# the possible other direction will be removed
						else:
							filtered_model.append(atom)
		else:
			for atom in r1.cross_model:
				if atom.name == "move":
					if atom.arguments[2].number <= t_finished:
						filtered_model.append(atom)

		r1.cross_model = list(filtered_model)

	def get_nested_dodge(self, r1, r2):
		t_return = r2.cross_length / 2
		r1.cross_length = t_return
		r1.cross_model = []
		for atom in r2.cross_model:
			if atom.arguments[2].number > t_return: # second half of plan not needed
				break
			if atom.arguments[2].number != 0: # arguments t changed to t-1
				r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number-1, atom.arguments[3].number]))
				if atom.arguments[2].number == t_return: # last move before returning is needed two times
					r1.cross_model.append(clingo.Function(atom.name, [atom.arguments[0].number, atom.arguments[1].number, atom.arguments[2].number, atom.arguments[3].number]))

	def run_crossing(self):
		self.to_check = []

		while self.orders != [] or self.orders_in_delivery != []:
			self.t += 1

			for r in self.robots:
				if (r.shelf == -1) or (r.next_action.name == ""):
					self.plan(r)

			for r in self.robots:
				self.state[r.pos[0]-1][r.pos[1]-1] = 1
			for r in self.robots:
				self.state[r.next_pos[0]-1][r.next_pos[1]-1] = 0

			self.to_check = list(self.robots)
			while self.to_check != []:
				r1 = self.to_check.pop(0)

				for r2 in self.robots:
					if r1.id == r2.id:
						continue

					if r1.next_pos == r2.next_pos:
						if self.verbose:
							print("conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
						if (r1.next_action.name == "move") and (not r1.in_conflict): # not r1.in_conflict
							if self.verbose:
								print("r"+str(r1.id)+" waits", file=verbose_out)
							r1.wait()
							self.state[r1.next_pos[0]-1][r1.next_pos[1]-1] = 0
						else:
							if self.verbose:
								print("r"+str(r2.id)+" waits", file=verbose_out)
							r2.wait()
							self.state[r2.next_pos[0]-1][r2.next_pos[1]-1] = 0

					if ((r1.next_pos == r2.pos) and (r1.pos == r2.next_pos)):
						if self.verbose:
							print("swapping conflict between "+str(r1.id)+" and "+str(r2.id)+" at t="+str(self.t), file=verbose_out)
						# first check for cases where one of the robots is already in a conflict
						if (r1.in_conflict and not r2.in_conflict):
							if self.verbose:
								print("r"+str(r1.id)+" already in conflict so r"+str(r2.id)+" has to dodge", file=verbose_out)
							self.get_nested_dodge(r2, r1)
							r2.use_crossroad()
							if r2 not in self.to_check:
								self.to_check.append(r2)
							self.state[r2.next_pos[0]-1][r2.next_pos[1]-1] = 0
						elif (not r1.in_conflict and r2.in_conflict):
							if self.verbose:
								print("r"+str(r2.id)+" already in conflict so r"+str(r1.id)+" has to dodge", file=verbose_out)
							self.get_nested_dodge(r1, r2)
							r1.use_crossroad()
							if r1 not in self.to_check:
								self.to_check.append(r1)
							self.state[r1.next_pos[0]-1][r1.next_pos[1]-1] = 0
						# TODO: case for when both robots are already in conflict (only possible if new conflicts is at a crossing)
						# none of the robots are already in a conflict
						else:
							r1.in_conflict = True
							r2.in_conflict = True
							r1.conflict_partner = r2
							r2.conflict_partner = r1

							# new functions for finding the crossroad -> add benchmarking as well *****************************
							r1.update_state(self.state)
							r1.find_crossroad()
							r2.update_state(self.state)
							r2.find_crossroad()

							if (r1.cross_length < r2.cross_length and r1.cross_length != -1) or (r2.cross_length == -1):
								#print(r1.cross_model)
								if self.verbose:
									print("r"+str(r1.id)+" dodges", file=verbose_out)
								self.get_dodging_dir(r1,r2)
								#print(r1.cross_model, file=verbose_out)
								r1.use_crossroad() # this also generate rest of plan (returning to start)
								#print("model with returning: ",end='', file=verbose_out)
								#print(r1.cross_model, file=verbose_out)
								if r1 not in self.to_check:
									self.to_check.append(r1)
								self.state[r1.next_pos[0]-1][r1.next_pos[1]-1] = 0

							else:
								#print(r2.cross_model)
								if self.verbose:
									print("r"+str(r2.id)+" dodges", file=verbose_out)
								self.get_dodging_dir(r2,r1)
								#print(r2.cross_model, file=verbose_out)
								r2.use_crossroad()
								#print("model with returning: ",end='', file=verbose_out)
								#print(r2.cross_model, file=verbose_out)
								if r2 not in self.to_check:
									self.to_check.append(r2)
								self.state[r2.next_pos[0]-1][r2.next_pos[1]-1] = 0


			for robot in self.robots:
				if robot.using_crossroad: # if robot is the dodging robot in a conflict
					self.perform_action(robot)
					if not robot.using_crossroad: # not dodging anymore after the action
						# -> conflict resolution is finished -> both robots aren't in conflict anymore
						robot.in_conflict = False
						if robot.conflict_partner is not None:
							robot.conflict_partner.in_conflict = False
				else:
					self.perform_action(robot)

		if self.benchmark:
			print("Tpl="+str(self.t)+",", file=sys.stderr, end='') # Total plan length

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

	def finish_order(self, order, shelf):
		"""Releases the shelf and removes the order from orders_in_delivery"""
		self.release_shelf(shelf)
		self.orders_in_delivery.remove(order)

	def release_order(self, order):
		"""Removes the order from list of orders which are currently being delivered
		and adds it to the list of order which are open
		This funtion is needed for situations in which the robot is deadlocked
		in his start position and can't start the delivery of the order
		"""
		self.orders_in_delivery.remove(order)
		self.orders.append(list(order))

	def reserve_order(self, order):
		"""Adds the order to list of orders which are currently being delivered
		and removes from list of orders which are open
		"""
		self.orders_in_delivery.append(order)
		self.orders.remove(order)

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
			print("R" + str(robot.id) + " at (" + str(robot.pos[0]) + "," + str(robot.pos[1]) + "), t=" + str(self.t) + ",", file=sys.stderr, end='')

		if found_plan: # if the robot found a plan the shelf has to be reserved
			self.reserve_shelf(robot.shelf)
			return True
		else: # robot couldn't find a plan
			if robot.shelf == -1:
				# robot couldn't start planning the order (because he deadlocked in his start position)
				# release the order so that other robots can try to plan it
				self.release_order(robot.order)
			return False

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

		resolveTime = 0
		for t in pathfind.resolve_times:
			resolveTime += t
		print("Trst=%s," %(resolveTime), file=sys.stderr, end='') # Total resolve time
		print("Tt=%s" %(initTime+runTime), file=sys.stderr) # Total time
