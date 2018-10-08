
#script (python)

import clingo


def output(actlist):
	for act in actlist:
		print("occurs(object(robot,1),action(move,("+str(act[1])+","+str(act[0])+")),"+str(act[2])+").")
	return
	


def main(prg):
	prg.ground([("base", [])])

	
	# festlegen der instanz
	size = [5,5]
	goal = [1,1]
	start = [5,1]
	# warteschlange von states: jeder besteht aus zeitpunkt des eintritts und matrix mit blockierten feldern
	states = [(0, [	[0,0,0,0,0],
					[0,1,0,0,0],
					[1,0,0,1,0],
					[0,0,1,0,0],
					[0,0,1,0,0]]),
	 	  (4, [		[0,0,0,0,0],
					[1,0,0,0,0],
					[1,0,1,1,0],
					[0,0,1,0,0],
					[0,0,1,0,0]]),
		  (5, [		[0,1,0,0,0],
					[0,0,0,0,0],
					[1,0,1,1,0],
					[0,0,1,0,0],
					[0,0,1,0,0]])]

	# externals festlegen
	prg.assign_external(clingo.Function("size", size), True)
	prg.assign_external(clingo.Function("start", start), True)
	prg.assign_external(clingo.Function("goal", goal), True)

	# initialisierung für schleife
	current_state = states[0][1] # speichert im moment blockierte felder
	states = states[1:] # aktuellen state aus warteschlange entfernen
	current_pos = start
	goal_reached = False
	solved = False # zum bestimmen wann erneut solve aufgerufen wird
	t = 1 # überprüfen der schritte beginnt ab t=1 (da bei t=0 keine bewegung)
	t_offset = 0 # zeit offset zum speichern der gesamt vergangenen zeit, da t für jedes solve zurückgesetzt wird

	# für print ausgaben in der schleife am besten mit clingo option --outf=3 ausführen
	# ausgaben zeigen gemachte bewegung an und wann auf ein blockiertes feld gelaufen würde
	
	actlist = []
	
	while(not goal_reached):
		if states != [] and states[0][0] == t+t_offset:
			# block atome müssen aktualisiert werden
			current_state = states[0][1] # speicher des neuen states
			states = states[1:] # entfernen des states aus der warteschlange
		if (not solved):
			solved = True
			opt_model = []
			# assign aller blocks
			for i in range(len(current_state)):
				for j in range(len(current_state[0])):
					prg.assign_external(clingo.Function("block", [i+1,j+1]), current_state[i][j])
			prg.assign_external(clingo.Function("start", start), False) # alter start wird falsch
			prg.assign_external(clingo.Function("start", current_pos), True) # neuer start wird true
			start = current_pos
			print("Solving")
			with prg.solve(yield_=True) as h:
				# auswählen des letzten (also des optimalen) modells
				for m in h:
					opt = m
				# kopieren des modells
				for atom in opt.symbols(shown=True):
					opt_model.append(atom)

		for atom in opt_model:
			if (atom.name == 'pos' and atom.arguments[2].number == t-1): # position am ende des vorherigen zeitschritts
				current_pos = [atom.arguments[0].number,atom.arguments[1].number]
				if current_pos == goal: # falls zeil erreicht ist kann schleife abgebrochen werden
					goal_reached = True
					#print("The goal has been reached")
					output(actlist)
					return
			elif (atom.name == "move" and atom.arguments[2].number == t): # move welcher in diesem zeitschritt ausgeführt werden soll
				move_dir = [atom.arguments[0].number,atom.arguments[1].number]

		next_x = current_pos[0]+move_dir[0]
		next_y = current_pos[1]+move_dir[1]
		if current_state[next_x-1][next_y-1]:
			# position auf die sich der roboter bewegen will ist blockiert
			#print("pos("+str(current_pos[0])+","+str(current_pos[1])+","+str(t+t_offset-1)+")")
			#print("move("+str(move_dir[0])+","+str(move_dir[1])+","+str(t+t_offset)+") would move robot onto block("+str(next_x)+","+str(next_y)+")")
			solved = False # es muss erneut gesolved werden
			# t wird auf 1 zurückgesetzt, speichern der vergangenen zeit im offset
			t_offset += t-1
			t = 1
		else:
			# move kann durchgeführt werden
			#print("pos("+str(current_pos[0])+","+str(current_pos[1])+","+str(t+t_offset-1)+")")
			#print("move("+str(move_dir[0])+","+str(move_dir[1])+","+str(t+t_offset)+")")
			actlist.append([move_dir[0], move_dir[1], t+t_offset])
			t += 1 # sonst nächsten zeitschritt prüfen

	
#end.

#external block(1..X, 1..Y) : size(X, Y).
#external start(1..X, 1..Y) : size(X, Y).
#external goal(1, 1).
