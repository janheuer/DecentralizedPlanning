% M�gliche Probleme
% Was, wenn sich Roboter gegenseitig im Weg stehen?
% Wann neuen Pfad berechenen? Bei allen �nderungen oder nur, wenn der derzeitige Pfad gest�rt wird?
% Wie in python simulieren? Wie sagen, dass �nderungen ankommen und wann.
% Atome direkt hinzuf�gen oder nur Instanzdatei �ndern?
% Mehrere Roboter => Gleichzeitige Zugriffe auf die Instanz => mutex? Simulieren durch Threads?
% Pos zum Zeitpunkt der Pfad�nderung auslesen, neue Information einarbeiten, alte Pos als neuen Startpunkt definieren und dann neu grounden/solven
% assign_external(atom(), True) -> not atom()
% clingo.parse.term()
% Start von Anfang an external -> kann rein und rausgenommen werden

% Alle Beispiele noch Python 2, Python 3 wird aber ben�tigt

%for x in prg.symbolic_atoms: # �ber alle #externals iterieren
%		if x.is_external == True:
%			print (x.symbol,  x.literal)
%			prg.assign_external(clingo.Function(x,[x]), True)

% Wie f�r alle M�glichkeiten allgemein nutzbar machen?
% Instanzdatei, auf der wir mit python lesen/schreiben und in der die derzeit aktive Instanz ist
% F�r jedes external eine Variable und True/False setzen, wenn aktiv/nicht aktiv




#script (python)

import clingo


def assign_var(prg, list, name, arguments):
	for x in list:
		if x[0] == name and x[1][0].number == arguments[0] and x[1][1].number == arguments[1]:
			# Wenn wir das gesuchte Element gefunden haben
			x[2] = True
			prg.assign_external(clingo.Function(x[0], x[1]), True)
	return list


def release_var(prg, list, name, arguments):
	return list

def main(prg):
	prg.ground([("base", [])])
	atomlist = []
	for x in prg.symbolic_atoms:
		if x.is_external == True: # �ber alle #externals iterieren
			atomlist.append([x.symbol.name, x.symbol.arguments, False])

	#list = assign_var(prg, atomlist, "block", [1, 3])

	prg.assign_external(clingo.Function("block", [2,2]), True)
	#prg.assign_external(clingo.Function("block", [2,3]), True)
	prg.assign_external(clingo.Function("block", [2,4]), True)
	prg.assign_external(clingo.Function("block", [2,6]), True)
	#prg.assign_external(clingo.Function("block", [3,4]), True)
	#prg.assign_external(clingo.Function("block", [3,7]), True)
	prg.assign_external(clingo.Function("block", [3,8]), True)
	prg.assign_external(clingo.Function("block", [4,1]), True)
	#prg.assign_external(clingo.Function("block", [4,2]), True)
	prg.assign_external(clingo.Function("block", [5,3]), True)
	prg.assign_external(clingo.Function("block", [6,1]), True)
	#prg.assign_external(clingo.Function("block", [7,3]), True)
	prg.assign_external(clingo.Function("block", [8,3]), True)

	prg.assign_external(clingo.Function("start", [8,1]), True)
	prg.assign_external(clingo.Function("goal", [1,1]), True)
	prg.assign_external(clingo.Function("size", [8,8]), True)

	#prg.solve()

	goal_pos = [8,8]
	goal = False
	solved = False
	t = 0

	while(not goal):
		if (not solved):
			opt_model = []
			with prg.solve(yield_=True) as h:
				for m in h:
					opt = m
				for atom in opt.symbols(shown=True):
					opt_model.append(atom)

		for atom in opt_model:
			if (atom.name == "pos" and atom.arguments[2]==t):
				current_pos = atom.arguments[:2]
				if current_pos == goal_pos:
					goal = True
					break
			elif (atom.name == "move" and atom.arguments[2]==t+1):
				move_dir = atoms.arguments[:2]
		# if current_pos+move_dir blocked?
		# then solved=False, external start ändern
		# else t+=1

	#prg.assign_external(clingo.Function("block", [2,1]), True)
	#prg.assign_external(clingo.Function("start", [4,3]), True)

	#prg.release_external(clingo.Function("start", [8,1]))

	#prg.solve()

#end.



#external size(8, 8).
#external block(1..X, 1..Y) : size(X, Y).


#external start(1..X, 1..Y) : size(X, Y).
#external goal(1, 1).
