class Printer(object):
    def add_inits(self, inits):
        for atom in inits:
            print(atom + ".")

    def add(self, rid, name, args, t):
        # for wait no atom is printed
        if name != "wait":
            txt = "occurs(object(robot,"+str(rid)+"),action("+name+",("
            if name == "move":
                txt += str(args[0])+","+str(args[1])
            elif name == "deliver":
                txt += str(args[0])+","+str(args[1])
            txt += ")),"+str(t)+")."

            print(txt)
