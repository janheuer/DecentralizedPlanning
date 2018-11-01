from simulator import *

class SimulatorCostum(Simulator):
    def __init__(self):
        super(Simulator, self).__init__()
        self._port = 5001
        self._name = 'simulator'
        self._viz_done = -1 # which steps is already visualized
        self._steps = -1 # how many steps have to be visualized

    def run(self, steps):
        self._steps = steps
        if self._connection is None:
            print 'Start ' + self._name
            self.connect()
        else:
            self.on_connect()
        while(True):
            if self._viz_done == steps: # when all steps are visualized reset everything
                self._to_send = {}
                self._sended = -1
                self._viz_done = -1
                self._steps = -1
                return
            if self.receive(1.0) != 0:
                return

    def on_control_symbol(self, symbol):
        if symbol.name == 'reset':
            self._reset = True
            self._to_send = {}
            self._data = []
        elif symbol.name == 'done' and len(symbol.arguments) == 1:
            self._viz_done = symbol.arguments[0].number # update which step has been visualized
            if self._steps == self._viz_done: # when all steps are visualized no new message has to be send
                return
            try:
                self.send_step(symbol.arguments[0].number + 1)
            except:
                return

    def add_inits(self, inits):
        # inits enthaelt alle inits im benoetigten format
        self._to_send[0] = []
        for atom in inits:
            self._to_send[0].append(atom)

    def add(self, rid, name, args, t):
        # atom muss in richtiges format umgewandelt werden
        t2 = t if t!=1 else 0 # action zu t=1 muss in liste fuer t=0
        if t2 not in self._to_send:
            self._to_send[t2] = []

        txt = "occurs(object(robot,"+str(rid)+"),action("+name+",("
        if name == "move":
            txt += str(args[0])+","+str(args[1])
        elif name == "deliver":
            txt += str(args[0])+","+str(args[1])
        txt += ")),"+str(t)+")"

        self._to_send[t2].append(txt)
