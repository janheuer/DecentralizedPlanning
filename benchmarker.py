import clingo
from typing import List
from json import dumps
from pathlib import Path


def solve(prg: clingo.Control):
    model: List[clingo.Symbol] = []
    with prg.solve(yield_=True) as h:
        for m in h:
            opt = m
        for atom in opt.symbols(shown=True):
            model.append(atom)
    return model


class Benchmarker(object):
    def __init__(self, strategy: str, instance: str, result_path: str):
        self.result_path: str = result_path + "/" + strategy + "/" + instance[:-3] + "/"
        Path(self.result_path).mkdir(parents=True, exist_ok=True)

        self.counter: int = 0

    def output(self, stats: dict, type: str):
        file: str = self.result_path
        if type == "main":
            file += "main" + ".json"
        else:
            # add key to track type of solving (i.e. assignment, plan, conflict)
            stats["type"] = type
            file += str(self.counter) + ".json"
        with open(file, 'w+') as f:
            print(dumps(stats, sort_keys=True, indent=4, separators=(',', ': ')), file=f)
        self.counter += 1

    def solve(self, prg: clingo.Control, type: str):
        results: clingo.Model = solve(prg)
        self.output(prg.statistics, type)
        return results