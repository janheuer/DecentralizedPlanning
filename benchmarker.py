import clingo

from typing import List
from json import dumps
from pathlib import Path


def solve(prg: clingo.Control) -> List[clingo.Symbol]:
    model: List[clingo.Symbol] = []
    found_model: bool = False
    with prg.solve(yield_=True) as h:
        for m in h:
            found_model = True
            opt = m
        if found_model:
            for atom in opt.symbols(shown=True):
                model.append(atom)
    return model


class Benchmarker(object):
    def __init__(self, strategy: str, instance: str, domain: str, result_path: str) -> None:
        self.result_path: str = result_path + "/" + strategy + "/" + domain + "/" + instance[:-3] + "/"
        Path(self.result_path).mkdir(parents=True, exist_ok=True)

        self.counter: int = 0

    def output(self, stats: dict, type: str) -> None:
        file: str = self.result_path
        if type == "main":
            file += "main" + ".json"
        else:
            if type != "plan":
                return
            # add key to track type of solving (i.e. assignment, plan, conflict)
            stats["type"] = type
            file += str(self.counter) + ".json"
        with open(file, 'w+') as f:
            print(dumps(stats, sort_keys=True, indent=4, separators=(',', ': ')), file=f)
        self.counter += 1

    def solve(self, prg: clingo.Control, type: str) -> List[clingo.Symbol]:
        results: List[clingo.Symbol] = solve(prg)
        self.output(prg.statistics, type)
        return results
