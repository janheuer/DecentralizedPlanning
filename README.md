# Decentralized Planning in the *asprilo* Framework

This project implements a decentralized approach to planning in the [**asprilo**](<https://potassco.org/asprilo>) framework.

The genral idea is that orders are assigned to the robots and each robot then plans its own order without taking into account the other robots.
During the execution of the plans conflicts are solved according to one of the implemented strategies (sequential, shortest replanning, crossing).

## Structure

**Python control structure**

pathfind.py - The general python framework distributing orders, simulating the plan execution and resolving conflicts

robot.py - Implements functionality for the robots (planning using the [**clingo**](<https://github.com/potassco/clingo>) module)

**Planning encodings**

pathfind.lp - The general planning encoding 

crossroad.lp - Special encoding for the crossing strategy

**Centralized version**

The centralized branch includes a centralized version based on the same python framework using the same planning encoding (but it only works for instances where the number of robots equals the number of robots) for comparing the decentralized approach.

## Instance Format

The input format is the normal [**asprilo format**](<https://github.com/potassco/asprilo/blob/master/docs/specification.md#input-format>) (for domain A). However product quantities are ignored (but need to be specified in the input).

Additionally a horizon for the instance has to be provided by a fact `time(1..N).`

The output format follows the [**asprilo format**](<https://github.com/potassco/asprilo/blob/master/docs/specification.md#output-format>) for domain B.

`./instances/` contains several example instances.

## Usage

```bash
python pathfind.py instance
```

Python (tested with version 2.7) and the python module of [**clingo**](<https://github.com/potassco/clingo>) are required.
The conflict solving strategy used, a custom planning encoding and different output options can be specified via command line options.

To get a list of all options run:
```bash
python pathfind.py --help
```
