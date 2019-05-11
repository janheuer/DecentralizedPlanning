# Decentralized Planning in the *asprilo* Framework

## Structure

**Python control structure**

pathfind.py - The general python framework 

robot.py - Implements functionality for the robots

**Planning encodings**

pathfind.lp - The general planning encoding 

crossroad.lp - Special encoding for the crossing strategy

**Centralized version**

The centralized branch includes a centralized version based on the same python framework using the same planning encoding (but it only works for instances where the number of robots equals the number of robots).

## Instance Format

The input and output format is the normal [**asprilo**](<https://potassco.org/asprilo>) format.
Additionally a horizon for the instance is required given by `time(1..N).`

## Usage

Python (tested with version 2.7) and the python module of [**clingo**](<https://github.com/potassco/clingo>) are required.

```bash
python pathfind.py instance
```

The conflict solving strategy used, a custom planning encoding and different output options can be specified via command line options.

To get a list of all options run:
```bash
python pathfind.py --help
```
