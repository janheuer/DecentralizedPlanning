# Decentralized Planning in the *asprilo* Framework

This project implements several decentralized approaches to planning in the [**asprilo**](<https://potassco.org/asprilo>) framework.

- Sequential, Shortest Replanning, and Crossing:

The general idea is that orders are assigned to the robots and each robot then plans its own order without taking into account the other robots.
During the execution of the plans conflicts are solved according to different strategies.
In sequential one of the robots in a conflict has to find a new plan. 
In shortest replanning both robots find a new plan but only the robot for which the new plan adds less timesteps uses its new plan.
In crossing both robots find a plan to dodge the other robot in order to resolve the conflict and the robot for which the dodging adds less timesteps then actual uses its plan.

- Prioritized:

Here the robots also only plan for themselves but they get the plans of other robots as an additional input.
Therefore, this strategy does not lead to conflicts in the plans.

- Traffic:

This strategy requires a special instance in which only one-way lanes exist. 
As a result no swapping conflicts between robots can occur.
All other conflicts are resolved by making one of the robots wait.

For more details see the following project reports:

[Jan Heuer and Cedric-John Martens: Decentralize Planning in the asprilo Framework, 2019](https://github.com/janheuer/DecentralizedPlanning/files/14181350/report2019.pdf)

[Jan Heuer and Cedric-John Martens: Avoiding Conflicts in Decentralized Planning, 2022](https://github.com/janheuer/DecentralizedPlanning/files/14181352/report2022.pdf)

## Structure

`pathfind.py` - The general python framework providing the control structure (distributing orders, simulating the plan execution and resolving conflicts)

`robot.py` - Implements functionality for the robots

`benchmarker.py` - Provides the actual solving function (using the [**clingo**](<https://github.com/potassco/clingo>) module) as well as a wrapper of the solving function for the benchmarking

`encodings/` contains all the encodings used for planning which are based on the [**asprilo encodings**](<https://github.com/potassco/asprilo-encodings>)

## Instance Format

The input format is the normal [**asprilo format**](<https://github.com/potassco/asprilo/blob/master/docs/specification.md#input-format>) (for domain A). However product quantities are ignored (but need to be specified in the input). 

Additionally the atom `nextto/3` (stating which nodes are connected) has to be specified in the instance. Using the `traffic` strategy requires that only nodes with shelves and neighbouring highways are connected in two ways. All other nodes have to form one-way lanes. Encodings to generate the `nextto/3` atoms are given in the `instances/` directory (for generating a `traffic` instance there are some additional constraints to the layout of the instance).

The output format follows the [**asprilo format**](<https://github.com/potassco/asprilo/blob/master/docs/specification.md#output-format>) for domain B or domain M depending on which domain was chosen via the command line options.

`instances/graph/` contains several example instances for the strategies `sequential, shortest, crossing, prioritized,` and `centralized`. 
For some of the instances corresponding `traffic` compatible instances can be found in `instances/traffic/`.

## Usage

```bash
python pathfind.py instance
```

Python (tested with version 3.7) and the python module of [**clingo**](<https://github.com/potassco/clingo>) are required.
The conflict solving strategy used, the domain and different output options can be specified via command line options.

To get a list of all options run:
```bash
python pathfind.py --help
```
