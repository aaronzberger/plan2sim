# Plan-to-Simulation Pipeline

Read high-level TACOMA plans and execute, interpret, and simulate them.

![Example Image](https://user-images.githubusercontent.com/35245591/177631133-9a3c89bd-fe19-427e-8dbe-5b82dd2417f3.png)

## Table of Contents
- [Usage](#Usage)
- [Pipeline](#Pipeline)
- [Conversion](#Conversion)
- [ROS Actions](#ROS-Actions)
  - [Custom Actions](#Custom-Actions)
- [Acknowledgements](#Acknowledgements)

## Usage
This node must be run with the other nodes in the [pipeline](#Pipeline).

A plan file is also required and an example is provided.

Clone to your `catkin_ws`, execute `catkin_make`, run your `roscore` and run

    `rosrun plan2sim main.py [PLAN FILE PATH]`

## Pipeline
This node is the first in the Plan-to-Simulation pipeline, responsible for interpreting the plan file and assigning tasks to subsystems at the specified times.

Each node is a separate ROS node, each receiving and publishing relevant data.

Code locations for the other nodes are listed below:
- [__SYSTEM CONTROLLERS__](https://github.com/aaronzberger/CMU_EKF_Node)
- [__MJ CONTROLLER__](https://github.com/aaronzberger/mj_controller)

## Conversion
The [`helpers.py`](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/helpers.py) file contains `TypedDict` objects corresponding exactly to the input format. The inputted plan file is passed to [`converter.py`](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/converter.py), where is it [deserialized](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/converter.py#L14:L28) and stored as a single `Plan` object.

The `Action`s are then placed in an [interval tree](https://github.com/aaronzberger/plan2sim/blob/main/src/interval_tree.py), which efficiently stores the time intervals for quick access during execution.

The `Action`s and `Constraint`s from the input file are also placed into hash tables for access during execution.

## ROS Actions
This pipeline uses ROS's [`actionlib`](http://wiki.ros.org/actionlib) library for communicating between nodes.

Like ROS services, actions begin with a single call to a server and end with a final communication back to the client. However, like ROS messages, during the execution of an action, the server sends a continuous stream of messages back to the client to indicate progress. Actions also allow for aborting and preempting actions, which is useful for a timeline with many constraints.

This node has [8 action clients](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L37:L44), one for each of the subsystems.

## Planner
After the conversion has finished, the planner is ready to begin. After [confirming](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L47:L54) the action servers are connected, we [start](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L56:L60) the simulation and [move](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L62:L77) the subsystems to their initial positions. The planner is then [started](https://github.com/aaronzberger/plan2sim/blob/main/src/main.py#L79:L83).

The planner is run at 1 Hz (since actions are only specified to start on the second). During each iteration,

  - We [query](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L162:L164) the interval tree to find which actions should be running.
  - Next, we [wait](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L176:L185) until the constraints for all the scheduled actions are satisfied.
  - Then, we [run](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L187) those actions, sending them to the action servers in the `System Controllers` node and awaiting [progress](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L101:L103) updates and a [final result](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L105:L117).

### Custom Actions
To determine which action client an action should be given to, we first determine the [state change](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L130): how the environment changes from executing the action (provided by preconditions and effects in the input file). We then map this state change to an action client via the [state mapper](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L134). However, if the resources needed to perform the action or the type of state change is unknown by the state mapper, the state mapper will yield no result, so the planner assumes this is a custom defined action:

Actions like spacecraft actions (turning the airflow off and on), must be customly defined in [`custom_actions.py`](https://github.com/aaronzberger/plan2sim/blob/main/src/custom_actions.py). The planner [calls](https://github.com/aaronzberger/plan2sim/blob/96c7b1dc42e1623735ce1c2248092bdf8e0a2a8d/src/main.py#L151) these actions and proceeds, whiule assuming the custom function executes correctly.

This process will be needed for each new plan, and simply reflects that the simulation will not perfectly represent the full state of the world. It optimizes for easy viewing and portability and does not understand tasks that involve resources or state changes that are unfamiliar.

## Acknowledgements
  - Zachary Rubinstein for plan file assistance and general feedback
  - Ashwin Misra for assistance in arm kinematics
  - Måns Magnusson for [`interval_tree.py`](https://github.com/moonso/interval_tree)
