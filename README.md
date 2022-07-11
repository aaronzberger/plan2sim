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
First, the plan file provided is interpreted by the conversion module:

  - Each task is converted to an `Action`, which specifies the subsystem, positions, times, and constraints.
  - Next, these `Action`s are placed in an [interval tree](https://github.com/aaronzberger/plan2sim/blob/main/src/interval_tree.py), which efficiently stores the time intervals for quick access during execution.
  - The `Action`s are also placed into a hash table for access during execution.

## ROS Actions
This pipeline uses ROS's [`actionlib`](http://wiki.ros.org/actionlib) library for communicating between nodes.

Like ROS services, actions begin with a single call to a server and end with a final communication back to the client. However, like ROS messages, during the execution of an action, the server sends a continuous stream of messages back to the client to indicate progress. Actions also allow for aborting and preempting actions, which is useful for a timeline with many constraints.

This node has [8 action clients](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L37:L44), one for each of the subsystems.

## Planner
After the conversion has finished, the planner is ready to begin. After [confirming](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L47:L54) the action servers are connected, we [start](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L56:L60) the simulation and [move](https://github.com/aaronzberger/plan2sim/blob/24c989b6fe3d5ed62dafd7a8910adda9d8a6511a/src/main.py#L62:L77) the subsystems to their initial positions. The planner is then [started](https://github.com/aaronzberger/plan2sim/blob/main/src/main.py#L79:L83).

The planner is run at 1 Hz (since actions are only specified to start on the second). During each iteration,

  - We [query](https://github.com/aaronzberger/plan2sim/blob/main/src/main.py#L133:L139) the interval tree to find which actions should be running.
  - Next, we [wait](https://github.com/aaronzberger/plan2sim/blob/main/src/main.py#L140:L161) until the constraints for all the scheduled actions are satisfied.
  - Then, we [run](https://github.com/aaronzberger/plan2sim/blob/main/src/main.py#L110:L128) those actions, sending them to the action servers in the `System Controllers` node and awaiting [progress](https://github.com/aaronzberger/plan2sim/blob/main/src/main.py#L92:L94) updates and a [final result](https://github.com/aaronzberger/plan2sim/blob/main/src/main.py#L96:L108).

### Custom Actions
When running the tasks specified in the plan file, we make an assumption that the task changes the state of the world and can therefore be concretely simulated by moving a joint to a position. However, this does not always hold true. For example, a space station executive turning the airflow off is not useful to simulate and does not involve moving any motor. Instead, this node recognizes that this task is different and asks the user to specify its implementation.

During the [conversion](#Conversion), this node prints warnings for any task it deems needs a special implementation. The user must then write these implementations in [`custom_actions.py`](https://github.com/aaronzberger/plan2sim/blob/main/src/custom_actions.py) by making a new function, and mapping the task's name to that function in the [dictionary](https://github.com/aaronzberger/plan2sim/blob/main/src/custom_actions.py#L7:L11).

This process will be needed for each new plan, and simply reflects that the simulation will not perfectly represent the full state of the world. It optimizes for easy viewing and portability and cannot possibly understand every possible task.

## Acknowledgements
  - Zachary Rubinstein for plan file assistance and general feedback
  - Ashwin Misra for assistance in arm kinematics
  - Måns Magnusson for [`interval_tree.py`](https://github.com/moonso/interval_tree)
