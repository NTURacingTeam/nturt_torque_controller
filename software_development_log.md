# National Taiwan University Racing Team Software Development Log
###### tags: `development_log` `NTURT`
##### Group: electrical system group
##### Person in charge: 羅紀翔
##### Authors: 羅紀翔 黃柏瑞
##### Subsystem: RPI
##### Subsystem number: RP4
##### Software name: torque_controller
##### Repository: [github](https://github.com/NTURacingTeam/nturt_torque_controller.git)
##### Started designing date: 2022/3/20
##### Current version: 1.0
##### Last modified date: 2022/8/28
---

## Engineering goal:

Control inverter through can signal, and implement simple control logic such as accelerator/brake plausibility check and soft start.

## Program structure:

TODO

## Included libraries:

-

## Testing environment:

- ros noetic
- docker virtual environment from [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker) with image `ros_matlab`, `ros_rpi` based on ubuntu20.04

##### Testing hardware:

- asus tuf gaming a15 FA506II-0031A 4800H
- raspberry pi 3B+

##### Operating system:

- ubuntu 20.04
- raspbian 32-bit

##### Compiler(intepreter) version:

- gcc 9.4.0 (Ubuntu 9.4.0-1ubuntu1~20.04.1)

---

## Testing result of 1.0:

still needs extensive testing with an inverter (or a fake one)

## Todos in 1.0:
