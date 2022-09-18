# NTURT torque controller

## Introduction

Receiving can signal from accelerator/brake pedals, then send can signal to the inverter with respect to those pedal signals. 

Especially, it implements `accelerator pedal plausibility check` and `brake pedal plausibility check` subject to FSAE rule.

In addition, it implements soft start when rpm of the motor is low, which gradually increase the torque output of the motor.

## Usage

The node may be run by

```bash=
rosrun nturt_torque_controller torque_controller_node
```

This node receive can message from `/received_messages` topic and send can message to `/sent_messages` that work with node `socketcan_bridge_node` from `socketcan_bridge` package.

### Accelerator/brake pedal plausibility check

TODO