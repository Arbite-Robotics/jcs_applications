# system_16dof_torque_controller

Four joint controllers, each with four motor controller operating in torque control mode.

## Overview

```
INFO   : ==============================================================================================================
INFO   : Device/process tree:
INFO   : 
INFO   : `- HOST                                            id: 0          address: 0          n_ports: 1 n_procs: 0
INFO   :    |- jc_0                                         id: 8          address: 33554432   n_ports: 4 n_procs: 0
INFO   :    |  |- mc_00                                     id: 16         address: 33558528   n_ports: 0 n_procs: 0
INFO   :    |  |- mc_01                                     id: 24         address: 33624064   n_ports: 0 n_procs: 0
INFO   :    |  |- mc_02                                     id: 32         address: 33689600   n_ports: 0 n_procs: 0
INFO   :    |  `- mc_03                                     id: 40         address: 33755136   n_ports: 0 n_procs: 0
INFO   :    |- jc_1                                         id: 48         address: 67108864   n_ports: 4 n_procs: 0
INFO   :    |  |- mc_10                                     id: 56         address: 67112960   n_ports: 0 n_procs: 0
INFO   :    |  |- mc_11                                     id: 64         address: 67178496   n_ports: 0 n_procs: 0
INFO   :    |  |- mc_12                                     id: 72         address: 67244032   n_ports: 0 n_procs: 0
INFO   :    |  `- mc_13                                     id: 80         address: 67309568   n_ports: 0 n_procs: 0
INFO   :    |- jc_2                                         id: 88         address: 100663296  n_ports: 4 n_procs: 0
INFO   :    |  |- mc_20                                     id: 96         address: 100667392  n_ports: 0 n_procs: 0
INFO   :    |  |- mc_21                                     id: 104        address: 100732928  n_ports: 0 n_procs: 0
INFO   :    |  |- mc_22                                     id: 112        address: 100798464  n_ports: 0 n_procs: 0
INFO   :    |  `- mc_23                                     id: 120        address: 100864000  n_ports: 0 n_procs: 0
INFO   :    `- jc_3                                         id: 128        address: 134217728  n_ports: 4 n_procs: 0
INFO   :       |- mc_30                                     id: 136        address: 134221824  n_ports: 0 n_procs: 0
INFO   :       |- mc_31                                     id: 144        address: 134287360  n_ports: 0 n_procs: 0
INFO   :       |- mc_32                                     id: 152        address: 134352896  n_ports: 0 n_procs: 0
INFO   :       `- mc_33                                     id: 160        address: 134418432  n_ports: 0 n_procs: 0
INFO   : 
INFO   : ==============================================================================================================
```

In this example the motor controllers (mc_xx) connects to ports 0,1,2,3 of the joint controllers (jc_x) (one motor controller per port).
The joint controllers are daisy chained and connect back to the host computer (HOST).

The motor controllers each take a torque command input from the host and return motor speed at a rate of 2kHz.

Other motor signals are returned at a rate of 10Hz.

The motor is configured with parameter `motor_Kt` to map the incoming torque into a motor current.