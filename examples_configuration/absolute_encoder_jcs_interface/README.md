# absolute_encoder_jcs_interface

A single joint controller and JCS networked absolute encoder example.

## Overview

```
INFO   : ==============================================================================================================
INFO   : Device/process tree:
INFO   : 
INFO   : `- HOST                                            id: 0          address: 0          n_ports: 1 n_procs: 0
INFO   :    `- jc_0                                         id: 8          address: 33554432   n_ports: 1 n_procs: 0
INFO   :       `- encoder_0                                 id: 16         address: 33689600   n_ports: 0 n_procs: 0
INFO   : 
INFO   : ==============================================================================================================
```

This example demonstrates the operation of a JCS networked absolute encoder.

A tracking loop estimator provides position and velocity estimates, available through signals `th` and `w`.
The encoder raw position, normalised to [0, 2pi] and the encoder raw counts are available through signals `th_encoder`  and `th_counts`. 

Analog port 0 signal is renamed to `temperature_0` to reflect that it is scaled to interface to a temperature sensor, and is available through signal `temperature_0`.
Analog port 1 is scaled as a voltage and left as a default signal name of `an_1`.

In this example the motor controller (mc_0) connects to port 0 of the joint controller (jc_0) and a strain gauge interface (sg_0) connects to port 1 of the joint controller,
The joint controller connects back to the host computer (HOST).

The motor controller is configured to accept a torque producing current on it's Q-axis via host signal `host_mc_q_i`.

The motor controller will return it's actual torque producing Q-axis current `i_q` and the strain gauge interface returns scaled channel 0 data `sg_an_0`. The strain gauge data represents the force applied by a moment arm.

Obtaining the motor torque constant Kt, or the relationship between motor torque and current relies on some test rig that will convert the motors output to a linear force to be sensed by the force sensor.
It is then a matter of applying currents, recording the actual current and force, then converting the force into a torque.

