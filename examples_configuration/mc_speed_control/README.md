# mc_speed_control

A single joint controller and motor controller with high bandwidth speed control.

## Overview

```
    INFO   : ==============================================================================================================
    INFO   : Device/process tree:
    INFO   : 
    INFO   : `- HOST                                            id: 0          address: 0          n_ports: 1 n_procs: 0
    INFO   :    `- jc_0                                         id: 8          address: 33554432   n_ports: 1 n_procs: 0
    INFO   :       `- mc_0                                      id: 16         address: 33558528   n_ports: 0 n_procs: 1
    INFO   :          |  - mc_0_spd_ctl                         id: 24         address: 33558592  
    INFO   : 
    INFO   : ==============================================================================================================

```

In this example the motor controller (mc_0) connects to port 0 of the joint controller (jc_0) and the joint controller connects back to the host computer (HOST).

A PID process (mc_0_spd_ctl) runs on the motor controller at `device rate`. For a motor controller the device rate is 15kHz.
The routing of the signals into and out of the PID process define the purpose of the process.
In this case, the PID process takes feedback from the motor controllers speed estimate (w_m_0) and takes a setpoint from a host signal.
The motor controller is configured to take a current command from the speed controller.
By defining feedback, setpoint and output signals in this manner, the PID process is acting as a speed controller. 

The speed controller PID process is configured similarly to devices - it has a yaml configuration file and the parameters are accessible via the parameter interface.
Since the speed controller output is a current command, output (and in turn, integrator) limits are configured.
The controller gains are also configured via the yaml configuration.

For debugging purposes, the speed controller output is connected back to the host.
Other interesting motor signals are returned at a rate of 10Hz.

## Setup

Configure the motor controller by following the steps in the `mc_torque_control` example, but omit the Kt parameters (or - leave them in if you prefer. The output of the speed controller will then connect to the motor controller torque input).

Once satisfactory current control performance is obtained, copy the motor controller configuration yaml into this directory to begin experimenting with speed control.


Start jcs_tool with this configuration:

```
sudo ./jcs_tool -p ../examples_configure/mc_speed_control/
```

### Notes on configuration

JCS configurations and parameters are not stored on the device.
If you would like a parameter to be configured at each startup, it must be present in the appropriate configuration file.