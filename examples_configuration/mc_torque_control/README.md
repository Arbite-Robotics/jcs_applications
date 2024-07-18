# mc_torque_control

A single joint controller and motor controller operating in torque control mode.

## Overview

```
    INFO   : ==============================================================================================================
    INFO   : Device/process tree:
    INFO   : 
    INFO   : `- HOST                                            id: 0          address: 0          n_ports: 1 n_procs: 0
    INFO   :    `- jc_0                                         id: 8          address: 33554432   n_ports: 1 n_procs: 0
    INFO   :       `- mc_0                                      id: 16         address: 33558528   n_ports: 0 n_procs: 0
    INFO   : 
    INFO   : ==============================================================================================================
```

In this example the motor controller (mc_0) connects to port 0 of the joint controller (jc_0) and the joint controller connects back to the host computer (HOST).

The motor controller takes a torque command input from the host and returns motor speed at a rate of 2kHz.

Other motor signals are returned at a rate of 10Hz.

The motor is configured with parameter `motor_Kt` to map the incoming torque into a motor current.

## Setup

To begin, the following is true:
- The motor controller parameter `encoder_0_position_offset` is commented out, as the encoder has not been zeroed. 
- The motor controller parameter `motor_Kt` is commented out, as we need to do tuning tests. 
- The start_list in dev_HOST.yaml is commented out as the motor controller is not ready to start (encoder is not zeroed).

Wire up the system and power it on.

Start jcs_tool with this configuration:

```
sudo ./jcs_tool -p ../examples_configure/mc_torque_control/
```

### Find initial current controller gains

We must first find some initial current controller gains.

```
- Navigate to the motor controller (mc_0), then to the Tuning tab.

- Click to enable tuning.

- Set some D and Q axes voltages.
Note: The voltages are dependent on motor winding resistance. If the motor is a high current type, start with 1V. 

- Click to measure phase resistance and inductance.

- The controller will measure the resistance and inductance of the D and Q axes.

Once this is successful, set a current controller bandwidth.
The tool will calculate the D and Q axes current controller gains.

- Write the gains to the controller, then try a step response.

- Iterate on the controller bandwidth until the step response is acceptable.

- Copy the current controller gains into the motor controller configuration file.

- Uncomment the parameter `motor_Kt`
```

### Zero the encoder

Once acceptable controller gains are found we must map the encoder position to the motor's D-axis.

```
- Navigate to the motor controller (mc_0), then to the Encoder tab.

- Click to enable the motor controller for zeroing.

- Enter a D-Axis alignment current. 
Note: D-axis alignment current requirements are dependent on the motor type. For high cogging torque motors, currents of 5A or more may be required.
Watch the motor to ensure that it rotates to a position and holds for the duration of the encoder zeroing time.

- Copy the encoder offset into the motor configuration file.
```

### Run the motor

```
- Click `START` to start jcs system.

- Navigate to the motor controller (mc_0), then to the parameter tab.

- Set `controller_mode` to `current_dq`. Write the parameter.

- Write `controller_start`.
```

The motor controller is now running.

Navigate to HOST, then to the Signal Plot tab.

You should see signals signals coming into jcs_host from the motor controller.
Adjust the motor torque by sliding `HOST::host_mc_tau`.

The motor should spin!

Uncomment `mc_start_i_ctl` list in dev_HOST.yaml to configure and enable the motor controller when calls are made to `start` in JCS.


### Troublshooting

#### Estop
If the system has an E-Stop, clear the error.
Click RESET to reset devices, then click STOP to put all devices into stop mode.
Click START to start the system again.

If running on a current limited power supply, most estops are caused by hitting the power supply current limit.
This causes the supply voltage to drop, and an undervoltage error to be generated.

##### Motor problems

You may encounter any of the following problems:

- Motor just locks and does nothing.
- For a small input command, motor just speeds up uncontrollably.

It is possible the phases are backwards.
Configure (or set via parameter) `motor_phase_swap` to swap motor phases internally.
Note: You should re-zero the encoder after swapping phases.

It is also possible the encoder is backwards.
Configure (or set via parameter) `encoder_0_direction` to change the encoder direction.
Note: You should re-zero the encoder after changing motor direction.

It is possible that the encoder is not zeroed correctly.
Increase the D-Axis alignment current and re-run zeroing.


### Notes on configuration

JCS configurations and parameters are not stored on the device.
If you would like a parameter to be configured at each startup, it must be present in the appropriate configuration file.