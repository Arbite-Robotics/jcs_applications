#!/bin/bash

# From:
# https://github.com/mikekaram/ether_ros/blob/master/docs/source/guide.rst
# https://github.com/mikekaram/ether_ros/blob/master/scripts/optimizations/reinstall_e1000e_wo_throttling.sh
# https://github.com/mikekaram/ether_ros/blob/master/scripts/optimizations/set_realtime_priority_irqeth.sh
#

# Set the network driver interrupt to something higher than the RT process
irq_pid=$(ps -e | grep "enp3s0" | grep -o -E '[0-9]+' | head -n 1)
irq_num=$(ps -e | grep "enp3s0" | grep -o -E '[0-9]+' | head -n 5 | tail -1)
# Set the priority of the interrupt thread
echo "IRQ PID " $irq_pid
echo "IRQ number " $irq_num
# Set a high priorty to ensure it is serviced
chrt -f -p 97 $irq_pid
# And pin the irq worker to cpu 1
echo 1 | tee /proc/irq/$irq_num/smp_affinity_list

# Dont forget to ensure irqbalance daemon is disabled, or it will just re-balance
#
# sudo systemctl stop irqbalance
# sudo systemctl disable irqbalance
