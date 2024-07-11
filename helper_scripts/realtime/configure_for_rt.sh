#
# Prepare for low latency tool from:
# https://github.com/OpenEtherCATsociety/SOEM/issues/171#issue-316624363

# ubuntu1710 4.13.0-38-lowlatency
# I211 , driver=igb , fast receive duration 20us => enp3s0
# I219-V , driver=e1000e , slow receive duration 70us...130us
# R8169 , driver=r8169 , fast receive duration 20us


# ----------------------------
# sudo gedit /etc/default/grub => add
# GRUB_CMDLINE_LINUX_DEFAULT="isolcpus=3 nohz_full=3 rcu_nocbs=3 rcu_nocb_poll elevator=noop nosmap  i915.enable_rc6=0 i915.enable_dc=0"

# ----------------------------
service irqbalance stop

# ----------------------------
ifname="enp3s0" # TOPTON router
# ifname="enp0s31f6" # Small Thinkcentre
# Clear rx/tx usecs for minimum latency
ethtool -L $ifname combined 1
ethtool -C $ifname rx-usecs 0 # default 3
ethtool -C $ifname tx-usecs 0 # default 3
sudo ethtool -K $ifname gso off gro off # generic-segmentation-offload , generic-receive-offload

# ----------------------------
# Elevate the priority of the network interrupt workers
new_irq_prio=97
for irq_pid in $(ps -e | grep $ifname | cut -d? -f1 | sed "s/ //g") ; do
        echo ifname: $ifname , irq_pid: $irq_pid elevate to pid $new_irq_prio
        chrt -f -p $new_irq_prio $irq_pid
done

# ----------------------------
# Pin the NIC IRQ workers to a single CPU.
# The NIC worker CPU should match the CPU that jcs_host is pinned to.
cpu_num=1
for irq_num in $(cat /proc/interrupts | grep $ifname | cut -d: -f1 | sed "s/ //g") ; do
        echo device: $ifname , irq: $irq_num map to cpu: $cpu_num
    echo $cpu_num >/proc/irq/$irq_num/smp_affinity_list
done

# ----------------------------
device="xhci_hcd"
cpu_num=3
irq_num=$(cat /proc/interrupts | grep $device | cut -d: -f1 | sed "s/ //g")
echo device: $device , irq: $irq_num map to cpu: $cpu_num
echo $cpu_num > /proc/irq/$irq_num/smp_affinity_list

# ----------------------------
device="i915"
cpu_num=2

irq_num=$(cat /proc/interrupts | grep $device | cut -d: -f1 | sed "s/ //g")
echo device: $device , irq: $irq_num map to cpu: $cpu_num
echo $cpu_num > /proc/irq/$irq_num/smp_affinity_list

# ----------------------------
echo 0 > /proc/sys/kernel/watchdog
