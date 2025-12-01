from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.utils.timerfd import timerfd_settime

# get network interface name from terminal command: ifconfig
interface = "enp129s0"
ChannelFactoryInitialize(0, interface)

sport_client = SportClient()
sport_client.SetTimeout(10.0)
sport_client.Init()
