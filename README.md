# Nvidia Isaac Universal Robots RTDE communication

## Introduction
This example was created in collaboration with [Fravebot](https://www.fravebot.com/) company. This example is a simple demonstration of using Nvidia's Isaac Sim software to control a collaborative robot UR5e from Universal Robots.

```javascript
Software
------------------------------------
| Nvidia Isaac Sim version 2022.2.0
| Isaac Python version 3.7
|   - ur-rtde 1.5.5
```

1) [Nvidia Isaac Requirements](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/requirements.html)

2) [Nvidia Omniverse Isaac Instalation](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html)

3) Install [ur_rtde](https://gitlab.com/sdurobotics/ur_rtde) (Check docs Win/Lin). After install python bindings:

```console
user@user-pc:~/.../isaac_sim-2022.2.0$ ./python.sh -m pip install ur_rtde
```

## How-to-Start

1) Go to workspace Nvidia Omniverse Isaac
2) Clone repository
3) Run VM Polyscope or Real Robot. 
4) Run command (robot-ip is ip adress of robot VM or Real):

```console
user@user-pc:~/.../isaac_sim-2022.2.0$ ./python.sh test_fravebot/isaac_rtde.py --robot-ip 192.168.200.135
```

## Video!

<p align="center">
<a href="https://www.youtube.com/watch?v=IJtdp1wdPPo"><img src="https://upload.wikimedia.org/wikipedia/commons/0/09/YouTube_full-color_icon_%282017%29.svg" alt="Nvidia Isaac Sim Universal Robots UR5e RTDE"/></a>
</p>

## :information_source: Contacts

:mailbox: m.juricek@outlook.com
