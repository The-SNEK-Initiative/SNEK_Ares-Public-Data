# SNEK_Ares-Public-Data
Research materials regarding the SNEK Ares project that we decide to disclose to the public.

## What is SNEK Ares??

Ares - Autonomous, Remote, Evaluatory, Succesionist missile defense system, it is a 64 bit NASM trajectory prediction system for missile guidance and targeting. Created by us to be something to outlast hardware shortages and capable of running on tiny amounts of resources, the .img itself fitting in 64kb and capable of running on almost any legacy hardware with a bios. Currently a closed source project, however we are willing and planning on releasing a bunch of the software we develop along with it, all falling under the same [LICENSE](https://github.com/The-SNEK-Initiative/SNEK_Ares-Public-Data/blob/main/LICENSE). 

# Data in this repo:

## [ares_hwsim.py](https://github.com/The-SNEK-Initiative/SNEK_Ares-Public-Data/blob/main/ares_hwsim.py)

Multi target hardware simulator for missile/defense tracking systems\
Generates telemetry data for 3D tracking with sensor faults

Running the sim: py/python ares_hwsim.py, connect on 127.0.0.1 port 4444 with your tracking client, simulates up to 6 target types, or more if you add them, with varying kinematics and RCS and injects track swaps, false targets, packet drops, and latency. Responds to missile launch guidance commands in addition to all that ;P

### Long live freeware, ATroubledSnake.
#### With love from The SNEK Initiative, muah <3
