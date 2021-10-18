#!/usr/bin/env python3

import os.path
import sys
#import socket

#leftovers from ROS 1
#default_remote_pc_ip='192.168.0.100'
#default_rover_ip='192.168.0.103'


joydev=''

for x in range(6):
    jsts = '/dev/input/js' + str(x)
    if os.path.exists(jsts):
        joydev=jsts
        break

if joydev == '':
    print('###############################')
    print('WARNING: Joypad not detected.  Joy nodes will not work.')
    print('###############################')
else:
    print('Joy device is', joydev)
    joydev = '--device=' + joydev

nc_env=''
if '-no-cache' in sys.argv:
    nc_env='--no-cache'

print('uid:', os.getuid())
print('gid:', os.getgid())

docparams='--network host -v $PWD:/ros -w /ros -v /dev/shm:/dev/shm'

fullbuildexec = 'docker build ' + nc_env + ' --build-arg USER_ID=' + str(os.getuid()) + ' --build-arg GROUP_ID=' + str(os.getgid()) + ' docker -t rmc:ros2'
os.system(fullbuildexec)
fullrunexec = 'docker run ' + docparams + ' ' + joydev + ' --rm -it rmc:ros2'
os.system(fullrunexec)