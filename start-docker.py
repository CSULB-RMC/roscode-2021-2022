#!/usr/bin/env python3

import os
import sys

#Docker file template:
docker_header = """FROM ros:foxy-ros-base

ARG USER_ID
ARG GROUP_ID

RUN addgroup --gid $GROUP_ID user
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user

RUN usermod -aG sudo user
RUN echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update
#RUN apt-get install -y xboxdrv ros-foxy-joy ros-foxy-joy-linux
#we used to install ros-noetic-joystick-drivers, but something broke with it.
#ros-noetic-joy should have the drivers we need though...

USER user

RUN /bin/bash /opt/ros/foxy/setup.bash
"""

docker_footer = """RUN rosdep update
RUN rosdep install -i --from-path /ros/dev_ws/src --rosdistro foxy -y
"""

#construct dockerfile
package_dirs=[]
search_dir = os.path.join('dev_ws', 'src')
for fn in os.listdir(search_dir):
    if os.path.isfile(os.path.join(search_dir,fn,'package.xml')):
        print('Found package ', fn)
        package_dirs.append(fn)

docker_file_string = docker_header
for pkg in package_dirs:
    docker_file_string += 'ADD dev_ws/src/' + pkg + '/package.xml /ros/dev_ws/src/' + pkg + '/package.xml\n'

docker_file_string += docker_footer

df = open('Dockerfile', 'w')
df.write(docker_file_string)
df.close()
#dockerfile constructed

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

docparams='--network host -v $PWD:/ros -w /ros -v /dev/shm:/dev/shm --device=/dev/video0:/dev/video0' 

fullbuildexec = 'docker build -f Dockerfile' + nc_env + ' --build-arg USER_ID=' + str(os.getuid()) + ' --build-arg GROUP_ID=' + str(os.getgid()) + ' . -t rmc:ros2'
os.system(fullbuildexec)
fullrunexec = 'docker run ' + docparams + ' ' + joydev + ' --rm -it rmc:ros2'
os.system(fullrunexec)
