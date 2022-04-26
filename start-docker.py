#!/usr/bin/env python3

import os
import sys
import grp
import json

config_file_name = "lbl_config.json"

#load json config
if not os.path.isfile(config_file_name):
    ex_config_data = open(config_file_name + ".example", "r")
    config_data_file = open(config_file_name, "w")
    config_data_file.write(ex_config_data.read())
    config_data_file.close()
    config_data_file = open(config_file_name, "r")
    ex_config_data.close()

else:
    config_data_file = open(config_file_name, "r")

config_data = json.loads(config_data_file.read())
config_data_file.close()

#Docker file template:
docker_header = """#################################
# THIS FILE IS AUTO-GENERATED
# DO NOT EDIT DIRECTLY
#################################

FROM ros:foxy

ARG USER_ID
ARG GROUP_ID
ARG VIDEO_GROUP_ID

RUN addgroup --gid $GROUP_ID user
RUN addgroup --gid $VIDEO_GROUP_ID video_host || true
RUN adduser --disabled-password --gecos '' --uid $USER_ID --gid $GROUP_ID user

RUN usermod -aG sudo user
RUN usermod -aG $VIDEO_GROUP_ID user
RUN echo "user ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

RUN apt-get update
RUN apt-get upgrade -y
#we used to install ros-noetic-joystick-drivers, but something broke with it.
#ros-noetic-joy should have the drivers we need though...
RUN echo "KERNEL==\"video[0-9]*\",MODE=\"0666\"" > /usr/lib/udev/rules.d/99-camera.rules

USER user

RUN /bin/bash /opt/ros/foxy/setup.bash
"""

docker_footer = """COPY disable_fastdds_shm.xml /tmp/
RUN rosdep update
RUN rosdep install -i --from-path /ros/dev_ws/src --rosdistro foxy -y
RUN echo "source /ros/dev_ws/install/setup.bash || true" >> /home/user/.bashrc
"""

docker_file_string = docker_header

def adddockerfile(file_to_add):
    global docker_file_string
    docker_file_string += 'ADD ' + file_to_add + ' /ros/' + file_to_add + '\n'

def genlaunchstring(launch_file):
    return 'RUN echo "ros2 launch ' + launch_file + ' && exit" >> /home/user/.bashrc\n'

#construct dockerfile
package_dirs=[]
search_dir = os.path.join('dev_ws', 'src')
for fn in os.listdir(search_dir):
    if os.path.isfile(os.path.join(search_dir,fn,'package.xml')):
        print('Found package ', fn)
        package_dirs.append(fn)


for pkg in package_dirs:
    docker_file_string += 'ADD dev_ws/src/' + pkg + '/package.xml /ros/dev_ws/src/' + pkg + '/package.xml\n'

docker_file_string += docker_footer

launch_type_string = "shell"
if config_data["default-launch-type"]:
    launch_type_string = config_data["default-launch-type"]

if '-launch-type' in sys.argv:
    launch_type_string = sys.argv[sys.argv.index('-launch-type')+1]

print("Launch type:", launch_type_string)

if launch_type_string == "shell":
    pass
elif launch_type_string == "autonomy":
    docker_file_string += genlaunchstring('launch/rmc_launch_autonomy.py')
elif launch_type_string == "teleop-min":
    docker_file_string += genlaunchstring('launch/rmc_launch_teleop_min.py')
elif launch_type_string == "build":
    docker_file_string += 'RUN echo "/ros/scripts/build_all.sh && exit" >> /home/user/.bashrc\n'
else:
    print("Invalid launch type.")
    quit()

df = open('Dockerfile', 'w')
df.write(docker_file_string)
df.close()
#dockerfile constructed

#joystick detection
joydev=''

for x in range(6):
    jsts = '/dev/input/js' + str(x)
    if os.path.exists(jsts):
        joydev=jsts

if joydev == '':
    print('###############################')
    print('WARNING: Joypad not detected.  Joy nodes will not work.')
    print('###############################')
else:
    print('Joy device is', joydev)
    joydev = '--device=' + joydev
#end joystick

#camera detection
cameradev=''

for x in range(6):
    campath = '/dev/video' + str(x)
    if os.path.exists(campath):
        cameradev += ' --device=' + campath + ' '
        print('Adding camera:', campath) 

if cameradev == '':
    print('###############################')
    print('WARNING: Camera not detected.  Camera nodes will not work.')
    print('###############################')
#end camera

#microcontroller detection
mcudev = ''

for x in range(6):
    mcupath = '/dev/ttyACM' + str(x)
    if os.path.exists(mcupath):
        mcudev += ' --device=' + mcupath + ' '
        print('Adding Microcontroller:', mcupath)

if mcudev == '':
    print('###############################')
    print('WARNING: No microcontrollers detected.  Micro-ROS and position tracking will not work.')
    print('###############################')
#end microcontroller detection

nc_env=''
if '-no-cache' in sys.argv:
    nc_env='--no-cache --pull'

print('uid:', os.getuid())
print('gid:', os.getgid())
print('video gid:', grp.getgrnam('video').gr_gid)

docparams='--net=host -v $PWD:/ros -w /ros -e FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/disable_fastdds_shm.xml ' 

fullbuildexec = 'docker build -f Dockerfile ' + nc_env + ' --build-arg USER_ID=' + str(os.getuid()) + ' --build-arg GROUP_ID=' + str(os.getgid()) + ' --build-arg VIDEO_GROUP_ID=' + str(grp.getgrnam('video').gr_gid) + ' . -t rmc:ros2'
os.system(fullbuildexec)
fullrunexec = 'docker run ' + docparams + ' ' + joydev + cameradev + mcudev + ' --rm -it rmc:ros2'
os.system(fullrunexec)
