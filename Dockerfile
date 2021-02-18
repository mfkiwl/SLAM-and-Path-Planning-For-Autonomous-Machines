# docker run -it -d --gpus all  nvidia/vulkan:1.1.121 /bin/bash

FROM adityang5/fsd:vnc

RUN sudo apt-get install checkinstall ros-kinetic-catkin python-catkin-tools -y

RUN sudo apt-get install git-all wget -y

RUN sudo apt-get install wget -y

RUN sudo apt-get install freeglut3 freeglut3-dev libxi-dev libxmu-dev

RUN sudo apt-get install module-init-tools -y

RUN wget -c "https://us.download.nvidia.com/XFree86/Linux-x86_64/460.39/NVIDIA-Linux-x86_64-460.39.run" 

# RUN pkill -SIGTERM -f lxsession && sudo sh NVIDIA-Linux-x86_64-460.39.run 

RUN wget https://developer.download.nvidia.com/compute/cuda/11.2.1/local_installers/cuda_11.2.1_460.32.03_linux.run

RUN sudo apt-get install pciutils -y

RUN sudo sh cuda_11.2.1_460.32.03_linux.run --silent --toolkit

RUN cd / && wget -c https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.0.0/fsds-v2.0.0-linux.zip

RUN cd / && unzip /fsds-v2.0.0-linux.zip

RUN useradd fsds

RUN chown fsds /fsds-v2.0.0-linux

COPY . slam_path_planning 

RUN cd /root/slam_path_planning/ && sh /root/slam_path_planning/run.sh

CMD "sh /root/slam_path_planning/run.sh"

