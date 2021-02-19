FROM nvidia/vulkan:1.1.121

RUN apt-get -y update

RUN apt-get -y install xauth

EXPOSE 8887

RUN useradd fsds

RUN usermod -m -d /fsds fsds

RUN mkdir /fsds

RUN chown fsds /fsds

RUN apt-get install wget -y

RUN apt-get install unzip -y

RUN apt-get -y install vulkan-utils

RUN wget -qO - http://packages.lunarg.com/lunarg-signing-key-pub.asc | apt-key add - && wget -qO /etc/apt/sources.list.d/lunarg-vulkan-1.1.121-bionic.list http://packages.lunarg.com/vulkan/1.1.121/lunarg-vulkan-1.1.121-bionic.list && apt update && apt install -y vulkan-sdk && apt upgrade -y && apt autoremove -y

RUN cd /fsds && wget -q -c https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator/releases/download/v2.0.0/fsds-v2.0.0-linux.zip 

RUN cd /fsds/ && unzip -q /fsds/fsds-v2.0.0-linux.zip

ENV TZ=Asia/Kolkata
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

RUN apt-get install git-all -y

RUN /bin/su -c "cd /fsds && git clone https://github.com/FS-Driverless/Formula-Student-Driverless-Simulator" - fsds

RUN /bin/su -c "touch .Xauthority" - fsds

COPY . /fsds 

CMD "sh /fsds/run.sh"
# /bin/su -c ./fsds-v2.0.0-linux/FSDS.sh - fsds

# RUN apt-get install checkinstall ros-kinetic-catkin python-catkin-tools git-all -y

# RUN sudo apt-get install git-all wget -y

# RUN sudo apt-get install wget -y

# RUN sudo apt-get install freeglut3 freeglut3-dev libxi-dev libxmu-dev

# RUN sudo apt-get install module-init-tools -y

# RUN wget -c "https://us.download.nvidia.com/XFree86/Linux-x86_64/460.39/NVIDIA-Linux-x86_64-460.39.run" 

# COPY . slam_path_planning 

# CMD "sh /root/slam_path_planning/run.sh"

