FROM adityang5/fsd:vnc
COPY . slam_path_planning 

# RUN sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv 2930ADAE8CAF5059EE73BB4B58712A2291FA4AD5

# RUN sudo apt update
# RUN sudo apt upgrade

RUN sudo apt-get install checkinstall ros-kinetic-catkin python-catkin-tools git-all -y

#CMD ["sh", "/root/slam_path_planning/run.sh"]

CMD "sh /root/slam_path_planning/run.sh"

