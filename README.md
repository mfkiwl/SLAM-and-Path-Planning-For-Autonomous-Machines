# SLAM-and-Path-Planning-For-Autonomous-Machines
SLAM (Simultaneous Localization And Mapping) is crucial for an autonomous system to locate itself in space and map out its surrounding environment. It can then make a decision of what to do i.e, "Plan its next move". This project is an implementation of the same.

<img src=imgs/cropped_lidar1.gif>

- Authors: [Dhruval PB](http://github.com/Dhruval360), [Aditya NG](http://github.com/AdityaNG)

# Docker 

Build command 

```bash
sudo docker build -t slam_path_planning:v1 .
```

To run the container

```bash
sudo docker run -it --rm -p 5900:5900 slam_path_planning:v1
```

Access the VNC server at localhost:5900. Once inside the container, the following command will update and run the project 

```bash
sh ~/slam_path_planning/run.sh
```
