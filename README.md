# SLAM-and-Path-Planning-For-Autonomous-Machines
SLAM (Simultaneous Localization And Mapping) is crucial for an autonomous system to locate itself in space and map out its surrounding environment. It can then make a decision of what to do i.e, "Plan its next move". This project is an implementation of the same.

- Authors: [Dhruval PB](http://github.com/Dhruval360), [Aditya NG](http://github.com/AdityaNG)

# Setup

Refered to https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/user-guide.html

## Systemd drop-in file

```bash
sudo mkdir -p /etc/systemd/system/docker.service.d

sudo tee /etc/systemd/system/docker.service.d/override.conf <<EOF
[Service]
ExecStart=
ExecStart=/usr/bin/dockerd --host=fd:// --add-runtime=nvidia=/usr/bin/nvidia-container-runtime
EOF

sudo systemctl daemon-reload \
  && sudo systemctl restart docker
```

## Daemon configuration file

The nvidia runtime can also be registered with Docker using the daemon.json configuration file:

```bash
sudo tee /etc/docker/daemon.json <<EOF
{
    "runtimes": {
        "nvidia": {
            "path": "/usr/bin/nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
EOF

sudo pkill -SIGHUP dockerd
```


# Docker 

Build command 

```bash
sudo docker build -t slam_path_planning:v2 .
```

To run the container

```bash
docker run --runtime=nvidia -it -d --gpus all --net=host -e DISPLAY -v /tmp/.X11-unix -e NVIDIA_DRIVER_CAPABILITIES=all --env DDISPLAY_COOKIE="(DISPLAY_COOKIE)" slam_path_planning:v2 /bin/sh /fsds/run.sh
```

Replace (DISPLAY_COOKIE) with the your machine's xauth display cookie, it should look like the following: 

```bash
xauth list
username/machine:0 MIT-MAGIC-COOKIE-1 [32 character string]
```


