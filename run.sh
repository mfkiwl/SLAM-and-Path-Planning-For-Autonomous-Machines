cd ~/slam_path_planning

# 16.04.7 LTS (Xenial Xerus)

if grep '^VERSION="16.04.7 LTS (Xenial Xerus)"$' /etc/os-release ; then
	echo "Updating..."
	git fetch --all
	git reset --hard origin/main
	git pull

	source ~/fsd_skeleton/fsd_environment.sh
	
	roslaunch fssim_interface fssim.launch &
else

	echo "Not inside ROS Container; Halting"
	exit

fi
