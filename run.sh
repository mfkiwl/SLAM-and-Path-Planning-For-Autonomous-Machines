

/bin/su -c "xauth add $DISPLAY_COOKIE" - fsds

/bin/su -c /fsds/fsds-v2.0.0-linux/FSDS.sh - fsds

exit

if grep '^VERSION="16.04.7 LTS (Xenial Xerus)"$' /etc/os-release ; then
	echo "Updating..."
	git fetch --all
	git reset --hard origin/main
	git pull

	git checkout testing2

	/bin/su -c /fsds/fsds-v2.0.0-linux/FSDS.sh - fsds
else

	echo "Not inside ROS Container; Halting"
	exit

fi
