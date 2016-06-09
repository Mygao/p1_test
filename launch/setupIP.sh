
export ROS_MASTER_URI=http://$1:11311
export ROS_IP=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')
export ROS_HOSTNAME=$(ip addr | grep 'state UP' -A2 | tail -n1 | awk '{print $2}' | cut -f1  -d'/')
export ROSLAUNCH_SSH_UNKNOWN=1
