SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
cd $SCRIPT_DIR

export DISPLAY=:0
echo kyon | sudo -S XAUTHORITY=/run/user/$(id -u gdm)/gdm/Xauthority xhost +si:localuser:root
docker compose up -d
docker compose exec zed-ros-noetic bash -ic "roslaunch zed_wrapper zedxm.launch camera_name:=zed_down camera_id:=1"
