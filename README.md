# WeGo


# 31번 라인
if [[ "$TERM" == *color* ]]; then
    color_prompt=yes
fi
# 맨 아래에 추가
source /opt/ros/noetic/setup.bash # ros를 초기화.
source ~/WeGO/catkin_ws/devel/setup.bash
source /usr/share/gazebo/setup.bash # gazebo를 초기화.
export XDG_RUNTIME_DIR=/run/user/$(id -u)
mkdir -p /run/user/$(id -u)
chmod 700 /run/user/$(id -u)

export ROS_MASTER_URL=http://localhost:11311
export ROS_IP=$(hostname -I | awk '{print $1}')

alias nb='sudo nano ~/.bashrc'
alias sb='source ~/.bashrc'
alias cm='cd ~/WeGO/catkin_ws && catkin_make'
alias killgazebo='pkill -9 gzserver; pkill -9 gzclient; pkill -9 gzweb; pkill -9 gzbridge'

- colcon build or catkin_make 해서 안되면 source instll/setup.bash 진행하기
- echo $TURTLEBOT3_MODEL 확인가능한돼 안뜨면 - > source ~/.bashrc 진행해서 한번더 확인필요함. 
![image](https://github.com/user-attachments/assets/454db090-5b4c-47a8-9616-286021de1ddc)
![image](https://github.com/user-attachments/assets/ecf787a4-a816-4edf-a6f6-041c44b9e89d)


