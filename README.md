# enpm663_summer2023
Code for ENPM663 (Summer 2023)

# Lecture 9: Task Planning

- `cd ~/enpm663_ws/src`
- `git clone https://github.com/zeidk/enpm663_summer2023.git -b lecture9` 
- `cd ~/ariac_ws`
- `rosdep install --from-paths src -y -i --rosdistro galactic`
- `colcon build`


or 

- `cd ~/enpm663_ws/src`
- `git pull`
- `git checkout lecture9`
- `cd ~/ariac_ws`
- `rosdep install --from-paths src -y -i --rosdistro galactic`
- `colcon build`

# Aliases

```bash
alias start_ariac="ros2 launch ariac_gazebo ariac.launch.py competitor_pkg:=moveit_demo"
alias start_moveit="ros2 launch ariac_moveit_config ariac_robots_moveit.launch.py"
alias start_competition="ros2 service call /ariac/start_competition std_srvs/srv/Trigger"
```
