# Task

Create a ROS package that will implement a simple State Machine for a turtlebot and create a service that will be used to switch the bot from one state to another.

## Subtask 1

Write a python script that contains a class that implements the three states of a robot (Move forwards/backwards, Rotate Clockwise/Anticlockwise, Stop) using three functions for the three states and write a publisher into it that makes the turtlebot move by publishing the velocity commands to /cmd_vel . The class should contain a member variable that holds the state the bot is currently in. For now, you can hardcode the state of the bot.

## Subtask 2

Create a ROS Service that you can use to switch the bot to the three different states in the following format

```code
string state // state to switch to
int32 dir // dir=1 means bot moves forward, dir=-1 means bot moves backwards
------------------
bool success // Indicates if the state change occurred successfully
```

## Subtask 3

Run the turtlbot3_world using

```bash
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

Try to use the services you have created earlier to move the bot from it’s start point to the blue goal in the image above. Create a rosbag file and upload it along with the package.

## Bonus task

Create another state where the bot moves in an eight/infinity symbol path. The direction variable will signify the two opposite eights it can form.
