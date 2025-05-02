# ICUAS25_finals
This is the repo for the final stage of the ICUAS competition 2025. that is in Charlotte.
It is based  on the one for the [simulation stage](https://github.com/larics/icuas25_competition), so for the basic instructions on Docker installation please check that repo. This one is adapted for flying with real UAVs, therefore, before starting it, please follow the prerequisites instructions:

### Motion Capture tracking - prerequisites: 
To obtain data from motion capture system, please install locally [NatNetSDK](https://github.com/whoenig/NatNetSDKCrossplatform) as given in README instructions.

### USB permissions - prerequisites: 
Copy file: `to_copy/99-bitcraze.rules` to  `/etc/udev/rules.d` directory. Please check these links on USB permissions: [crazyradio](https://www.bitcraze.io/documentation/repository/crazyflie-lib-python/master/installation/usb_permissions/). This enables to use the USB Radio and Crazyflie 2 over USB without being root.

## Run Crazyflies in the Docker container
### Setting up
Clone the [repository]([https://github.com/larics/icuas25_finals]):
```
git clone git@github.com:larics/icuas25_finals.git

```
Add  to  `~/.bashrc` and source it, or type in the current terminal: 
```
export DOCKER_BUILDKIT=1
```
Run Dockerfile from the project root directory using the following commands:
```bash
# Build a Dockerfile
docker build -t icuas_competition_img . 

# Run the crazyswarm2_img container for the first time
./first_run.sh


# This will create docker container icuas_competition container and position you into the container
```
When in docker build the ros2 workspace: 
```
cd ros2_ws
colcon build --symlink-install --merge-install 
```
For future runs, you can use the following commands:
```
# Start the crazyflies_cont:
docker start -i icuas_competition_cont

# Open the crazyflies_cont in another terminal, while it is already started:
docker exec -it icuas_competition_cont bash

# Stop the conatainer
docker stop icuas_competition_cont

# Delete the container
docker rm icuas_competition_cont
```
## Connecting to crazyflies
To connect to crazyflie you must plug in the Crazyradio [flashed USB dongle](https://www.bitcraze.io/documentation/tutorials/getting-started-with-crazyradio-2-0/) into the laptop. 

The general application [cfclient](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/)to connect to crazyflie can be started with (this shouldn't be running while crazyswarm2 packages are launched):

```
cfclient
```
For connecting with ROS2 crazyswarm2 package follow [instructions] (https://imrclab.github.io/crazyswarm2/usage.html)

## Start:

To connect with crazyflies with crazyswarm2 navigate to `/root/ros2_ws/src/icuas25_competition/startup`. and run `./start.sh`. This will start node to connect the camera to the wifi, crazyflie_server, octomap_server and the emergency script.


## INFO part
If you are working in the group and you are all using the same network, please check [ROS_DOMAIN_ID](https://docs.ros.org/en/eloquent/Tutorials/Configuring-ROS2-Environment.html#the-ros-domain-id-variable) in .bashrc in container. Random number should be set during the build, however it is not impossible that some of you got the same number. If that is the situatation please change it, so that your simulations do  not crash.

General information about Crazyflies can be found [here](https://www.bitcraze.io/products/crazyflie-2-1/).

Tutorials on Crazyflies are [here](https://www.bitcraze.io/documentation/start/).

General information about Cfclient can be found [here](https://www.bitcraze.io/documentation/repository/crazyflie-clients-python/master/userguides/userguide_client/).

More info about Crazyswarm2 is described in [here](https://imrclab.github.io/crazyswarm2/).

## Bonus section
The provided Docker image comes with a few preinstalled tools and configs which may simplify your life.

**Tmuxinator** is a tool that allows you to start a tmux session with a complex layout and automatically run commands by configuring a simple yaml configuration file. Tmux is a terminal multiplexer - it can run multiple terminal windows inside a single window. This approach is simpler than having to do `docker exec` every time you need a new terminal. You can move between terminal panes by holding down `Ctrl` key and navigating with arrow keys. Switching between tabs is done with `Shift` and arrow keys. If you have a lot of open panes and tabs in your tmux, you can simply kill everything and exit by pressing `Ctrl+b` and then `k`.

Here are some links: [Tmuxinator](https://github.com/tmuxinator/tmuxinator), [Getting starded with Tmux](https://linuxize.com/post/getting-started-with-tmux/), [Tmux Cheat Sheet](https://tmuxcheatsheet.com/)

**Ranger** is a command-line file browser for Linux. While inside the Docker container, you can run the default file browser `nautilus` with a graphical interface, but it is often easier and quicker to view the files directly in the terminal window. You can start ranger with the command `ra`. Moving up and down the folders is done with arrow keys and you can exit with a `q`. When you exit, the working directory in your terminal will be set to the last directory you opened while in Ranger.

**Htop** is a better version of `top` - command line interface task manager. Start it with the command `htop` and exit with `q`.

**Mcap** is a format to bag data in ROS2, it is added in this dockerfile and you can use it to store data from runs. Check [this link] (https://mcap.dev/guides/getting-started/ros-2).

**VS Code** - If you normally use VS Code as your IDE, you can install [Dev Containers](https://code.visualstudio.com/docs/remote/containers#_sharing-git-credentials-with-your-container) extension which will allow you to continue using it inside the container. Simply start the container in your terminal (`docker start -i mrs_project`) and then attach to it from the VS code (open action tray with `Ctrl+Shift+P` and select `Dev Containers: Attach to Running Container`).

**Foxglove** - this is a data visualization tool. You can download it on your laptop [here](https://foxglove.dev/download) After the download you can start it from the Apps, there will be purple icon. You can visualize data or plot raw messages live, as you are flying (There is already rosbridge_websocker launched.). Checkout this [link](https://docs.foxglove.dev/docs/connecting-to-data/frameworks/ros2#rosbridge). Additionally you can record bags using mcap or any other tool, transfer the bag on your laptop and open it with foxglove. 
