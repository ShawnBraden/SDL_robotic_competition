# 'Arm'y repo
This repo contains the OS of the robotic arm for the Space Dynamics compition. 

## Maintainers:
Shawn Jones -Team lead\
Taryn Jones -Contributer\
Devin Nielsen -Contributer
# Nodes 
## `state_loations`:

    This node recives requests from the ArmyStateService.srv, it then computes the location of the end point and publishes the info on the PoseRequestPub.msg
## `test_service_request`:
    
    This node test the state_location node. It sends a buch of requests and then listens for the response.

# Messages
## `PoseRequestPub.msg`:
    This message contains all the state information for a possotions service request.
### Messge Members:
    1. float64 theta1 : angle of the first joint
    2. float64 theta2 : angle of the second joint
    3. float64 theta3 : angle of the third joint
    4. float64 beta : angle of the base of the robot
    5. int64 id : id of the service request
    6. geometry_msgs/PoseStamped pose_stamped: Built in possition message for ros2, contains all the possition information.

## Services 
## `ArmyStateService.srv`:
    This service creates a request for a possition to be calculated
### Messge Members:
    1. float64 theta1 : requested angle of the first joint
    2. float64 theta2 : requested angle of the second joint
    3. float64 theta3 : requested angle of the third joint
    4. float64 beta : requested angle of the base of the robot
    5. int64 id : id of the service request
    6. bool recived : bool that the server respons with to let the client the service has been recived and added to the queue.

## Compling with pandocs
    To complie .md to a pdf: pandoc -s README.md -V geometry:margin=1in -o README.pdf
    To complie to a stand alone html doc: pandoc  --metadata title="README" -s --self-contained README.md -o README.html

## Linting (Grading your code, and making sure it matches our coding standars)
This is the method that is used to check the code and make sure it fits coding standar and best pratice. The package is called `pylint` and can be installed with 
``` python
    pip install pylint  
```
or 
```python
    pip3 install pylint 
```
depending on context. The command to run `pylint` is:
```python
    python3 -m pylint --jobs 0 --rcfile .pylintrc <name of python file or folder>
```
## Compling with ros
1. Install Ros2.
2. Set up the following Alias.
    ```bash
    alias make_venv='python3 -m venv --system-site-packages venv && touch venv/COLCON_IGNORE && source venv/bin/activate && rosdep install --from-paths src --ignore-src -r -y'
    alias sd='source /opt/ros/humble/setup.bash && source venv/bin/activate && . install/setup.bash'
    alias build='source /opt/ros/humble/setup.bash && source venv/bin/activate && python3 -m colcon build --symlink-install && . install/local_setup.bash'
    ```
3. Run the following command.
    ```bash
    mkdir -p robotic_army/src
    ```
4. Navigate to `robotic_army/src`.
5. install the git repo using.
    ```bash
    git clone git@github.com:ShawnBraden/SDL_robotic_competition.git
    ```
6. return to the `robotic_army` folder.
7. Run 
    ```bash
    make_venv
    ```
8. build the repo.
    ```bash
    build
    ```
9. Run a ros2 node.
    ```ros2
    ros2 run <work space> <node>
    Ex: ros2 run state_space_algo_pub state_sim
    ```
10. Running a ros2 node with args
    ```ros2
    ros2 run <work space> <node> <arg> ... <arg>
    EX: ros2 run state_space_algo_pub state_test_client 90 90 90 270
    ```

