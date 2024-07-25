# MAKARA simulator + GNC modules

## Starting the simulator

It is presumed that you have docker on your desktop and hence the instructions to install docker are omitted here. Before you can begin, we need to build the docker image needed for the simulator to run. The docker image can be build with the following command executed at the root of the repository that you have cloned.

```
$ ./ros2_devdocker.sh
```
Once the build is complete, you can invoke the simulator with the command (again to be run at the root folder of the repository)

```
$ ./ros2_simulator.sh
```
You will see that this starts the simulator and it continuously prints out the states of the vehicle to the terminal. At the start you should see that the propeller and rudder commands must be zero and the vehicle will be at rest. However, you should see the time get incremented. 

## Starting your GNC package

Once the simulator is up and running, open another terminal and navigate to the root folder of the repository. Now execute the following command to start your GNC block to interact with the simulator.

```
$ ./ros2_gnc.sh
```