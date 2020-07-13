# Multiagent Programming Contest 2017 Team TUBDAI

Technische Universität Berlin - DAI-Labor - http://www.dai-labor.de/

**Team members**

* Christopher-Eky Hrabia: christopher-eyk.hrabia@dai-labor.de
* Patrick Marvin Lehmann: pat@my-lehmann.de
* Nabil Battjbuer: nabil.badjuber@campus.tu-berlin.de
* Axel Hessler: axel.hessler@dai-labor.de

Contest homepage: https://multiagentcontest.org/2017/

## Dependencies

Our team implementation depends on ROS (Robot Operating System) Kinetic and the RHBP (ROS Hybrid Behaviour Planner) framework.
We recommend to setup a Ubuntu 16.04 environment for execution. Debian with the ROS Kinetic should work as well. 

1. `sudo apt install ros-kinetic-desktop`
    
2. `pip install lindypy python-pandas`


## Download and Build

1. Clone the main repo `git clone git@gitlab.tubit.tu-berlin.de:mac17/mac_workspace.git --recursive`
    
2. `cd mac_workspace`
    
3. `catkin_make`
    
4. `source devel/setup.bash`
    
5. `cd ..`
    
6. `mkdir massim`
    
7. `cd massim`
    
7. Download and extract latest massim release (https://github.com/agentcontest/massim/releases) into the directory massim. 
    
8. Adjust the start script massim version numbers in `mac_workspace/script/start_massim.sh` 

## Graphhopper and MASSIM Server

1. Graphhopper is linked as submodule in the mac_workspace repository
     
2. Some OSM files are placed in the data/osm directory of this package, you can update or extend it as explained below

3. Copy all maps (*.pbf) from `massim/massim-2017-1.*/server/osm/` and paste them into tub_contest/data/osm

4. Launching graphhopper server instances is automatically handled by the graphhopper node located in graphhopper.py


## Start of Simulation

To run the MASSIM simulation with our implemented agents you need to follow the following instructions:

1. Start the MASSIM simulator `mac_workspace/script/start_massim.sh` 
    
2. Select a configuration supporting 28 agents

3. Start our agents with e.g. `roslauch tub_contest tub_contest_A.launch` or `roslauch tub_contest tub_contest_TUBDAI.launch` 
(If necessary you can adjust the credentials in the launch files.)
    
4. Go back to the terminal tab where you started the massim simulation and press enter to start the tournament with our agents.


## Configuration


1. If you want to change the names and credentials of our agents, just open the file `tub_contest_TUBDAI.launch` in the directory `mac_workspace/src/tub_contest/launch`.

2. Preloaded maps can be adjusted in graphhopper.launch

3. It is important that the number of agents corresponds with the server configuration. 

    3.1 First the number of agents is configured by the started processes/ROS nodes in `tub_contest_TUBDAI.launch`
    
    3.2 Second the configuration of NUMBER_OF_AGENTS and NUMBER_OF_GROUPS in `src/list_of_numbers.py` has to reflect the number of agents in the simulation
