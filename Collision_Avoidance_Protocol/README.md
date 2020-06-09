To run the simulation, go to "worlds" folder and open "RCAP_show.wbt".

The simulation RCAP_show.wbt shows two robots moving on their individual trajectories. It has three predefined trajectories which are chosen
randomly when the simulation starts. The predefined trajectories are intended to show the deadlock scenario and the block path scenario.

There are three constraints on the RCAP:
1. The final goal node of one robot cannot block the path of another robot's trajectory
2. Only one robot is allowed to move at a time
3. The next node in the trajectory do not appear near its previous node. This is to ensure the two robots do not get stuck in the deadlock.
Based on observation, our RRT does not generate this kind of node. So, it's not a problem if the protocol takes input trajectory from the
RRT.
