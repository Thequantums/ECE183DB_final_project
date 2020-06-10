To run the simulation, go to "worlds" folder and open "RCAP_show.wbt".

The simulation RCAP_show.wbt shows two robots moving on their individual trajectories. It has three predefined trajectories which are chosen randomly when the simulation starts. The predefined trajectories are intended to show the deadlock scenarios and the block path scenarios.

In the simulation, there are three robots running on three different controllers, new_hawk_RCAP.py, new_hippo_RCAP.py, new_hound_RCAP.py. The first robot running on the new_hawk.RCAP.py sends two trajectories to the two other robots at the start of the simulation. After received their trajectories, the two robots stats driving to according to their trajectories using the collision avoidance protocol to protect them from hitting each other.

How does the protocol work?
A: The protocol only allows one of the two robots to move from one node to another node at a single time. If the robot with right to move cannot move because the other robot's current position blocks its way, it will check if deadlock occurs. If deadlock occurs, it will call deadlock resolution protocol. Otherwise, it will pass the right of way to the second robot to move. Both of the two robots running the same protocol. 

There are two constraints on the RCAP:
1. The final goal node of one robot cannot block the path of another robot's trajectory
2. Only one robot is allowed to move at a time
3. Since there is no replanning, there cannot be a deadlock scenario where one of the robot's parent node appears near to the other robot's current node.
