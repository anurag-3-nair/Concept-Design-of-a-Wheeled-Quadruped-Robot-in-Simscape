# **Wheeled Quadruped Robot Concept Design in Simscape&trade;**
This is the repository for the Software Lab project in 2025
Project Goals to be updated!

## **Engineers**
1. Jiang Hongyu
2. Anurag R Nair
3. Nikil Magesh

## **Coaches**
1. Andreas Apostolatos
1. Jan Janse van Rensburg
1. Steve Miller

## **Goals For Project**
1. To develop a design for a Wheeled Quadruped Rescue Robot
2. Should be able to have a payload capacity and be able to carry deliverables in hard to reach areas

## **Robot Visualization**
![](Images/Screenshots/sm_qp08_crawl_4legs_soln_mechExp.png)

## **Path Planning**
While the Pure-Pursuit controller is responsible for tracking a given reference trajectory, it does not generate the trajectory itself. Therefore, a path-planning algorithm is essential for computing an optimal, collision-free path from the starting position to the target within the environment. In this section, an attempt will be made to explain the choice of method for the path planning algorithm, which aids the pure-pursuit algorithm for the robot.

### **Environment Representation and Map Scaling**
The environment is represented using a **two-dimensional occupancy grid**, discretised into uniformly sized square cells. Each square cell represents a region of physical space and is classified as either free or occupied. 

In the implementation, a scaling factor is applied as shown below:

    scale = 3;
    
    % parameters
    MAX_X = round(100*scale)
    MAX_Y = round(100*scale)

This results in a $300 \times 300$ grid, effectively increasing spatial resolution while maintaining the same physical layout. Higher resolution improves the smoothness of the path and the definition of obstacles at the cost of increased computational effort. In the map, obstacle regions are defined analytically using geometric primitives such as rectangles and circles, and subsequently rasterized into the grid by evaluating logical occupancy conditions. Cell semantics used here are as follows:

- **Free Space:** Traversable cells.
- **Obstacles:** Non-traversable cells.
- **Start and goal:** explicitly enforced as free cells.

To ensure collision-free motion, and taking into account the physical footprint of the robot, obstacle inflation is applied by introducing a safety margin around occupied cells. This grid-based representation provides a structured search space that is well-suited for graph-based planners such as A*, enabling efficient computation of collision-free paths for integration with waypoint-following controllers.

### **A-star Algorithm**
Path planning is performed using the A* algorithm, which extends the Dijkstra algorithm by incorporating a heuristic function to guide the search forward and direct it toward the goal. The A* algorithm is employed to compute an optimal collision-free path between the predefined start and goal on the discretized grid map.
