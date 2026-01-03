# **Wheeled Quadruped Robot Concept Design in Simscape&trade;**
This is the repository for the Software Lab project in 2025!

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

## **Objectives of the Robot**
1. Autonomous Operation
2. Environmental Sensing
3. Payload Delivery
4. Surveillance
5. Mobility

## **Robot Visualization**
![](Images/Screenshots/sm_qp08_crawl_4legs_soln_mechExp.png)
![](C:\Software_Lab_Temp\Software_Lab_2025\Final_Robot_image.png)

## **Path Planning**
While the Pure-Pursuit controller is responsible for tracking a given reference 
trajectory, it does not generate the trajectory itself. Therefore, a path-planning 
algorithm is essential for computing an optimal, collision-free path from the 
starting position to the target within the environment. In this section, an attempt 
will be made to explain the choice of method for the path planning algorithm, 
which aids the pure-pursuit algorithm for the robot.

### **Environment Representation and Map Scaling**
The environment is represented using a **two-dimensional occupancy grid**, 
discretised into uniformly sized square cells. Each square cell represents a 
region of physical space and is classified as either free or occupied.\ 

In the implementation, a scaling factor is applied as shown below:

    scale = 3;
    
    % parameters
    MAX_X = round(100*scale)
    MAX_Y = round(100*scale)

This results in a $300 \times 300$ grid, effectively increasing spatial resolution
 while maintaining the same physical layout. Higher resolution improves the 
smoothness of the path and the definition of obstacles at the cost of increased 
computational effort. In the map, obstacle regions are defined analytically using 
geometric primitives such as rectangles and circles, and subsequently rasterized 
into the grid by evaluating logical occupancy conditions. Cell semantics 
used here are as follows:

- **Free Space:** Traversable cells.
- **Obstacles:** Non-traversable cells.
- **Start and goal:** explicitly enforced as free cells.

To ensure collision-free motion, and taking into account the physical footprint 
of the robot, obstacle inflation is applied by introducing a safety margin 
around occupied cells. This grid-based representation provides a structured 
search space that is well-suited for graph-based planners such as A*, enabling 
efficient computation of collision-free paths for integration with 
waypoint-following controllers.

### **A-star Algorithm**
Path planning is performed using the A* algorithm, which extends the Dijkstra 
algorithm by incorporating a heuristic function to guide the search forward 
and direct it toward the goal. The A* algorithm is employed to compute an 
optimal collision-free path between the predefined start and goal on the 
discretized grid map.

#### **Initialisation and Map Setup**
The algorithm initializes the grid map with predefined dimensions 
(|MAX_X, MAX_Y|) and assigns semantic values to each cell. Traversable cells 
are initialised with positive cost values, while obstacles are marked as 
non-traversable with negative cost values. The obstacles are generated randomly 
or deterministically using analytical rectangle and circle definitions and 
rasterized into a binary occupancy grid.\

Parameter declaration for the map dimensions:

    % parameters
    MAX_X = round(100*scale)
    MAX_Y = round(100*scale)
    MAP = 2*(ones(MAX_X, MAX_Y));

Grid representation assigns semantic meaning to each cell:

    % map values
    MAP(map_inflated) = -1;     % for obstacles
    MAP(xStart, yStart) = 1;    % for start
    MAP(xTarget, yTarget) = 0;  % for target   

To ensure collision-free motion, considering the robot's physical size, 
obstacle inflation is applied using morphological dilation (*|imdilate|*) 
with a disk-shaped structuring element corresponding to the robot radius. 
The start and target positions are explicitly enforced as free cells to 
guarantee algorithmic validity. \

Obstacle inflation here is forced to respect robot's physical footprint:

    % obstacles
    se = strel('disk', robot_radius);
    map_inflated = imdilate(BW, se);

#### **A-Star Data Structure Setup**

Two primary data structures are used: the **OPEN list** and the **CLOSED list**.

- The **CLOSED list** is pre-populated with all obstacle cells, 
    preventing their expansion during the search.
- The **OPEN list** initially only contains the start node, 
    for which the accumulated cost $g(n)$, heuristic cost $h(n)$, and 
    total cost $f(n)$ are computed.

Preloading obstacles into the closed list prevents them from ever being expanded:

    % adding obstacles in the closed list
    k=1; % dummy counter
    for i = 1:MAX_X
        for j = 1:MAX_Y
            if(MAP(i,j) == -1)
                CLOSED(k, 1) = i; 
                CLOSED(k, 2) = j; 
                k = k+1;
            end
        end
    end

The heuristic function is defined as the Euclidean distance to the goal:
    $h(n) = \sqrt{(x_n - x_{goal})^2 + (y_n - y_{goal})^2}$

This setup is illustrated in the \textbf{A* Data Structure Setup} block of 
the flow diagram and implemented via the **insert_open** function 
in the code.

### **Main A-Star Search Loop**

The algorithm iteratively expands nodes until the goal is reached or no valid
path to the goal exists. At each iteration:

- The current node is expanded using an 8-connected neighbourhood model, 
  allowing diagonal motion.
- Successor nodes are evaluated and filtered against the **CLOSED** list.
- If a successor already exists in the **OPEN list**, its cost is 
  updated only if a lower-cost path is found.
- New valid successors are inserted into the **OPEN list**.

The node with the minimum total cost, $f(n)$, is then selected using the
**min_fn** function and moved from the **OPEN** list to the **CLOSED** list. 
This process is selected until the target node is selected or the **OPEN** list
is completely exhausted.\

A node's parent is updated only if a lower-cost path is found:

    if (exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3))
        OPEN(j,8) = min(OPEN(j,8), exp_array(i,5));
        % updating the parent nodes if better path found
        if OPEN(j,8) == exp_array(i,5)
            OPEN(j,4) = xNode;
            OPEN(j,5) = yNode;
            OPEN(j,6) = exp_array(i,3);
            OPEN(j,7) = exp_array(i,4);
        end
        flag = 1;
        break;
    end

### **Path Termination and Reconstruction**

When the goal node is reached, the algorithm reconstructs the optimal path 
by backtracking through the stored parent nodes. This process generates an 
ordered sequence of grid coordinates, starting from the start and ending at 
the target.\

Storing parent pointers during expansion enabling efficient backtracking:

    % determining the parent nodes
    parent_x = OPEN(node_index(OPEN,xval,yval),4); % row index
    parent_y = OPEN(node_index(OPEN,xval,yval),5); % col index
   
    while (parent_x ~= xStart || parent_y ~= yStart)
        Optimal_path(i,1) = parent_x;
        Optimal_path(i,2) = parent_y;

        inode    = node_index(OPEN, parent_x, parent_y);
        parent_x = OPEN(inode,4);
        parent_y = OPEN(inode,5);
        i = i + 1;
    end

A validation step ensures that no reconstructed path point intersects an 
obstacle cell. The final path is then visualized over the grid path and 
converted into waypoints suitable for downstream use by the Pure Pursuit controller.\

An extra step of safety ensured through this validation step to avoid 
conflicts with regions with obstacles:

    for idx = 1:j
        if MAP(Optimal_path(idx,1), Optimal_path(idx,2)) == -1
            fprintf('WARNING: Path goes through/over obstacle at (%d, %d)!\n', Optimal_path(idx,1), Optimal_path(idx,2));
            path_valid = false;
        end
    end

![Path Planning](C:\Software_Lab_Temp\Software_Lab_2025\path_planning.gif)