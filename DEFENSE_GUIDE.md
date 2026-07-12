# COMPLETE DEFENSE GUIDE - RMPC Assignment 2

## TABLE OF CONTENTS
1. [Overall Architecture](#1-overall-architecture)
2. [A* Algorithm (lattice_planner.py)](#2-a-algorithm---lattice_plannerpy)
3. [RRT Planner (rrt_planner.py)](#3-rrt-planner---rrt_plannerpy)
4. [PRM Planner (prm_planner.py)](#4-prm-planner---prm_plannerpy)
5. [Trajectory Generation (traj_generation.py)](#5-trajectory-generation---traj_generationpy)
6. [How Everything Connects (main.py pipeline)](#6-how-everything-connects---mainpy-pipeline)
7. [Potential Tough Questions & Answers](#7-potential-tough-questions--answers)
8. [Quick-Fire Q&A Cheat Sheet](#8-quick-fire-qa-cheat-sheet)

---

## 1. OVERALL ARCHITECTURE

The project implements **three path planning algorithms** (A* on a lattice, RRT, PRM) and a **trajectory generator** that converts a discrete path into a smooth, time-parameterized trajectory with velocity and acceleration profiles.

### File Map
```
main.py                          -- Orchestrator: builds map, runs planners, plots
path_planner/
  utils.py                       -- Graph data structure + ObstaclesGrid collision checker
  lattice_planner.py             -- Lattice graph construction + A* solver
  rrt_planner.py                 -- Rapidly-exploring Random Tree
  prm_planner.py                 -- Probabilistic Roadmap
trajectory_generator/
  traj_generation.py             -- Path resampling, velocity profiling, interpolation
```

### Data Flow
```
main.py creates a 10x10 lattice grid (100x100 map in cells)
  --> LatticeGraph builds vertices (row, col, angle) with 4 orientations
  --> Obstacles are placed on the boolean grid
  --> Edges crossing obstacles are set to infinity
  --> A* finds the shortest path on the lattice
  --> path_interpolation() densifies the path (straight lines + arcs)
  --> resample_path() creates a full trajectory with (x, y, theta, v, a, omega)
  --> matplotlib plots the result
```

### Key Concepts
- **Lattice graph**: Each vertex is a (row, col, angle) tuple. Angles are 0, 90, 180, 270 degrees. This means the robot has a heading at each node.
- **Straight edges** have weight 1 (one cell length). **Arc edges** have weight pi (quarter-circle arc length).
- **Obstacles** are represented as a boolean numpy array. An edge is invalid if any sampled point along it falls inside an obstacle cell.

---

## 2. A* ALGORITHM - lattice_planner.py

### What you implemented (4 sections):

### 2.1 solve_astar() -- Lines 209-238

```python
distances[s] = 0
costs[s] = self.calH(s, g)

while not open_set.empty():
    cost, curr = open_set.get()

    if curr in closed_set:
        continue
    closed_set.add(curr)

    if curr == g:
        path = self.traverse_path(s, g, parent_node)
        return path

    neighbors = self.get_neighbor(curr, graph_vert_list, adjacency_matrix)
    for n in neighbors:
        if n in closed_set:
            continue

        g_cost = distances[curr] + self.cal_expand_cost(curr, n, edge_dict)

        if n not in distances or g_cost < distances[n]:
            distances[n] = g_cost
            f = g_cost + self.calH(n, g)
            costs[n] = f
            parent_node[n] = curr
            open_set.put((f, n))

print("no path found")
return None
```

#### LINE-BY-LINE EXPLANATION:

1. **`distances[s] = 0`** -- The g-cost (cost from start) of the start node is 0.
2. **`costs[s] = self.calH(s, g)`** -- The f-cost of start = g(s) + h(s) = 0 + heuristic.
3. **`while not open_set.empty()`** -- Main loop: keep exploring while there are nodes in the priority queue.
4. **`cost, curr = open_set.get()`** -- Pop the node with the LOWEST f-cost (PriorityQueue is a min-heap).
5. **`if curr in closed_set: continue`** -- Skip if already expanded (lazy deletion pattern). This is necessary because we may push the same node multiple times with different f-values.
6. **`closed_set.add(curr)`** -- Mark as expanded/visited.
7. **`if curr == g`** -- Goal check AFTER popping (not when adding). This guarantees optimality because the first time we pop the goal, it has the minimum f-cost.
8. **`neighbors = self.get_neighbor(...)`** -- Get adjacent vertices from adjacency matrix. The pre-built `get_neighbor` checks which entries in the adjacency matrix row are finite and positive.
9. **`if n in closed_set: continue`** -- Don't re-expand already visited nodes.
10. **`g_cost = distances[curr] + self.cal_expand_cost(curr, n, edge_dict)`** -- Tentative g-cost through current node.
11. **`if n not in distances or g_cost < distances[n]`** -- Only update if this is a NEW node or we found a CHEAPER path.
12. **`distances[n] = g_cost`** -- Update best-known g-cost.
13. **`f = g_cost + self.calH(n, g)`** -- Compute f = g + h.
14. **`parent_node[n] = curr`** -- Record how we reached n (for path reconstruction).
15. **`open_set.put((f, n))`** -- Push with f-cost as priority. May create duplicates (lazy deletion handles this).

#### WHY LAZY DELETION?
Python's PriorityQueue doesn't support decrease-key. Instead of updating a node's priority, we push a new entry. When we pop, we check if it's already in closed_set and skip it. This is O(E log V) in practice and correct.

#### WHY CHECK GOAL AFTER POPPING?
If you check when adding to the open set, you might find the goal through a suboptimal path first. By checking after popping, you guarantee the goal's g-cost is minimal because the priority queue orders by f-cost, and with an admissible heuristic, the first pop of the goal is optimal.

### 2.2 traverse_path() -- Lines 255-262

```python
path = []
curr = g
while curr != s:
    path.append(curr)
    curr = parent_node[curr]
path.append(s)
path.reverse()
return path
```

#### EXPLANATION:
- Start from goal, follow parent pointers back to start.
- This gives the path in reverse order (goal -> start), so we reverse it.
- Standard backtracking technique used in all graph search algorithms.

#### POTENTIAL QUESTION: "Why not build the path forward from start?"
Because during search, we only know each node's parent (predecessor), not its children (successors). The parent_node dictionary maps child -> parent, so we must trace backwards.

### 2.3 cal_expand_cost() -- Line 291

```python
return edge_dict[(v1, v2)]
```

#### EXPLANATION:
- Simply looks up the edge weight from the edge dictionary.
- For straight edges this is 1 (one cell unit).
- For arc edges this is pi (quarter circle = pi*r/2 with r=1 lattice cell, but the code uses pi as the arc cost).
- Edges through obstacles have been set to np.inf by `update_obstacles()`, so they will never be chosen as optimal.

### 2.4 calH() -- Lines 307-309

```python
dx = v1[0] - v2[0]
dy = v1[1] - v2[1]
return np.sqrt(dx**2 + dy**2)
```

#### EXPLANATION:
- **Euclidean distance** heuristic between current vertex and goal.
- Uses only the (row, col) components (indices 0 and 1), ignoring the angle.
- This is **admissible** because the straight-line distance is always <= the actual path cost (you can't get there faster than a straight line).
- This is also **consistent** (monotone): h(n) <= cost(n, n') + h(n') for any neighbor n'. This guarantees A* finds the optimal path without reopening nodes.

#### POTENTIAL QUESTION: "Why not use Manhattan distance?"
Euclidean is tighter (closer to real cost) because the lattice allows diagonal movement via arcs. Manhattan distance would overestimate diagonal paths. You could use Manhattan but it would be less informed, leading to more node expansions.

#### POTENTIAL QUESTION: "Why ignore the angle in the heuristic?"
The heuristic only needs to be a lower bound. Including angle differences would make it more informed but also more complex. The Euclidean distance on (row, col) is already admissible and works well.

#### POTENTIAL QUESTION: "Is your heuristic admissible? Prove it."
The minimum cost to travel one lattice cell horizontally/vertically is 1 (straight edge weight). The Euclidean distance between two grid positions (r1,c1) and (r2,c2) is sqrt((r1-r2)^2 + (c1-c2)^2). Since any path must traverse at least this distance (and usually more due to arcs costing pi > 1), the heuristic never overestimates. Therefore it is admissible.

---

## 3. RRT PLANNER - rrt_planner.py

### What you implemented (6 methods):

### 3.1 sample_random_point() -- Lines 68-70

```python
rx = np.random.randint(0, self.map_size[0])
ry = np.random.randint(0, self.map_size[1])
return Node(rx, ry)
```

#### EXPLANATION:
- Uniform random sampling over the entire map.
- `np.random.randint(0, N)` gives integers in [0, N-1].
- Returns a Node object (not just coordinates) because the tree stores Node objects.

#### POTENTIAL QUESTION: "Why not add goal biasing?"
Goal biasing (e.g., sample the goal 5-10% of the time) would speed up convergence toward the goal. The basic version without bias still works but may take more iterations. I kept it simple as the assignment didn't require it, and with 500 max iterations and step_size=5 on a 100x100 map, it converges well.

#### POTENTIAL QUESTION: "Why sample integers and not floats?"
The obstacle map is a discrete boolean grid indexed by integers. Using integers ensures clean collision checks against `self.obstacles.map[px, py]`. Floats would need rounding anyway.

### 3.2 find_nearest_node() -- Lines 82-89

```python
best = None
best_d = float('inf')
for n in self.tree:
    d = np.sqrt((n.x - rand_node.x)**2 + (n.y - rand_node.y)**2)
    if d < best_d:
        best_d = d
        best = n
return best
```

#### EXPLANATION:
- Linear scan through all tree nodes, computing Euclidean distance to the random sample.
- Keeps track of the closest node found so far.
- O(n) per query where n = number of tree nodes.

#### POTENTIAL QUESTION: "This is O(n). How would you improve it?"
Use a **KD-Tree** (like scipy.spatial.KDTree) for O(log n) nearest-neighbor queries. For the PRM planner I actually used KDTree. For RRT with 500 max iterations the linear scan is fine -- premature optimization wasn't needed.

### 3.3 steer() -- Lines 102-109

```python
dx = rand_node.x - nearest_node.x
dy = rand_node.y - nearest_node.y
d = np.sqrt(dx**2 + dy**2)
if d <= self.step_size:
    return Node(rand_node.x, rand_node.y, nearest_node)
nx = nearest_node.x + dx / d * self.step_size
ny = nearest_node.y + dy / d * self.step_size
return Node(nx, ny, nearest_node)
```

#### EXPLANATION:
- Computes the direction vector from nearest to random node.
- If the random node is within step_size distance, use it directly.
- Otherwise, move exactly step_size units in that direction (normalize the vector and scale).
- `dx/d` and `dy/d` form the unit direction vector.
- The new node's parent is set to nearest_node (third argument to Node constructor).

#### POTENTIAL QUESTION: "Why limit the step size?"
Without a step size limit, the tree could jump across obstacles. The step size ensures incremental growth, and collision checking between nearest and new node is reliable over short distances. It also controls tree density.

#### POTENTIAL QUESTION: "What if d is 0 (random point equals nearest node)?"
If d=0, then d <= step_size is true, so we return the random node directly with nearest_node as parent. No division by zero occurs.

### 3.4 is_colliding() -- Lines 122-132

```python
dist = np.sqrt((new_node.x - nearest_node.x)**2 + (new_node.y - nearest_node.y)**2)
n_steps = max(int(dist), 1)
for i in range(n_steps + 1):
    t = i / n_steps
    px = int(nearest_node.x + t * (new_node.x - nearest_node.x))
    py = int(nearest_node.y + t * (new_node.y - nearest_node.y))
    if px < 0 or px >= self.map_size[0] or py < 0 or py >= self.map_size[1]:
        return True
    if self.obstacles.map[px, py]:
        return True
return False
```

#### EXPLANATION:
- **Line-segment collision check** by sampling points along the segment.
- `n_steps = max(int(dist), 1)` ensures at least 1 step (avoids division by zero) and roughly one sample per pixel of distance.
- `t` ranges from 0 to 1, linearly interpolating between nearest and new node.
- Each sample point is converted to integer grid coordinates.
- Bounds checking first (out of map = collision), then obstacle grid check.
- If ANY point is in collision, return True (collision found). If all pass, return False.

#### POTENTIAL QUESTION: "Is this collision check complete? Could it miss obstacles?"
With n_steps = int(dist), we sample roughly one point per unit distance. Since obstacles occupy grid cells (1x1 units), this is usually sufficient. For a perfectly rigorous check, you could use Bresenham's line algorithm to enumerate all grid cells the line passes through. But for this step_size (5) the sampling density is adequate.

### 3.5 reached_goal() -- Lines 144-145

```python
d = np.sqrt((new_node.x - self.goal.x)**2 + (new_node.y - self.goal.y)**2)
return d <= self.step_size
```

#### EXPLANATION:
- Checks if the new node is within step_size distance of the goal.
- We use step_size as the threshold because that's the maximum distance we can cover in one step -- if we're within one step of the goal, we can reach it.

#### POTENTIAL QUESTION: "Why not check exact equality?"
With floating-point arithmetic and continuous space, hitting the exact goal coordinates is essentially impossible. A proximity threshold is the standard approach in sampling-based planners.

### 3.6 construct_path() -- Lines 157-163

```python
path = []
node = end_node
while node is not None:
    path.append((node.x, node.y))
    node = node.parent
path.reverse()
return path
```

#### EXPLANATION:
- Backtracks from the end node to the root using parent pointers.
- The root node (start) has `parent = None`, which terminates the loop.
- Returns a list of (x, y) tuples, reversed to go from start to goal.
- Same backtracking concept as A*'s traverse_path, but using Node objects with parent attributes instead of a dictionary.

---

## 4. PRM PLANNER - prm_planner.py

### What you implemented (5 methods):

### 4.1 construct_roadmap() -- Lines 49-69

```python
self.roadmap.append(self.start)
self.roadmap.append(self.goal)

for _ in range(self.num_samples):
    p = self.sample_free_point()
    if p is not None:
        self.roadmap.append(p)

# connect each node to k nearest neighbors
for node in self.roadmap:
    nearest = self.find_k_nearest(node, self.k_neighbors)
    for nb in nearest:
        if not self.is_colliding(node, nb):
            if node not in self.edges:
                self.edges[node] = []
            if nb not in self.edges:
                self.edges[nb] = []
            if nb not in self.edges[node]:
                self.edges[node].append(nb)
            if node not in self.edges[nb]:
                self.edges[nb].append(node)
```

#### EXPLANATION:
- **Phase 1 - Sampling**: Add start and goal first (they MUST be in the roadmap). Then sample `num_samples` (200) random collision-free points.
- **Phase 2 - Connection**: For each node, find its k nearest neighbors and try to connect with collision-free edges.
- **Bidirectional edges**: If A connects to B, then B also connects to A (undirected graph). The duplicate checks (`if nb not in self.edges[node]`) prevent storing the same edge twice.
- **Edge storage**: `self.edges` is a dictionary mapping Node -> list of connected Nodes (adjacency list representation).

#### POTENTIAL QUESTION: "Why add start and goal first?"
If start/goal aren't in the roadmap, no path can be found. They need to be connected to nearby samples. By adding them first, they participate in the neighbor-finding and edge-building process.

#### POTENTIAL QUESTION: "Why check duplicates in edges?"
Because if node A finds B as a neighbor and adds A->B, then when processing B, it might find A as a neighbor and try to add B->A again. The checks prevent duplicate entries in the adjacency lists.

### 4.2 sample_free_point() -- Lines 78-83

```python
for _ in range(100):
    x = np.random.randint(0, self.map_size[0])
    y = np.random.randint(0, self.map_size[1])
    if not self.obstacles.map[x, y]:
        return Node(x, y)
return None
```

#### EXPLANATION:
- Rejection sampling: generate random points and reject those in obstacles.
- Up to 100 attempts to find a collision-free point.
- Returns None if no free point found after 100 tries (highly unlikely unless the map is mostly obstacles).
- The caller (`construct_roadmap`) handles None by simply not adding it.

#### POTENTIAL QUESTION: "Why 100 attempts?"
With the given obstacle configuration (roughly 20-30% of the map is obstacles), the probability of failing to find a free point in 100 tries is astronomically small (~0.3^100). 100 is a safe upper bound that avoids infinite loops.

### 4.3 find_k_nearest() -- Lines 96-107

```python
pts = np.array([[n.x, n.y] for n in self.roadmap])
tree = KDTree(pts)
k_actual = min(k + 1, len(self.roadmap))
_, idxs = tree.query([node.x, node.y], k=k_actual)

result = []
for i in idxs:
    nb = self.roadmap[i]
    if nb.x == node.x and nb.y == node.y:
        continue
    result.append(nb)
return result[:k]
```

#### EXPLANATION:
- Builds a KDTree from all roadmap node coordinates for efficient nearest-neighbor search.
- Queries for `k+1` neighbors because the query point itself is in the tree (distance 0) and we want to exclude it.
- Filters out the node itself by coordinate comparison.
- Returns exactly k neighbors (or fewer if the roadmap is small).

#### POTENTIAL QUESTION: "You rebuild the KDTree every call. Isn't that wasteful?"
Yes, building the KDTree is O(n log n). Ideally you'd build it once after all samples are collected. But since construct_roadmap calls this n times, it's O(n^2 log n) total. For 200 samples this is negligible. If performance mattered, I'd build the tree once and pass it in.

#### POTENTIAL QUESTION: "Why use KDTree instead of brute force like in RRT?"
PRM connects EVERY node to its k neighbors, so find_k_nearest is called 200+ times. KDTree makes each query O(log n) instead of O(n). For RRT, find_nearest is called at most 500 times on a growing tree, and the simpler brute-force approach was sufficient.

### 4.4 is_colliding() -- Lines 120-130

```python
d = np.sqrt((node1.x - node2.x)**2 + (node1.y - node2.y)**2)
steps = int(d / self.step_size) + 1
for i in range(steps + 1):
    t = i / max(steps, 1)
    x = int(node1.x + t * (node2.x - node1.x))
    y = int(node1.y + t * (node2.y - node1.y))
    if x < 0 or x >= self.map_size[0] or y < 0 or y >= self.map_size[1]:
        return True
    if self.obstacles.map[x, y]:
        return True
return False
```

#### EXPLANATION:
- Same concept as RRT's collision check: sample points along the line segment.
- `steps = int(d / self.step_size) + 1` -- sample one point per step_size distance (5 units), plus endpoints.
- `max(steps, 1)` prevents division by zero when nodes are at the same position.
- Checks bounds and obstacle grid at each sample.

#### POTENTIAL QUESTION: "How is this different from RRT's is_colliding?"
Same principle, slightly different step calculation. RRT uses `n_steps = max(int(dist), 1)` (one sample per unit), PRM uses `steps = int(d / step_size) + 1` (one sample per step_size). Both are valid; PRM's is coarser but faster since PRM edges can be longer.

### 4.5 plan() -- Lines 139-189

```python
from queue import PriorityQueue

dist = {}
prev = {}
visited = set()
pq = PriorityQueue()

sk = (self.start.x, self.start.y)
gk = (self.goal.x, self.goal.y)

dist[sk] = 0
pq.put((0, sk))

# lookup table
lookup = {}
for n in self.roadmap:
    lookup[(n.x, n.y)] = n

while not pq.empty():
    c, curr = pq.get()
    if curr in visited:
        continue
    visited.add(curr)

    if curr == gk:
        path = []
        k = gk
        while k in prev:
            path.append(k)
            k = prev[k]
        path.append(sk)
        path.reverse()
        return path

    node = lookup.get(curr)
    if node is None or node not in self.edges:
        continue

    for nb in self.edges[node]:
        nbk = (nb.x, nb.y)
        if nbk in visited:
            continue
        w = np.sqrt((node.x - nb.x)**2 + (node.y - nb.y)**2)
        new_c = dist[curr] + w
        if nbk not in dist or new_c < dist[nbk]:
            dist[nbk] = new_c
            prev[nbk] = curr
            pq.put((new_c, nbk))

print("PRM: no path found")
return None
```

#### EXPLANATION:
This is **Dijkstra's algorithm** (A* with h=0) on the roadmap graph.

- **lookup table**: Maps (x,y) tuples to Node objects. Needed because PriorityQueue can't handle Node objects directly (not comparable), so we use (x,y) tuples as keys.
- **sk, gk**: Start key and goal key as tuples.
- **dist**: Shortest known distance to each node.
- **prev**: Previous node in shortest path (for backtracking).
- **visited**: Already-expanded nodes (closed set).
- The main loop is identical in structure to A* but without a heuristic (pure shortest path by edge weight = Euclidean distance).

#### POTENTIAL QUESTION: "Why Dijkstra instead of A*?"
Both work. Dijkstra is simpler and guaranteed optimal. A* would be faster with a good heuristic, but for 200 nodes Dijkstra is instant. I chose simplicity.

#### POTENTIAL QUESTION: "Why use tuples as dictionary keys instead of Node objects?"
Node objects aren't hashable by default (well, they are by id, but two different Node objects at the same coordinates wouldn't match). Using (x,y) tuples ensures correct dictionary lookups based on position, not object identity.

#### POTENTIAL QUESTION: "What if start or goal has no edges?"
The `if node is None or node not in self.edges: continue` guard handles this. If start/goal are isolated (no collision-free connections), the queue empties and we print "PRM: no path found".

---

## 5. TRAJECTORY GENERATION - traj_generation.py

### What you implemented (9 sections):

### 5.1 Gear and Station Calculation -- Lines 30-47

```python
for i in range(1, len(path)):
    d = self.distance(path[i], path[i-1])
    stations[i] = stations[i-1] + d
    # figure out if we are going forward or backwards
    d_row = path[i][0] - path[i-1][0]
    d_col = path[i][1] - path[i-1][1]
    avg_th = (path[i-1][2] + path[i][2]) / 2.0
    h_row = -math.sin(math.radians(avg_th))
    h_col = math.cos(math.radians(avg_th))
    dot = d_row * h_row + d_col * h_col
    if dot >= 0:
        gears[i] = 1
    else:
        gears[i] = -1
if len(gears) > 1:
    gears[0] = gears[1]
else:
    gears[0] = 1
```

#### EXPLANATION:

**Stations (cumulative distance):**
- `stations[i]` = total distance traveled from path[0] to path[i].
- Computed incrementally: stations[i] = stations[i-1] + distance(path[i], path[i-1]).

**Gears (forward/backward determination):**
- The displacement vector between consecutive points is `(d_row, d_col)`.
- The robot's heading direction is computed from the average angle of the two points: `avg_th = (theta_prev + theta_curr) / 2`.
- The heading unit vector in grid coordinates:
  - `h_row = -sin(avg_th)` (negative because row increases downward in grid coordinates)
  - `h_col = cos(avg_th)` (column increases rightward, matching standard math convention)
- The **dot product** of displacement and heading tells us if the robot is moving forward (positive dot) or backward (negative dot).
- `gears[0]` is set to match `gears[1]` since there's no "previous" point for the first element.

#### POTENTIAL QUESTION: "Why -sin for the row component?"
In the grid coordinate system, **row increases downward** (like image coordinates). In standard math, y increases upward. So heading angle 90 degrees (pointing "up" in math) means row DECREASES. The heading in row direction is `-sin(theta)`. For column (which increases rightward like x), it's `cos(theta)`.

#### POTENTIAL QUESTION: "Why average the angles?"
At the transition between two orientations (e.g., in an arc), the actual heading is somewhere between the two endpoints' angles. Averaging gives a reasonable approximation of the robot's heading along that segment.

### 5.2 Velocity and Angular Velocity -- Lines 86-95

```python
for i in range(1, nfe):
    dx = result.states[i].x - result.states[i-1].x
    dy = result.states[i].y - result.states[i-1].y
    result.states[i].v = math.sqrt(dx**2 + dy**2) / dt
    dth = result.states[i].theta - result.states[i-1].theta
    result.states[i].omega = dth / dt
# copy first element from second
if nfe > 1:
    result.states[0].v = result.states[1].v
    result.states[0].omega = result.states[1].omega
```

#### EXPLANATION:
- **Linear velocity**: v = displacement / dt = sqrt(dx^2 + dy^2) / dt. This is the magnitude of the velocity vector (speed).
- **Angular velocity**: omega = d(theta) / dt. Simple finite difference of heading angle over time step.
- **First element**: We can't compute a backward difference for index 0 (no index -1), so we copy from index 1. This is a standard boundary condition for numerical differentiation.

#### POTENTIAL QUESTION: "Why not use a centered difference?"
A centered difference `(x[i+1] - x[i-1]) / (2*dt)` would be more accurate (second-order vs first-order), but it would require special handling at both boundaries. For a first implementation, forward difference is simpler and sufficient.

### 5.3 Acceleration -- Lines 99-102

```python
for i in range(1, nfe):
    result.states[i].a = (result.states[i].v - result.states[i-1].v) / dt
if nfe > 1:
    result.states[0].a = result.states[1].a
```

#### EXPLANATION:
- **Linear acceleration**: a = dv/dt. Finite difference of velocity.
- Same boundary treatment: copy from index 1 for index 0.
- Note: angular acceleration is not computed (not needed for the assignment output).

### 5.4 Acceleration Phase (Forward Pass) -- Lines 124-131

```python
fwd = [0.0] * len(stations)
for i in range(1, len(stations)):
    ds = stations[i] - stations[i-1]
    if ds < 1e-9:
        fwd[i] = fwd[i-1]
    else:
        v2 = fwd[i-1]**2 + 2 * max_accel * ds
        fwd[i] = min(math.sqrt(max(v2, 0)), max_velocity)
```

#### EXPLANATION:
This computes the **maximum velocity achievable at each station if the robot accelerates from rest**.

Uses the kinematic equation: **v_f^2 = v_i^2 + 2*a*ds**
- `v2 = fwd[i-1]**2 + 2 * max_accel * ds` -- velocity squared at station i assuming max acceleration from station i-1.
- `fwd[i] = min(sqrt(v2), max_velocity)` -- clamp to max velocity.
- `max(v2, 0)` prevents negative values under sqrt (numerical safety).
- `if ds < 1e-9` handles zero-distance segments (duplicate stations).

This is the **forward velocity envelope**: "how fast CAN we be going at each point if we accelerate as hard as possible from the start?"

### 5.5 Deceleration Phase (Backward Pass) -- Lines 136-143

```python
bwd = [0.0] * len(stations)
for i in range(len(stations) - 2, -1, -1):
    ds = stations[i+1] - stations[i]
    if ds < 1e-9:
        bwd[i] = bwd[i+1]
    else:
        v2 = bwd[i+1]**2 + 2 * max_accel * ds
        bwd[i] = min(math.sqrt(max(v2, 0)), max_velocity)
```

#### EXPLANATION:
Same kinematic equation but **backwards from the end** (where we must stop, so bwd[-1] = 0).

This is the **backward velocity envelope**: "how fast CAN we be going at each point if we need to decelerate to zero by the end?"

The loop goes from the second-to-last station backward to the first. At each point, it computes the maximum velocity that still allows deceleration to the next station's velocity.

### 5.6 Constant Velocity Phase (Merge) -- Lines 147-154

```python
for i in range(len(stations)):
    profile[i] = min(fwd[i], bwd[i])
# avoid zero at the endpoints so we dont divide by zero later
if len(profile) > 1:
    if profile[0] < 1e-6:
        profile[0] = profile[1] * 0.5
    if profile[-1] < 1e-6:
        profile[-1] = profile[-2] * 0.5
```

#### EXPLANATION:
- **`min(fwd[i], bwd[i])`**: The actual velocity at each station is the MINIMUM of what the forward and backward envelopes allow. This creates the classic **trapezoidal velocity profile**:
  - Accelerate (limited by forward envelope)
  - Cruise at max velocity (both envelopes are at max_velocity)
  - Decelerate (limited by backward envelope)
- **Endpoint fix**: At the very start and end, both envelopes give 0 velocity (start from rest, end at rest). But later, `time_profile[i] = ds / profile[i]` would divide by zero. Setting small nonzero values prevents this. Using half the adjacent velocity is a smooth approximation.

#### POTENTIAL QUESTION: "Why is the velocity profile trapezoidal?"
The forward pass gives a velocity curve that rises from 0 (accelerating). The backward pass gives a velocity curve that rises from the end backwards (decelerating when viewed forward). Taking the minimum of both creates: ramp up -> flat top (if distance is long enough) -> ramp down. This is the time-optimal profile under constant acceleration/deceleration constraints.

#### POTENTIAL QUESTION: "What if the path is too short to reach max velocity?"
Then the forward and backward envelopes intersect before reaching max_velocity, creating a **triangular** profile (accelerate then immediately decelerate, no cruise phase). The `min()` handles this automatically.

### 5.7 normalize_angle() -- Lines 229-233

```python
while angle > math.pi:
    angle -= 2 * math.pi
while angle < -math.pi:
    angle += 2 * math.pi
return angle
```

#### EXPLANATION:
- Wraps any angle into the range [-pi, pi].
- Subtracts or adds 2*pi until the angle is in range.
- Used in `to_continuous_angle()` to compute the shortest angular difference between consecutive headings.

#### POTENTIAL QUESTION: "Why not use modulo?"
`angle % (2*pi)` gives [0, 2*pi), not [-pi, pi]. You could use `((angle + pi) % (2*pi)) - pi`, but the while-loop approach is clearer and for typical angles (near 0) it executes 0-1 iterations.

### 5.8 distance() -- Line 246

```python
return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
```

- Standard 2D Euclidean distance. Uses indices [0] and [1] (x, y or row, col).

### 5.9 to_continuous_angle() -- Lines 258-269

```python
# the angles come in degrees from the path so convert to radians first
rads = []
for a in angles:
    rads.append(math.radians(a))
if len(rads) == 0:
    return rads
out = [rads[0]]
for i in range(1, len(rads)):
    diff = rads[i] - rads[i-1]
    diff = self.normalize_angle(diff)
    out.append(out[-1] + diff)
return out
```

#### EXPLANATION:
- **Problem**: Angles like [350, 10] degrees have a raw difference of -340, but the actual angular change is +20 degrees. Interpolating raw angles would produce wild swings.
- **Solution**: Convert to **continuous** (unwrapped) representation where angles accumulate smoothly.
- **Step 1**: Convert degrees to radians (the path uses degrees because the lattice angles are 0, 90, 180, 270 degrees).
- **Step 2**: For each consecutive pair, compute the angular difference and normalize it to [-pi, pi] (shortest rotation).
- **Step 3**: Accumulate: `out[i] = out[i-1] + normalized_diff`. This can produce values outside [-pi, pi] (e.g., 3*pi for multiple full turns), but that's correct for interpolation purposes.

#### POTENTIAL QUESTION: "Why convert from degrees to radians?"
The lattice path uses degree angles (0, 90, 180, 270) because that's how `generate_lattice` defines vertices. But for computing velocities, accelerations, and angular velocities, we need radians (the standard unit for angular calculations in physics/robotics).

#### POTENTIAL QUESTION: "What does 'continuous' mean here?"
It means no sudden jumps. For example, a path going from heading 350 deg to 10 deg should be represented as a smooth +20 degree change, not a -340 degree change. The unwrapped sequence might look like [6.10, 6.28, 6.45, ...] radians instead of [6.10, 0.17, 0.35, ...].

---

## 6. HOW EVERYTHING CONNECTS - main.py pipeline

### Step-by-step execution flow:

1. **Graph creation**: `LatticeGraph()` creates a 10x10 grid with 4 orientations per cell = 400 vertices. Edges connect adjacent cells with weight 1 (straight) or pi (arc).

2. **Obstacle setup**: 5 rectangular regions are marked True in a 100x100 boolean array.

3. **Edge invalidation**: `update_obstacles()` iterates all edges, samples points along each edge (lines or arcs), and sets edges crossing obstacles to infinity.

4. **A* pathfinding**: `graph.solve(s_3d, g_3d, ...)` runs A* on the lattice. Returns a list of (row, col, angle) tuples.

5. **RRT pathfinding**: Creates an RRT planner with the same start/goal (2D), runs `plan()`. Returns (x, y) tuples. Note: RRT operates in continuous 2D space, not on the lattice.

6. **PRM pathfinding**: Creates a PRM planner, builds roadmap with 200 samples, runs `plan()`. Returns (x, y) tuples.

7. **Path interpolation** (only for lattice path): `path_interpolation()` densifies the lattice path by sampling points along straight segments and arcs. Converts from lattice coordinates to map coordinates (divided by cell_size=3 for scaling).

8. **Trajectory resampling**: `resample_path()` takes the interpolated path and produces:
   - Gear determination (forward/backward)
   - Cumulative distance (stations)
   - Optimal velocity profile (trapezoidal)
   - Time parameterization
   - Uniform time-step interpolation
   - Velocity, angular velocity, acceleration computation

9. **Visualization**: Plots the velocity profile and the path on the obstacle map.

---

## 7. POTENTIAL TOUGH QUESTIONS & ANSWERS

### A* Questions

**Q: "What is the time complexity of your A* implementation?"**
A: O(V log V + E log V) where V = number of vertices and E = number of edges. The PriorityQueue operations (put/get) are O(log n). In the worst case we process all V vertices, and for each we examine its neighbors (total E edges across all vertices). With lazy deletion, we might push up to E entries into the queue, so it's O(E log E) = O(E log V) since E <= V^2.

**Q: "What happens if there's no path?"**
A: The open_set empties (all reachable nodes are explored without finding the goal). The while loop exits, we print "no path found" and return None. main.py checks `if path:` before proceeding to trajectory generation.

**Q: "What is the difference between g-cost, h-cost, and f-cost?"**
A:
- g-cost (`distances[n]`): Actual cost of the best known path from start to node n.
- h-cost (`calH(n, g)`): Estimated cost from node n to goal (heuristic, must be admissible).
- f-cost = g + h: Estimated total cost of the best path through n. A* always expands the node with the smallest f-cost.

**Q: "Could you use a different heuristic?"**
A: Yes. Manhattan distance (|dx| + |dy|) would work but would be less tight since diagonal moves (arcs) are possible. Chebyshev distance (max(|dx|, |dy|)) would also work. Any admissible heuristic guarantees optimality. The zero heuristic degenerates A* to Dijkstra's algorithm.

**Q: "What if the heuristic is not admissible?"**
A: A* may return a suboptimal path. It would still find A path (if one exists), but not necessarily the shortest one. The heuristic must never overestimate the true cost for A* to guarantee optimality.

### RRT Questions

**Q: "Is RRT guaranteed to find the optimal path?"**
A: No. RRT finds A path (probabilistically complete -- guaranteed to find one if one exists, given enough iterations), but it's generally not the shortest path. For optimal paths, you need RRT* which rewires the tree to reduce costs. Standard RRT prioritizes speed of finding any valid path.

**Q: "What is probabilistic completeness?"**
A: As the number of iterations approaches infinity, the probability of finding a path (if one exists) approaches 1. It's not deterministic -- any single run might fail, but given enough time it will succeed.

**Q: "How does step_size affect RRT performance?"**
A:
- Too small: slow expansion, many iterations needed to traverse the map.
- Too large: might jump over narrow passages, collision checks become less reliable.
- step_size=5 on a 100x100 map is a good balance (covers map in ~20 steps, fine enough to navigate around obstacles).

**Q: "Why doesn't your RRT connect the final node directly to the goal?"**
A: When `reached_goal()` returns True, `construct_path()` backtracks from the new_node (which is within step_size of the goal, not exactly at the goal). For a more precise implementation, you could add the actual goal node as a child of new_node. However, being within step_size of the goal is sufficient for most practical applications.

### PRM Questions

**Q: "What is the difference between PRM and RRT?"**
A:
- **PRM is multi-query**: Build the roadmap once, query it for different start/goal pairs. **RRT is single-query**: builds a new tree for each query.
- **PRM has two phases**: construction (sampling + connection) and query (graph search). RRT combines exploration and pathfinding in one phase.
- **PRM is better** when you need to solve many queries in the same environment. **RRT is better** for single queries or changing environments.

**Q: "Why use Dijkstra instead of A* for PRM's plan()?"**
A: Both would work. Dijkstra is simpler (no heuristic function needed). With only ~200 nodes in the roadmap, the performance difference is negligible. A* with Euclidean heuristic would expand fewer nodes but the total runtime difference is microseconds.

**Q: "What happens if the roadmap is disconnected (start and goal in different components)?"**
A: Dijkstra will exhaust all nodes reachable from start without finding the goal. The while loop empties, and we return None with "PRM: no path found". Solution: increase num_samples or k_neighbors.

**Q: "How do num_samples and k_neighbors affect PRM?"**
A:
- More samples -> denser roadmap -> higher chance of connectivity -> slower construction.
- More neighbors -> more edges -> higher chance of connectivity -> slower construction.
- With 200 samples and k=10, the roadmap is usually well-connected for this map configuration.

### Trajectory Generation Questions

**Q: "Explain the trapezoidal velocity profile."**
A: The robot starts at rest, accelerates at max_acceleration until it reaches max_velocity (or needs to start decelerating), cruises at max_velocity, then decelerates at max_acceleration to stop at the goal. This is time-optimal under constant acceleration constraints. The forward pass computes the "accelerating from rest" envelope, the backward pass computes the "decelerating to rest" envelope, and taking the minimum gives the feasible profile.

**Q: "What is the kinematic equation you used?"**
A: v_f^2 = v_i^2 + 2*a*s, where v_f = final velocity, v_i = initial velocity, a = acceleration, s = distance. This comes from eliminating time from the equations v_f = v_i + a*t and s = v_i*t + 0.5*a*t^2.

**Q: "Why do you compute the time profile from the velocity profile?"**
A: time = distance / velocity. For each segment: dt = ds / v. The total time at each station is the cumulative sum of these dt values. This converts from the spatial domain (distance along path) to the temporal domain (time), which is needed for uniform time-step interpolation.

**Q: "What is the purpose of path_interpolation?"**
A: The lattice path is coarse (only lattice vertices). path_interpolation samples many points along each edge (10 points per straight segment, ~15 per arc), creating a smooth, dense path suitable for trajectory generation. Without this, the trajectory would have only a few waypoints.

**Q: "Why do you need to_continuous_angle?"**
A: Without it, interpolating between heading 350deg and 10deg would go through 180, 90, 0 instead of smoothly crossing through 360/0. The continuous representation ensures the interpolator sees a +20 degree change, not a -340 degree change. This prevents the robot from spinning wildly during heading transitions.

---

## 8. QUICK-FIRE Q&A CHEAT SHEET

| Question | Short Answer |
|----------|-------------|
| What data structure does A* use? | Priority queue (min-heap) for open set, set for closed set, dict for distances and parents |
| What makes A* different from Dijkstra? | A* uses a heuristic h(n) to guide search toward the goal. f = g + h |
| What does "admissible" mean? | The heuristic never overestimates the true cost to the goal |
| What does "consistent" mean? | h(n) <= cost(n,n') + h(n') for all neighbors. Implies admissible |
| Is Euclidean distance admissible? | Yes, straight line is always <= any path |
| What is the open set? | Nodes discovered but not yet expanded, ordered by f-cost |
| What is the closed set? | Nodes already expanded (visited) |
| Why use lazy deletion? | Python PriorityQueue lacks decrease-key; push duplicates and skip stale ones |
| What is RRT? | Rapidly-exploring Random Tree. Incrementally builds a tree toward random samples |
| Is RRT optimal? | No, it's probabilistically complete but not optimal. RRT* is asymptotically optimal |
| What is PRM? | Probabilistic Roadmap. Samples free space, connects neighbors, searches graph |
| PRM phases? | Construction (sample + connect) and Query (graph search) |
| When is PRM better than RRT? | Multiple queries in the same environment |
| What is a trapezoidal velocity profile? | Accelerate -> cruise -> decelerate. Time-optimal under constant accel constraints |
| v_f^2 = v_i^2 + 2as comes from? | Kinematics: combining v = v0 + at and s = v0*t + 0.5*a*t^2 to eliminate time |
| Why forward AND backward pass? | Forward = max speed if accelerating from start. Backward = max speed if decelerating to stop. min = feasible |
| What is angular velocity? | Rate of change of heading angle: omega = d(theta)/dt |
| Why convert degrees to radians? | Physics/math formulas use radians. sin, cos, angular velocity all need radians |
| What does normalize_angle do? | Wraps angle to [-pi, pi] to find shortest rotation direction |
| What does to_continuous_angle do? | Unwraps angle sequence so interpolation doesn't create jumps at 0/360 boundary |
| What is the gear? | Forward (1) or reverse (-1) motion direction, determined by dot product of displacement and heading |
| What is a station? | Cumulative arc-length distance along the path from the starting point |
| What is the lattice cell size? | 10 units. Each grid cell spans 10x10 in the obstacle map coordinates |
| How many vertices in the lattice? | 10 rows * 10 cols * 4 angles = 400 vertices |
| Weight of straight edge? | 1 (one cell unit) |
| Weight of arc edge? | pi (quarter circle circumference for unit radius) |
| How are obstacles handled? | Edges through obstacles get weight = infinity, so A* never uses them |
| What does the adjacency matrix store? | Edge weights. 0 = no edge, positive = edge weight, inf = blocked edge |

---

## BONUS: UNDERSTANDING THE PRE-BUILT CODE

### Graph class (utils.py)
- `_vert_list`: List of all vertex tuples (row, col, angle)
- `_edge_dict`: Maps (v1, v2) tuples to edge weights
- `_adjacency_matrix`: NxN numpy array where entry [i,j] = weight of edge from vertex j to vertex i (note the transpose convention)

### ObstaclesGrid class (utils.py)
- `map`: Boolean numpy array (100x100). True = obstacle.
- `is_edge_valid()`: Samples points along an edge (line or arc) and checks if any point is in an obstacle cell.
- `get_pts_from_line()`: Linear interpolation between two grid vertices.
- `get_pts_from_arc()`: Uses precomputed arc primitives to sample points along curved paths.

### generate_lattice() (lattice_planner.py)
- Creates 400 vertices (10x10x4 orientations).
- Straight edges: angle 0 -> move right (col+1), angle 90 -> move up (row-1), angle 180 -> move left (col-1), angle 270 -> move down (row+1). Weight = 1.
- Arc edges: turns of 90 degrees. E.g., from angle 90 at (r,c) to angle 0 at (r-1,c+1) -- a right turn while moving forward. Weight = pi.
- Arc primitives: precomputed (x,y) sample points along quarter-circle arcs, used for collision checking and path interpolation.

### get_neighbor() (lattice_planner.py)
- Finds the row in the adjacency matrix corresponding to vertex u.
- Returns all vertices v where adjacency_matrix[row_u, col_v] is finite and positive (i.e., there's a valid edge).

### path_interpolation() (traj_generation.py)
- For each consecutive pair of path vertices:
  - If same angle: straight line, sample `lattice_cell_size` points.
  - If different angle: arc, use precomputed arc primitive points.
- Coordinates are divided by `cell_size=3` for scaling to visualization coordinates.

### interpolate_1d() (traj_generation.py)
- Linear interpolation. Given known (x,y) pairs and target x values, computes interpolated y values.
- Uses `np.searchsorted` for efficient binary search of the correct interval.
- Handles edge cases: t beyond range returns endpoint values; zero-width intervals return left value.
