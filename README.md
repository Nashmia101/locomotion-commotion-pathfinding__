# locomotion-commotion-pathfinding__
“Solution for the problem Locomotion Commotion, implementing an optimal pathfinding algorithm that models roads and a cyclic train loop, builds a time-expanded graph, and uses Dijkstra’s algorithm with a custom MinHeap to compute the minimum-cost interception route

---

## Problem Overview
- The city consists of **locations** (roads and train stations).  
- Roads have a **cost** (fuel/tolls) and a **time** (minutes to traverse).  
- A single **train loop** connects ≤20 stations, with travel times of 1–5 minutes.  
- The driver must intercept the friend at a station **without waiting** (arrivals must align in time).  

**Objective:** Compute the **minimum-cost interception path**, breaking ties by driving time.  
If no interception is possible, return `None`.  

---

## Solution Design
1. **Graph Construction**  
   - Built a directed adjacency list graph with custom `Vertex` and `Edge` classes.  
   - Each vertex stores outgoing edges, distance, and path metadata.  

2. **Time-Expanded Multiverse Graph**  
   - Expanded the graph into **time layers** representing the city at each minute.  
   - Edges connect across layers to synchronize road travel with the train’s cycle.  

3. **Shortest Path Algorithm**  
   - Implemented **Dijkstra’s algorithm** with a custom `MinHeap` supporting `add`, `get_min`, and `update`.  
   - Explores all possible interception points and records cost, time, and path.  
   - Returns the optimal path with **minimum cost**, breaking ties by **time**.  

---

## Results
- Implemented a working **interception algorithm** tested on multiple city maps.  
- Handles edge cases including **unsolvable maps**, **repeated locations**, and **tie-breaking by time**.  
- Produces valid outputs consistent with specification.  

---

## Author
**Nashmia Shakeel**  
