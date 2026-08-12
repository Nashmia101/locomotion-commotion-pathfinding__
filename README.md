# locomotion-commotion-pathfinding

A solution to the "Locomotion Commotion" pathfinding problem: implements an optimal pathfinding algorithm that models roads and a cyclic train loop, builds a time-expanded graph, and uses Dijkstra's algorithm with a custom MinHeap to compute the minimum-cost interception route.

## Problem Overview

- The city consists of locations (roads and train stations).
- Roads have a cost (fuel/tolls) and a time (minutes to traverse).
- A single train loop connects ≤20 stations, with travel times of 1-5 minutes.
- The driver must intercept the friend at a station without waiting (arrivals must align in time).

**Objective:** Compute the minimum-cost interception path, breaking ties by driving time. If no interception is possible, return `None`.

## Solution Design

**1. Graph Construction**
- Built a directed adjacency list graph with custom `Vertex` and `Edge` classes.
- Each vertex stores outgoing edges, distance, and path metadata.

**2. Time-Expanded Multiverse Graph**
- Expanded the graph into time layers representing the city at each minute of the friend's full loop duration.
- Edges connect across layers to synchronize road travel with the train's cycle, using modulo arithmetic to wrap around once the friend completes a loop.
- Duplicating the graph into `T` time layers allows the driver to revisit the same physical location at different points in time — necessary since an interception may require waiting for the friend to cycle back around.

**3. Shortest Path Algorithm**
- Implemented Dijkstra's algorithm with a custom `MinHeap` supporting `add`, `get_min`, and `update`.
- At each vertex visited, checks whether it matches the friend's location at that exact time layer — if so, records the candidate interception (cost, time, path).
- Explores all possible interception points; returns the one with minimum cost, breaking ties by time, then returns any of the tied paths if cost and time both match.

## Complexity

`intercept` and `dijkstra` run in **O(|R| log |L|)** time and **O(|L| + |R|)** space, where `|R|` is the number of roads and `|L|` is the number of unique locations. The number of time layers `T` is treated as a constant (bounded by the assignment's limits of ≤20 stations and ≤5 minutes per hop, so `T` ≤ 100) and is dropped from the final complexity.

## Results

- Implemented a working interception algorithm tested on multiple city maps.
- Handles edge cases including unsolvable maps, repeated locations, and tie-breaking by time.
- Produces valid outputs consistent with the specification.

## Author

Nashmia Shakeel  
