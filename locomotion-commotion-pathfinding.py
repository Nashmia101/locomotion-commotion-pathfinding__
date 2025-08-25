
# Vertex class represents a vertex in the graph 
class Vertex:
    
    def __init__(self,id):
        
        """
        Function description: 
            This function creates each and every single vertex(location) in a graph. Each vertex (location) 
            has an id, list of edges, distance from the source,
            previous vertex in the path, discovered flag and a visited flag intialised to it
        
        :Input:
            
            id:(int):This uniquely identifies a vertex (location)
             
        :postcondition:
            A vertex (location) is intialised with unique ids and default attributes
            
        :Time complexity:
            O(1)
            
        :Time complexity analysis:
            Time complexity is O(1) because the constructor performs constant operation of assigning values
            
        :Space complexity: O(1)
       
        :Space complexity analysis:
            
        Input Space : O(1) because id holds a single integer
          
        Auxiliary space is constant because no addtional space is used as the number of attributes are fixed
        """
        # Initializes attributes with default values 
        self.id = id
        self.edges = []
        self.distance = float('inf')
        self.previous = None
        self.visited = False 
        self.discovered = False
        
    def __lt__(self,other):
        
        """
        Function description: 
            less than comparsion method that compares the vertices (locations) based on their distance values. 
            if the distances are equal it returns false otherwise returns True helps maintaining
            the heap property
        
        :Input:
            
            other: Vertex (location) (It is the another vertex (location) object that we are comparing with)
            
             
        :return:
           
            returns False if the 2 distances are equal otherwise returns true
            
        :Time complexity:
            O(1)
            
        :Time complexity analysis:
            Time complexity is O(1) because the cost of comparison here is constant 
            
        :Space complexity: O(1)
       
        :Space complexity analysis:
            
          Input Space : O(1) because other holds a single vertex
          
          Auxiliary space is constant because no addtional space is used as only comparsions have been made
     
        """
        # allows comparsion to be done on the basis of distance
        if self.distance < other.distance:
            return True       
        else:
            return False
   
# Edge class represents a directed edge with cost and time
class Edge:
    
    def __init__(self,u,v,cost,time):
        
        """
        Function description: 
            This function intialises an edge (road) between 2 vertices (locations) with cost and time
        
        :Input:
            
            u: Vertex (the source vertex of the edge) (starting location)
            v: Vertex (the destination vertex of the edge) (destination location)
            cost: is the cost associated with going from one vertex (location) to another vertex (location) through this edge 
            time: is the time associated with going from one vertex (location) to another vertex (location) through this edge
            
             
        :postcondition:
           
            an Edge object is created with attributes u,v,cost,time
            
        :Time complexity:
            O(1)
            
        :Time complexity analysis:
            
            Time complexity is O(1) because the constructor performs constant operation of assigning values 
            
        :Space complexity: O(1)
       
        :Space complexity analysis:
            
          Input Space : O(1) because all inputs like u, v, cost, time hold single values
          
          Auxiliary space is constant because no addtional space is used as the number of attributes are fixed
     
        """
        
        self.u = u
        self.v = v
        self.cost = cost
        self.time = time
  
 # Builds a adjacency list graph that represents the road network
class Graph:
    def __init__(self,roads):
        
        """
        Function description: 
            This function creates a graph from a list of roads where each road is a edge between
            two vertices (locations). It takes the vertex (location) from the roads input and creates a vertex object and then stores 
            it in a list and then creates a edge object and connects it between the particular vertices (locations)
        
        :Input:
            
            roads: list of tuples (u,v,cost,time)
                   
              
        :postcondition:
           
            A graph is created where vertices (locations) are connected through edges (roads)
            
        :Time complexity:
            O( L + R)
            where R is the number of edges (Roads) and L is the number of unique vertex IDs (locations)
            
        :Time complexity analysis:
            
            -First we iterate over the roads to find the maximum number of edges with constant O(1) operation
               of comparsion which gives us O(R)
            -Then we make a track_list which takes O(L) 
            -Then we iterate over the roads and append each vertex (location) once in the id_list with 
                constant operations O(1) of assigning. Hence this takes O(L) complexity 
            - Then we loop through the id_list and create Vertex objects this takes 
                O(L) 
            - Then we loop through the roads again and get vertices (locations) through get_vertex function 
                which has O(1) time complexity because it makes use of direct index access which is 
                constant. Then we create a edge and append it to the edges list which takes O(1)
                time complexity. Hence this takes a total of O(R) 
            So the total time complexity is O(R+L+L+L+R) = O(2R+3L) = O(R + L)
            
        :Space complexity: O(R + L) where R is the number of roads (edges) and L is the number of locations (vertices)
       
        :Space complexity analysis:
            Input Space:
                The roads list has a space complexity of O(R) 
            Auxiliary Space:
                - The track_list created to tackle avoiding storing duplicate
                      vertices. This list takes up O(L) auxiliary space  
                - The id_list which stores the unique ids for 
                      the vertices takes up O(L) auxiliary space  
                - vertices is the list that stores the Vertex objects it takes up O(L) space 
                -Edges (roads) stored in the first_vertex.edges list takes up O(R) space
                
        Total Auxiliary Space is O(R + L)
     
        """
        self.vertices = [] 
        self.id_list = []  
        
        # finds the vertex with the maximum id 
        max_vertex = 0
        for i in roads:
            u,v,cost,time = i
            if u > max_vertex:
                max_vertex = u
            if v > max_vertex:
                max_vertex = v
         
        # then creates a boolean list in order to tackle duplicates so that duplicate ids are not appended 
        #in the id_list
        
        track_list = [False] * (max_vertex +1)
        for i in roads:
            u,v,cost,time = i
            if track_list[u] is not True:
                self.id_list.append(u)
                track_list[u] = True
            if track_list[v] is not True:
                self.id_list.append(v)
                track_list[v] = True
               
        # creates Vertex objects according to their unique ids 
        for i in range(len(self.id_list)):
            self.vertices.append(Vertex(i))
        
        # creates Edge objects and connects the edges by appending them in the edges list
        for u,v,cost,time in roads:
            first_vertex = self.get_vertex(u)
            second_vertex = self.get_vertex(v)
            create_edge = Edge(first_vertex,second_vertex,cost,time)
            if first_vertex is not None and second_vertex is not None:
                first_vertex.edges.append(create_edge)
            
    def get_vertex(self,id):
        
        """
        Function description: 
            This function returns the Vertex object at a particular index in the vertices list
        
        :Input:
            
            id: int ( it is the index of the vertex (location) in the vertices list)
            
             
        :return:
           
            returns the appropriate Vertex object
            
        :Time complexity:
            O(1)
            
        :Time complexity analysis:
            Time complexity is O(1) because it access through index is a constant operation 
            
        :Space complexity: O(1)
       
        :Space complexity analysis:
            
         Input Space : O(1) because id holds a single integer
          
         Auxiliary space is constant because no addtional space is used
     
        """
        # returns the vertex at the particular index
        return self.vertices[id]
    
# Min-heap class for the dijkstra algorithim 
class MinHeap:
    def __init__(self, max_size):
        
        """
        Function description: 
            This function intialises MinHeap with a maximum capacity
        
        :Input:
            
            max_size: int( Maximum number of vertices (locations) in a heap)
             
        :postcondition:
            - A heap_array with a size of max_size + 1 is intialised to help with 1-based indexing
            - A map_array with a size of max_size + 1 is intialised to help track the positions of the vertices (locations) in the heap
            
        :Time complexity:
            O(L) where L is the number of vertices (locations)
            
        :Time complexity analysis:
            Time complexity is O(L) because that two arrays of max_size + 1 (L) is created 
            
        :Space complexity: 
            O(L) where L is the number of vertices (locations) 
       
        :Space complexity analysis:
            
         Input Space : O(1) because max_size is a single value integer
          
         Auxiliary Space complexity is O(L) because two arrays of size L are created to store L number of vertices (locations)
     
        """
        # Intializes the heap with maximum size and intializes map_array for index mapping
        
        self.length = 0
        self.heap_array = [None] * (max_size + 1)
        self.map_array = [None] * (max_size + 1)

    def __len__(self):
        
        """
        Function description: 
            This function returns the number of vertices  (locations) in the heap
        
             
        :Return:
           returns the number of vertices (locations) in the heap
            
        :Time complexity:
            O(1) 
            
        :Time complexity analysis:
            Time complexity is O(1) because the function is only returning which is constant time operation
            
        :Space complexity: 
            O(1) 
       
        :Space complexity analysis:
          
         Auxiliary space is constant because no addtional space is used
     
        """
        # returns the current number of vertices in the heap 
        
        return self.length

    def rise(self, n):
        
        """
        Function description: 
            This function rises the vertex (location) at index n up the heap so that the heap property 
            can be maintained 
        
        :Input:
            n: (int) index of the vertex (location) we need to rise
             
        :Postcondition:
          The vertex (location) has to move up the heap and map_array needs to be updated to reflect its new index 
            
        :Time complexity:
            O(log(L)) where L is the number of vertices (location)
            
        :Time complexity analysis:
            In the case of worst case complexity, this particular vertex (location) might be at the leaf node of 
            the heap and its correct position is at the root of the heap 
            which means that it has to go through log L levels in order to rise up to the root position of the heap 
            
        :Space complexity: 
            O(1) 
       
        :Space complexity analysis:
            
        Input Space : O(1) because n is the single value index of the vertex we want to rise hence it 
            is constant
          
        Auxiliary space is constant because no addtional space is used only local variables have been used 
    
     
        """
        # Rises the vertex up the heap to its rightful position in order to restore the heap property
        
        current = self.heap_array[n]
        while n > 1 and current.distance < self.heap_array[n // 2].distance:
            self.heap_array[n] = self.heap_array[n // 2]
            self.map_array[self.heap_array[n].id] = n
            n = n // 2
        self.heap_array[n] = current
        self.map_array[current.id] = n

    def smallest_child(self, n):
        
        """
        Function description: 
            This function finds the smallest child of the node at the index n
            
        :Input: n(int): n is the index of the parent node 
        
             
        :return:
          returns index of the the smallest child  of the parent node
            
        :Time complexity:
            O(1)
            
        :Time complexity analysis:
            Time complexity is O(1) because we are performing constant comparison operation and returning the 
            index
        :Space complexity: 
            O(1) 
       
        :Space complexity analysis:
        
        Input Space : O(1) because n stores the index of the parent node hence it is constant
          
        Auxiliary space is constant because no addtional space is used only local variables have been used 
     
     
        """
        # Returns the index of the smaller child of the parent node at the index n 
        
        if 2 * n == self.length or self.heap_array[2 * n].distance < self.heap_array[2 * n + 1].distance:
            return 2 * n
        else:
            return 2 * n + 1

    def sink(self, n):
        
        """
        Function description: 
            This function moves the vertex (location) at index n down the heap so that the heap property 
            can be maintained 
        
        :Input:
            n: (int) index of the vertex (location) we need to down
             
        :Postcondition:
          The vertex (location) has to move down the heap and map_array needs to be updated to reflect its new index 
            
        :Time complexity:
            O(log(L)) where L is the number of vertices (locations)
            
        :Time complexity analysis:
            In the case of worst case complexity, this particular vertex (location) might be at the root node of 
            the heap and its correct position is at the leaf of the heap 
            which means that it has to go through log L levels in order to sink down to the leaf position of the heap 
            
        :Space complexity: 
            O(1) 
       
        :Space complexity analysis:
            
         Input Space : O(1) because n is the single value index of the vertex we want to sink hence it 
             is constant
          
        Auxiliary space is constant because no addtional space is used only local variables have been used 
     
        """
        # Sinks the vertex down the heap to its rightful position in order to restore the heap property
        
        current = self.heap_array[n]
        while 2 * n <= self.length:
            small_child = self.smallest_child(n)
            if self.heap_array[small_child].distance >= current.distance:
                break
            self.heap_array[n] = self.heap_array[small_child]
            self.map_array[self.heap_array[n].id] = n
            n = small_child
        self.heap_array[n] = current
        self.map_array[current.id] = n

    def add(self, vertex):
        
        """
        Function description: 
            This function adds a vertex  (location) to the heap and then calls rise() to make sure that the heap 
            property is restored after adding the new vertex
        
        :Input:
            vertex: (Vertex) The vertex  (location) we want to add
             
        :Postcondition:
          The vertex  (location) is added to the heap_array and then the map_array is updated and then the rise()
          is called to restore the heap property
            
        :Time complexity:
            O(log(L)) where L is the number of vertices  (locations)
            
        :Time complexity analysis:
           Adding the vertex (location) into the heap is a O(1) operation however calling the rise() in order to restore the
           heap property leads the time complexity to O(log(L))
            
        :Space complexity: 
            O(1) 
       
        :Space complexity analysis:
            
        Input Space : O(1) is constant because it is the vertex object we want to add
          
        Auxiliary space is constant because no addtional space is used only local variables have been used 
     
        """
        # adds the new vertex in the heap and restores the heap property by using rise 
        
        self.length += 1
        self.heap_array[self.length] = vertex
        self.map_array[vertex.id] = self.length 
        self.rise(self.length)

    def get_min(self):
                
        """
        Function description: 
            This function removes and returns the vertex (location) that has the smallest distance value
       
             
        :Return:
          Returns the vertex  (location) with the smallest distance value
            
        :Time complexity:
            O(log(L)) where L is the number of vertices  (locations)
            
        :Time complexity analysis:
           Removing the vertex  (location) from the heap is a O(1) operation however calling the sink() in order to restore the
           heap property leads the time complexity to O(log(L))
            
        :Space complexity: 
            O(1) 
       
        :Space complexity analysis:
          
        Auxiliary space is constant because no addtional space is used only local variables have been used 
     
        """
        # Removes and returns the vertex that has the smallest distance (lowest cost)
        
        minimum = self.heap_array[1]
        self.heap_array[1] = self.heap_array[self.length]
        self.length -= 1
        if self.length > 0:
            self.map_array[self.heap_array[1].id] = 1
            self.sink(1)
        self.map_array[minimum.id] = None
        return minimum

    def update(self, vertex):
                
        """
        Function description: 
            This function updates the position of the vertex (location) in the heap if its distance has been updated
            
        :Input:
            vertex: (Vertex) The vertex (location) we want to update
             
        :Postcondition:
          The vertex (location) is updated IN the heap and then rise() is called to restore the heap property because in 
          dijkstra we only consider updating the distance if it is lower then the previous one so henceforth 
          rise() will be called to position the vertex (location) in its right position
            
        :Time complexity:
            O(log(L)) where L is the number of vertices (locations)
            
        :Time complexity analysis:
           updating the vertex (location) in the the heap is a O(1) operation however calling the rise() in order to restore the
           heap property leads the time complexity to O(log(L))
            
        :Space complexity: 
            O(1) 
       
        :Space complexity analysis:
            
        Input Space : O(1) is constant because it is the vertex object we want to update
          
        Auxiliary space is constant because no addtional space is used only local variables have been used 
     
        """
        # updates the position of the vertex in the heap after a smaller distance is discovered 
        
        current_index = self.map_array[vertex.id]
        if current_index is None:
            return 
        self.rise(current_index)

# main function that stimulates the multiverse graphs and returns the intercept between friend and driver

def intercept(roads, stations, start, friendStart):
        """
        Function description: 
            This function decides whether or not we intercept our friend who is traversing through stations 
            in a loop and if we do intercept then the function returns a tuple that has the minimum cost, total time 
            and the path at which we intercepted the friend
            
         Approach description (if main function):
             
             T is the total time that the friend takes to complete one loop of the stations
             
             The main idea behind this function is to find out the mimimum cost at which I can intercept my friend 
             at a particular station at any time.
             - We can achieve this by getting the total time taken by the friend to complete one whole loop of the 
                 stations since he is going in a cycle 
             - Then we can use this total time to create T duplicates of the original graph where T is the total time 
                 taken by the friend
            - We construct this multiverse by duplicating all vertices for each unit of time in the friend's cycle
                'time_spent' attribute is assigned to let us know which time_layer does this vertex 
                belong to and then we connect edges across graphs according to the travel time associated with each road by 
                using modulo to ensure that time runs cyclically as our friend is also in a cycle
            - The reason why we have created a multiverse of graphs is that it would enable us to visit a 
                location repeatedly in case we can find an interception in which driver has to repeatedely
                visit the same location in order to intercept the friend. Hence this allows us to visit 
                that particular location in different time layers that represent a single unit of time
             - We also keep a track of our friend's location at a particular time 
             - Then we Run dijkstra to find the path with the minimum cost
             - then we return the path along with the minimum cost, total time 
             
        :Input: 
            roads : list of tuples with (u,v,cost,time)
                    These roads are connecting perticular vertices (location) with cost and time taken to reach each vertex (location)
            stations : list of tuples (station, duration)
                        These are the stations through which the friend is traversing and the time 
                        he takes to reach them
            start: (int) This my (Driver) location at which I start my journey to intercept my friend
            
            friendstart: (int) This is the location of the station at which my friend starts his journey 
                                in the repeated cycle
        :Returns:
          Returns a tuple of (minimum cost , time_taken , path) if the friend and driver intercept otherwise it 
          returns None
            
        :Time complexity:
            O(|R| log |L|) 
            
            where R is the number of roads (edges)
            where L is the number of unique vertices (location)
            |R| is the length of the roads list
            |L| is the number of unique vertices (location)
            
        :Time complexity analysis:
            
            T is the total time that the friend takes to complete one loop of the stations
           
            - constructs the intial graph which has O( |R| + |L| ) as this creates |L| Vertex (location) objects 
                which leads to O(|L|) complexity then it creates |R| Edge (roads) objects and then appends them
                to the edges list which gives us O(|R|) Hence the total complexity is O( |R| + |L| )
            - Then we iterate through the stations list in order to find the index of the friendStart 
                which gives us O(|L|) 
            - Then we calculate the total duration in which the friend completes one entire cycle of the stations
                as the friend's traversal is cyclic and we also track where the friend will be at a particular time
                we do this by iterating through the friendjourney list which gives us O(|L|)
            - Then we create the multiverse by duplicating the graphs where we create T time layers where
                each layer represents the graph at a different time unit.Then we create Vertex objects (locations)
                for each layer and then we create edges (roads) across T number
                of graphs which leads us to a total complexity of O( T x |L| + T x |R|)
            - Then we run dijkstra on this multiverse graphs which leads us to the total complexity 
                of O(|R| log |L|)
            - Hence our total complexity is  O(|R| log |L|) + O(|L|) +  O(|L|) + O( |R| + |L| ) + O( T x |L| + T x |R|)
                which simplifies to  O(|R| log |L|) 
                We have removed T because we
                assume T as a constant because T depends on the 
                total time taken by a friend to complete one cycle between the stations so this would remain 
                constant as according to the specification of the assignment the maximum
                number of stations that we can have is 20 and maximum time is 5 mins hence this means that the maximum
                number of graphs that we can have is 100 which is a constant. Hence since the creation of the 
                many graphs depend on T which is constant the total complexity would be the one stated above.
            
                
        :Space complexity: 
            O(|L| + |R|) 
            
            where R is the number of roads (edges)
            where L is the number of unique vertices (location)
            |R| is the length of the roads list
            |L| is the number of unique vertices (location)
            
       
        :Space complexity analysis:
            
            - Input space:
                 roads : takes O(|R|) space
                 stations: takes O(1) Space since number of stations is bounded 
                 Friendstart and Start take O(1) space
                 
            - Auxiliary Space:
                
                T is the total time that the friend takes to complete one loop of the stations
                     
                - constructs the intial graph which has O( |R| + |L|) in auxiliary complexity thus stores |L| Vertex (location) objects 
                    which leads to O(|L|) complexity then it stores |R| Edge (road) objects which gives us O(|R|) 
                    Hence the total complexity is O( |R| + |L| )
                - Then we create friend_journey and friend_approach lists which stores T items 
                    hence the auxiliary space complexity is O(|T|)
                - Stores the multiverse graphs in the multi_verse list which gives us O (T x |L|) auxiliary space.Then
                    stores the edges (roads) as adjacentcy list in the multiverse vertices which gives takes O (T x |R|) auxiliary space
                    hence total auxiliary space complexity is O((T x |L|) + (T x |R|))
                - Then dijkstra runs and map_array and heap_array both take space of O (T x |L|) 
                    
                    
            - Hence our total Auxiliary space complexity is O((|R|) + (|L|)) + O((T x|L|) + (T x |R|)) + O (T x |L|) + O(|T|)
                    which simplifies to  O(|L| + |R|)
                    We have removed T because we
                    assume T as a constant because T depends on the 
                    total time taken by a friend to complete one cycle between the stations so this would remain 
                    constant as according to the specification of the assignment the maximum
                    number of stations that we can have is 20 and maximum time is 5 mins hence this means that the maximum
                    number of graphs that we can have is 100 hence T is bounded thus it is a constant. Hence we wont take T into account
                    for auxiliary space
            
        """
        
        create_graph = Graph(roads) # creates the initial graph
        num_vertices = len(create_graph.vertices)
        friend_journey = []
        friend_approach = []
        multi_graphs = []
        beginning_point = 0
        duration = 0
        
        
        if not roads:
            return None
        
        if not stations:
            return None
        
        # find the index of the friendStart 
        
        for i in range(len(stations)):
            if stations[i][0]== friendStart:
                beginning_point = i
        
        # Helps us correct the stations order. Like now our stations order begins from the station that our friend was at the beginning
        count = 0 
        while count < len(stations):
            friend_index = ((beginning_point + count) % len(stations))
            friend_journey.append(friend_index)
            count = count + 1
            
        # through this we are tracking at what time our friend is at what position. we store this
        # information in friend_approach list and we also calculate the time taken by the friend to 
        # complete one full loop of the stations 
        
        for i in range(len(friend_journey)):
            station, stay = stations[friend_journey[i]]
            friend_approach.append((station,duration))
            duration = duration + stay
        
        # create copies of the graph depending on the duration constant for each time unit
        for d in range(duration):
             dup_graph = ([None] * num_vertices) 
             for j in range(num_vertices):
                 vertex = Vertex(j) # create a new vertex with this id 
                 vertex.time_spent = d  # assign the time layer this vertex belongs to
                 dup_graph[j]= vertex  # add vertex in that particular time layer
             multi_graphs.append(dup_graph)
            
        # adds edges across the graphs 
        
        for d in range(duration):
             for node in create_graph.vertices: # iterate over each vertex in the initial graph
                 current_node = node.id # get ID 
                 for e in node.edges: # iterate over all the outgoing edges of this vertex 
                     destination_node = e.v.id # get the id of destination vertex
                     cost = e.cost
                     time = e.time
                     time_to_next_node = ((d+time) % (duration)) # calculating the time layer the driver should go to after traversing through this edge
                     from_vertex = multi_graphs[d][current_node] # vertex at the current time layer
                     to_vertex = multi_graphs[time_to_next_node][destination_node] # vertex at the destination time layer
                     connect = (Edge(from_vertex,to_vertex,cost,time)) # create a edge between vertices across different time layers
                     from_vertex.edges.append(connect)
        
        # call dijkstra function to get the intercept with the lowest cost 
        return dijkstra(multi_graphs, start, friend_approach, duration, num_vertices)

def dijkstra(multi_graphs, start, friend_approach, duration, num_vertices):
    
    """
    Function description: 
        This function runs the dijkstra algorithim on the multiverse graphs (time layers)
        which stimulate the traversal of driver across the multiverse according to the time.
        Then finds the path with the most minimum cost 
        to intercept the friend
        
    Approach description (part of the main function):
         
         This dijkstra algorithim is used to find the path with the most minimum cost in the 
         multiverse graph. Each vertex in the graph represents a location at a specific time. Our 
         algorithim explores all the possible locations while also tracking cost (distance) and time
         taken to reach that location 
         
         - Intailize all the vertices (locations) at the start of the algorithim to 'inf'. This helps reset the attributes
         - Then we make use of min-heap class to explore the multi verse graphs and extract the vertex (location) with the 
             smallest distance (cost). By using min_heap we ensure that our algorithim always chooses the 
             vertex (location) with the lowest cost
         - For each outgoing edge of the current vertex (location) where we are at currently. We calculate the distance and
             time to the neighbour 
         - If the new cost is less than the neighbour's current distance then update its current distance, time 
             and previous and update it in the heap if the vertex is dicovered otherwise add in the heap
         - At each vertex (location) that the driver visits check whether he can intercept his friend at that particular vertex (location)
             and in that particular time layer and if they intercept store this path and the cost and time
         - After the heap is empty check for the path with the lowest cost and return it and if the cost of path are
             equal break the tie by the minimum time and if the time is also equal return any path. 
             If no interception was found return None
       
    :Input:
        multi_graphs: list of graphs where each graph is a list of Vertex objects representing the graph 
                      at a specific time unit. Each Vertex (location) maintains an adjacency list of outgoing Edge object.
                      
        start: ( int ) The driver’s starting location index.
        
        friend_approach: list of tuples (station_id, time) It tells us which station the friend is at a particular time.
        
        duration: (int) total time taken by the friend to complete the loop.
        
        num_vertices: (int) Number of vertices (locations) in the graph.

    :Returns:
      Returns a tuple of (minimum cost , time_taken and path) if the friend and driver intercept otherwise it 
      returns None
        
    :Time complexity:
        O(|R| log |L|) 
        
        where R is the number of roads (edges)
        where L is the number of unique vertices (locations)
        |R| is the length of the roads list
        |L| is the number of unique vertices (locations)
        
    :Time complexity analysis:
         T is the total time that the friend takes to complete one loop of the stations
       
        - Intializes all multiverse vertices (locations) which takes O(T x |L|)
        - Intializes all the Min_heap and adds the starting vertex (location) which takes O(log(T x |L|)) as the rise() in the add function
            in the min_heap class has O(log(L)) where L is the vertex (location) so for every location in the multiverse
            across the multiverse graphs the complexity would be  O(log(T x |L|))
        - During the dijkstra algorithim each of the vertices of every graph is added into or updated in the heap at most once
              which leads to O(T × |L| × log(T × |L|)) complexity as both add() and update() functions from Min_heap class 
              are called . Then each of the edges across different time layers
              is relaxed at most once
              which leads to O(T × |R| × log(T × |L|)) Hence the total complexity of heap operations is 
              O((T × |L| + T × |R|) log(T × |L|))
        - Then checking for intercept and constructing the path of the intercept takes O(T) because we might 
            intercept the friend at T different units. 
        - Then we loop through all possible interceptions and get the one with lowest cost or break the tie with 
            lowest time which leads to O(T) because we can have T number of interceptions
        - Hence our total complexity 
            is O(T x |L|) + O(log(T x |L|)) + O((T × |L| + T × |R|) log(T × |L|)) + O(T) + O(T)
            this simplifies to O(|L|) + O(log(|L|)) + O((|L| + |R|)(log(|L))).We have removed T because we
            assume T as a constant because T depends on the 
            total time taken by a friend to complete one cycle between the stations so this would remain 
            constant as according to the specification of the assignment the maximum
            number of stations that we can have is 20 and maximum time is 5 mins hence this means that the maximum
            number of graphs that we can have is 100 which is a constant.Hence we would exclude T from our complexity calculation
            Hence it gives us:
                O((|L| + |R|) log |L|). We can also further simplify this by removing |L| 
            from (|L| + |R|) because the number of edges (Roads) is >= the number of vertices (locations)
            hence this would give us  O(|R| log |L|). 
            
    :Space complexity: 
        O(|L|) 
        
        where L is the number of unique vertices (locations)
        |L| is the number of unique vertices  (locations)
   
    :Space complexity analysis:
        T is the total time that the friend takes to complete one loop of the stations
        
        Input Space:
            
            - multi_graphs is the list of the multiverse graphs. Hence Vertex (location) objects takes O(T x |L|)
              and Edge objects are stored in the adjacency lists which leads to O( T x |R|)
            - friend_approach takes O(T) space
            - Duration and num_vertices take O(1) space
            
        Auxiliary Space:
            -dijkstra runs and map_array and heap_array both take space of O (T x |L|) 
            - the friend_intercept list takes up O(T) space because it can have have T number of possible interceptions
            - Then lowest_cost list temporarily stores all the minimun cost interceptions and then through that it
                concludes which interception has the lowest cost and break the tie with time if the costs are the same
                hence storing this takes space complexity of O(T)
                
     - Total Auxiliary Space complexity is O(T x |L|) + O(T) + O(T)
         which simplifies to O(|L|) 
         We have removed T because we
         assume T as a constant because T depends on the 
         total time taken by a friend to complete one cycle between the stations so this would remain 
         constant as according to the specification of the assignment the maximum
         number of stations that we can have is 20 and maximum time is 5 mins hence this means that the maximum
         number of graphs that we can have is 100 hence T is bounded thus it is a constant. Hence we wont take T into account
         for auxiliary space
 
    """
    # resetting the values of the fixed attributes 
    for i in range(duration):
        for j in multi_graphs[i]:
            j.distance = float('inf')
            j.previous = None
            j.visited = False
            j.discovered = False
            j.time_taken = float('inf')
    
    # intializes the starting vertex at time 0
    
    source = multi_graphs[0][start]
    source.distance = 0
    source.time_taken = 0
    
    discovered = MinHeap(duration * num_vertices) # creates Min_heap
    discovered.add(source)
    lowest_cost =[]
    friend_intercept =[]

    while len(discovered) > 0: 
        u = discovered.get_min() # extract vertex with the minimum cost 
        
        # check if the current vertex matches friend's position at the current time layer 
        
        for i in range(len(friend_approach)):
            location, current_time = friend_approach[i]
            if location == u.id and current_time == u.time_spent:
                short_path = []
                destination_node = u
                # constructing the path through .previous 
                
                while destination_node is not None:
                    short_path.append(destination_node.id)
                    destination_node = destination_node.previous
                short_path.reverse() # to fix the order of the path 
                
                friend_intercept.append((u.distance, u.time_taken, short_path))

        u.visited = True # mark as vertex as visited 
        
        # Explore the neighbour vertex through the outgoing edges 
        for edge in u.edges:
                v = edge.v
                updated_cost = u.distance + edge.cost 
                
                # if the updated cost is less than the current distance then update 
                
                if (updated_cost < v.distance):
                    
                    if not v.discovered:
                        v.discovered = True
                        v.distance = u.distance + edge.cost
                        v.time_taken = u.time_taken + edge.time
                        v.previous = u
                        discovered.add(v)
                    else:
                        v.distance = u.distance + edge.cost
                        v.time_taken = u.time_taken + edge.time
                        v.previous = u
                        discovered.update(v)
   # check all the possible interception paths to find the one with the minimum cost
   
    if friend_intercept:
        low_cost = friend_intercept[0][0]
        
        # find the paths with the minimum cost
        for i in friend_intercept:
            if i[0] < low_cost:
                low_cost = i[0]
        
        # append the paths with the lowest cost in the list 
        
        for j in friend_intercept:
            if j[0] == low_cost:
                lowest_cost.append(j)
        

        # if there is only one interception with the minimum cost then return it 
        if len(lowest_cost) == 1:
            return lowest_cost[0]
        
        # else if many interceptions have the same minimum cost break the tie with time 
        
        else:
            low_time = lowest_cost[0]
            for z in lowest_cost:
               if z[1] < low_time[1]:
                   low_time = z
            return low_time   # if many have same minimum cost and time then return any of the paths
                
    else:
        return None # return none if no interception is found
    
