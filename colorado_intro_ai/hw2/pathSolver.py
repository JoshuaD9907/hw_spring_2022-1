from collections import deque
import heapq
import math

class Frontier_PQ:
    ''' frontier class for uniform search, ordered by path cost '''
    
    def __init__(self, start, cost):
        self.states = {}
        self.q = []
        self.add(start, cost)
        
    def add(self, state, cost):
        ''' push the new state and cost to get there onto the heap'''
        heapq.heappush(self.q, (cost, state))
        self.states[state] = cost

    def pop(self):
        (cost, state) = heapq.heappop(self.q)  # get cost of getting to explored state
        self.states.pop(state)    # and remove from frontier
        return (cost, state)

    def replace(self, state, cost):
        ''' found a cheaper route to `state`, replacing old cost with new `cost` '''
        self.states[state] = cost
        for i, (oldcost, oldstate) in enumerate(self.q):
            if oldstate==state and oldcost > cost:
                self.q[i] = (cost, state)
                heapq._siftdown(self.q, 0, i) # now i is posisbly out of order; restore
        return

"""
 Class PathSolver

"""

# Create PathSolver Class
class PathSolver:
    """Contains methods to solve multiple path search algorithms"""

    # init for PathSolver Class
    def __init__(self):
        """Create PathSolver"""

    def path(self, previous, s): 
        """
        `previous` is a dictionary chaining together the predecessor state that led to each state

        `s` will be None for the initial state

        otherwise, start from the last state `s` and recursively trace `previous` back to the initial state,
        constructing a list of states visited as we go
        """ 
        
        if s is None:
            return []
        else:
            return self.path(previous, previous[s])+[s]

    def pathcost(self, path, step_costs):
        """add up the step costs along a path, which is assumed to be a list output from the `path` function above"""
        
        cost = 0
        for s in range(len(path)-1):
            cost += step_costs[path[s]][path[s+1]]
        return cost
    

    def breadth_first_search(self,start: tuple, goal, state_graph, return_cost=False):
        """ find a shortest sequence of states from start to the goal """
        print("calliing BFS")
        
        frontier = deque([start]) # doubly-ended queue of states
        previous = {start: None}  # start has no previous state; other states will
        
        # Return on start is goal
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out

        # loop through frontine searching nodes until we find a goal
        while frontier:
            s = frontier.popleft()
            for s2 in state_graph[s]:
                if (s2 not in previous) and (s2 not in frontier):
                    frontier.append(s2)
                    previous[s2] = s
                    if s2 == goal:
                        path_out = self.path(previous, s2)
                        if return_cost: return path_out, self.pathcost(path_out, state_graph)
                        return path_out
        
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []


    def depth_first_search(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing DFS")
        frontier = [start] # regular Python list works as LIFO queue
        previous = {start: None}  # start has no previous state; other states will
        if start == goal:
            path_out = [start]
            if return_cost: return path_out, self.pathcost(path_out, state_graph)
            return path_out
        while frontier:
            s = frontier.pop()
            for s2 in state_graph[s]:
                if (s2 not in previous) and (s2 not in frontier):
                    frontier.append(s2)
                    previous[s2] = s
                    if s2 == goal:
                        path_out = self.path(previous, s2)
                        if return_cost: return path_out, self.pathcost(path_out, state_graph)
                        return path_out
                    
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []

    def uniform_cost_search(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing UCS")
        frontier = Frontier_PQ(start, 0)
        previous = {start : None}
        explored = {}
        while frontier:
            if frontier.q == []:
                break
            s= frontier.pop()
            if s[1] == goal:
                if return_cost: return self.path(previous, s[1]), s[0]
                return self.path(previous, s[1])
            explored[s[1]] = s[0]
            for s2 in state_graph[s[1]]:
                newcost = explored[s[1]]+state_graph[s[1]][s2]
                if (s2 not in explored) and (s2 not in frontier.states):
                    frontier.add(s2, newcost)
                    previous[s2] = s[1]
                elif (s2 in frontier.states) and (frontier.states[s2] > newcost):
                    frontier.replace(s2, newcost)
                    previous[s2] = s[1]
                    
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []

    def a_star_euclidian(self,start: tuple, goal, state_graph, return_cost=False):
        """Problem 2.b: you need to implement this function"""
        print("calliing A* Euclidean")
        frontier = Frontier_PQ(start, 0)
        previous = {start : None}
        explored = {}
        y_g, x_g = goal
        while frontier:
            if frontier.q == []:
                break
            s= frontier.pop()
            if s[1] == goal:
                if return_cost: return self.path(previous, s[1]), s[0]
                return self.path(previous, s[1])
            y_n, x_n = s[1]
            h = math.sqrt((x_g - x_n)**2 + (y_g - y_n)**2)
            explored[s[1]] = s[0]-h
            explored[start] = 0
            for s2 in state_graph[s[1]]:
                y_n, x_n = s2
                h = math.sqrt((x_g - x_n)**2 + (y_g - y_n)**2)
                newcost = explored[s[1]]+state_graph[s[1]][s2] + h
                if (s2 not in explored) and (s2 not in frontier.states):
                    frontier.add(s2, newcost)
                    previous[s2] = s[1]
                elif (s2 in frontier.states) and (frontier.states[s2] > newcost):
                    frontier.replace(s2, newcost)
                    previous[s2] = s[1]
                    
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []

    def manCalc(self, node, goal):
        y_g, x_g = goal
        y_n, x_n = node
        return (x_g - x_n) + (y_g - y_n)
    
    def a_star_manhattan(self,start: tuple, goal, state_graph, return_cost=False):
        print("calliing A* Manhattan")
        frontier = Frontier_PQ(start, 0)
        previous = {start : None}
        explored = {}
        
        while frontier:
            if frontier.q == []:
                break
            s= frontier.pop()
            if s[1] == goal:
                if return_cost: return self.path(previous, s[1]), s[0]
                return self.path(previous, s[1])
            explored[s[1]] = s[0] - self.manCalc(s[1], goal)
            explored[start] = 0
            for s2 in state_graph[s[1]]:
                newcost = explored[s[1]]+state_graph[s[1]][s2] + self.manCalc(s2, goal)
                if (s2 not in explored) and (s2 not in frontier.states):
                    frontier.add(s2, newcost)
                    previous[s2] = s[1]
                elif (s2 in frontier.states) and (frontier.states[s2] > newcost):
                    frontier.replace(s2, newcost)
                    previous[s2] = s[1]
                    
        # no solution
        if return_cost:
            return [], 0
        else: 
            return []
