import heapq
import copy

class State:
    def __init__(self, n, source_peg, peg_2, peg_3, goal_peg, parent,cost):
        self.disks = n
        self.source_peg = source_peg
        self.peg_2 = peg_2
        self.peg_3 = peg_3
        self.goal_peg = goal_peg
        self.parent = parent
        self.cost = cost

    def __lt__(self, other):
        return (self.cost + self.heuristic_distance()) > (other.cost + other.heuristic_distance())

    def heuristic_misplaced(self):
        heuristic = self.disks
        for i in range(1, len(self.goal_peg) + 1):
            if self.goal_peg[-i] == i:
                heuristic -= 1
        return heuristic
            
    def heuristic_distance(self):
        n = self.disks
        distance = 0
        pegs = [self.source_peg, self.peg_2, self.peg_3, self.goal_peg]
        for i in range(len(pegs)):
            for j in range(len(pegs[i])):
                distance = distance + (n-j-1) + (3-i) + (n - pegs[i][j]) 
        return distance
                

def move(state):
    pegs = [state.source_peg, state.peg_2, state.peg_3, state.goal_peg]
    new_pegs = copy.deepcopy(pegs)
    possible_states = []
    for i in range(len(pegs)):
        for j in range(len(pegs)):
            if i != j:
                pegs = copy.deepcopy(new_pegs)
                if len(pegs[i]) == 0:
                    continue
                elif len(pegs[j]) == 0:
                    pegs[j].append(pegs[i].pop())
                    possible_states.append(State(state.disks, pegs[0], pegs[1], pegs[2], pegs[3], state, state.cost + 1))
                elif len(pegs[i]) != 0 and len(pegs[j]) != 0:
                    if pegs[i][-1] < pegs[j][-1]:
                        pegs[j].append(pegs[i].pop())
                        possible_states.append(State(state.disks, pegs[0], pegs[1], pegs[2], pegs[3], state, state.cost + 1))
                        pegs = new_pegs
    return possible_states

def reconstruct_path(goal_state):
    path = []
    current_state = goal_state
    while current_state:
        disk_positions = (current_state.source_peg,current_state.peg_2,current_state.peg_3,current_state.goal_peg)
        path.append(disk_positions)
        current_state = current_state.parent
    path.reverse()
    return path

def a_star(n):
    state = State(n, list(range(n, 0, -1)), [], [], [], None, 0)
    frontier = []
    visited = []
    extended_states = list()
    heapq.heappush(frontier, (state.cost + state.heuristic_distance(), state))
    while frontier:
        priority, state = heapq.heappop(frontier)
        current_state = [state.source_peg, state.peg_2, state.peg_3, state.goal_peg]
        if current_state in visited:
            continue
        else:
            visited.append(current_state)
        if state.goal_peg == list(range(n, 0, -1)):
            cost = state.cost
            path = reconstruct_path(state)
            print('Goal achieved!!',len(visited),'states visited')
            print('cost is', state.cost)
            break
        else:
            extended_states = move(state)
            for i in range(len(extended_states)):
                heapq.heappush(frontier, ((extended_states[i].cost) + extended_states[i].heuristic_distance(), extended_states[i]))
            extended_states = []
    return path
    
path = a_star(3) 
print(*path,sep='\n')
