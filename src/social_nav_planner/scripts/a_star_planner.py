class PathPlanner:
    def __init__(self):
        self.grid = None
        self.start = None 
        self.goal = None

    def initialize_grid(self, width, height):
        """Initialize empty grid"""
        self.grid = [[0 for _ in range(width)] for _ in range(height)]

    def set_start_and_goal(self, start, goal):
        """Set start and goal positions"""
        self.start = start
        self.goal = goal
    
    def find_path(self):
        """Simple A* pathfinding implementation"""
        if not self.grid or not self.start or not self.goal:
            print("Grid, start, or goal not set.")
            return None
            
        # Initialize open and closed sets
        open_set = {self.start}
        closed_set = set()
        
        # Track path and costs
        came_from = {}
        g_score = {self.start: 0}
        f_score = {self.start: self._heuristic(self.start)}
        
        while open_set:
            current = min(open_set, key=lambda pos: f_score.get(pos, float('inf')))
            
            if current == self.goal:
                return self._reconstruct_path(came_from, current)
                
            open_set.remove(current)
            closed_set.add(current)
            
            # Check neighbors
            for dx, dy in [(0,1), (1,0), (0,-1), (-1,0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if (not (0 <= neighbor[0] < len(self.grid)) or
                    not (0 <= neighbor[1] < len(self.grid[0])) or
                    neighbor in closed_set or
                    self.grid[neighbor[0]][neighbor[1]] == 1):
                    continue
                    
                tentative_g = g_score[current] + 1
                
                if neighbor not in open_set:
                    open_set.add(neighbor)
                elif tentative_g >= g_score.get(neighbor, float('inf')):
                    continue
                    
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = g_score[neighbor] + self._heuristic(neighbor)
        
        return None
    
    def _heuristic(self, pos):
        """Manhattan distance heuristic"""
        return abs(pos[0] - self.goal[0]) + abs(pos[1] - self.goal[1])
        
    def _reconstruct_path(self, came_from, current):
        """Reconstruct path from start to goal"""
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

# Example usage
if __name__ == "__main__":
    planner = PathPlanner()
    planner.initialize_grid(10, 10)
    planner.set_start_and_goal((0,0), (9,9))
    path = planner.find_path()
    print(f"Found path: {path}")