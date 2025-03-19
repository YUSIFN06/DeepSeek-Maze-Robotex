class MazeSolver:
    def solve(self, robot):
        raise NotImplementedError("This method should override by other methods!")

class RightHandRule(MazeSolver):
    def solve(self, robot):
        print("RightHandRule is workinggg...")

class DFSSolver(MazeSolver):
    def solve(self, robot):
        print("DFSSolver is workinggg...")

class FloodFillSolver(MazeSolver):
    def solve(self, robot):
        print("Flood-Fill is workingg...")
