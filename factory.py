from algorithms import RightHandRule, DFSSolver, FloodFillSolver


class MazeSolverFactory:
    @staticmethod
    def get_solver(algorithm_type):
        solvers = {
            "right_hand": RightHandRule(),
            "dfs": DFSSolver(),
            "flood_fill": FloodFillSolver()
        }
        return solvers.get(algorithm_type, None)