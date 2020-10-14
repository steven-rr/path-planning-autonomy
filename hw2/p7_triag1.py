# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : p7_triag1.py
# //|
# //| Description : Solves a convex tangram puzzle using A*.
# //|               Heuristic involves difference between centers of shapes.
# //|
# //| Notes : Future work can include making a class so main() is less cluttered
# //|
# //|
# //|
# //|
# ////////////////////////////////////////////////////////////////////////////

from math import sqrt


class Triangle1:
    # default constructor for triangle1:
    def __init__(self, list_vertices, tangram_vertices):
        self.square_vertices = [[0, 0], [1, 0], [0, 1], [1, 1]]
        self.tangram_vertices = tangram_vertices
        self.list_vertices = list_vertices
        self.possible_moves = []
        self.max_x = 0
        self.max_x_idx = [0, 0]
        self.max_x_tangram = 0
        self.min_x_tangram = 0

    def get_possible_moves(self):
        self.possible_moves = self.list_vertices[:]
        return self.possible_moves
