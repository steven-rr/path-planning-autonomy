# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : p7_square.py
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


# -------------------------------------------------------------------------
#  Class : Square
#  Description: Holds functions for expanding squares.
#               Get Possible moves returns possible moves for a square.
#                   1) One possibility is a non-rotated square.
#                   2) Square rotated by 45 degrees.
#
#  Notes:  square_vertices[1][0] is the length of the square.
# -------------------------------------------------------------------------
class Square:
    # -------------------------------------------------------------------------
    #  Function: init
    #  Description: Constructor
    # -------------------------------------------------------------------------
    def __init__(self, list_vertices, tangram_vertices):
        self.square_vertices = [[0, 0], [1, 0], [0, 1], [1, 1]]
        self.tangram_vertices = tangram_vertices
        self.list_vertices = list_vertices
        self.possible_moves = []
        self.max_x = 0
        self.max_x_idx = [0, 0]
        self.max_x_tangram_static = 0
        self.max_x_tangram_rotated = 0
        self.min_x_tangram = 0

    # -------------------------------------------------------------------------
    #  Function: reset
    #  Description: reinitialize vertices to non-rotated square at origin.
    # -------------------------------------------------------------------------
    def reset(self):
        self.square_vertices = [[0, 0], [1, 0], [0, 1], [1, 1]]
        return 0

    # -------------------------------------------------------------------------
    #  Function: rotate
    #  Description: rotate vertices about center of square.
    # -------------------------------------------------------------------------
    def rotate(self):
        self.square_vertices = [[0, sqrt(2) / 2], [sqrt(2) / 2, 0], [sqrt(2) / 2, sqrt(2)], [sqrt(2), sqrt(2) / 2]]

    # -------------------------------------------------------------------------
    #  Function: get_interaction_triangle
    #  Description: Derives all possible moves when close to a triangle.
    # -------------------------------------------------------------------------
    def get_interaction_triangle(self, max_x, max_x_idx):
        possible_moves = self.possible_moves_in[:]
        # determine if triangle has flat surface or slanted surface:
        counter = 0
        for i in range(self.list_vertices[max_x_idx[0]]):
            if self.list_vertices[max_x_idx[0]][i][0] == max_x:
                counter = counter + 1
        # if triangle face is slanted
        if counter == 1:
            # triangle above the half way line. have square top vertex alignment
            if self.list_vertices[max_x_idx[0]][max_x_idx[1]][1] > sqrt(2) / 2:
                diffx = self.list_vertices[max_x_idx[0]][max_x_idx[1]][0] - self.square_vertices[2][0]
                diffy = self.list_vertices[max_x_idx[0]][max_x_idx[1]][1] - self.square_vertices[2][1]
                for i in range(0, self.square_vertices):
                    self.square_vertices[i][0] = self.square_vertices[i][0] + diffx
                    self.square_vertices[i][1] = self.square_vertices[i][1] + diffy
                    # if out of bounds due to tangram geometry, no more possible moves possible.
                    if self.square_vertices[i][1] < 0 or self.square_vertices[i][1] > sqrt(2):
                        self.reset()
                        return possible_moves

                possible_moves.append(self.square_vertices)
            # triangle below halfway line. have square bottom vertex alignment.
            else:
                diffx = self.list_vertices[max_x_idx[0]][max_x_idx[1]][0] - self.square_vertices[1][0]
                diffy = self.list_vertices[max_x_idx[0]][max_x_idx[1]][1] - self.square_vertices[1][1]
                for i in range(0, self.square_vertices):
                    self.square_vertices[i][0] = self.square_vertices[i][0] + diffx
                    self.square_vertices[i][1] = self.square_vertices[i][1] + diffy
                    # if out of bounds due to tangram geometry, no more possible moves possible.
                    if self.square_vertices[i][1] < 0 or self.square_vertices[i][1] > sqrt(2):
                        self.reset()
                        return possible_moves
                possible_moves.append(self.square_vertices)

        # triangle face is flat:
        else:
            diffx = self.list_vertices[max_x_idx[0]][max_x_idx[1]][0] - self.square_vertices[0][0]
            for i in range(0, self.square_vertices):
                self.square_vertices[i][0] = self.square_vertices[i][0] + diffx
                possible_moves.append(self.square_vertices)

        # update class variable possible moves.
        self.possible_moves = possible_moves
    # -------------------------------------------------------------------------
    #  Function: rotated_square_moves
    #  Description: Derives all possible moves for a rotated square
    # -------------------------------------------------------------------------
    def rotated_square_moves(self):
        # reset and rotate vertices
        self.reset()
        self.rotate()
        possible_moves = self.possible_moves_in[:]
        # derive x limits due to tangram perimeter geometry.
        self.max_x_tangram_rotated = self.tangram_vertices[2][0] - sqrt(2)*self.square_vertices[1][0]
        # check if square can fit inside tangram geometry
        if self.max_x_tangram_rotated > self.max_x > self.min_x_tangram:
            # interaction between rotated square and triangle
            if len(self.list_vertices[self.max_x_idx[0]]) == 3:
                possible_moves = self.get_interaction_triangle(self.max_x, self.max_x_idx)
        elif self.max_x == 0:
            possible_moves.append(self.square_vertices)

    # -------------------------------------------------------------------------
    #  Function: static_square_moves
    #  Description: Derives all possible moves for a non-rotated square
    #               General idea is to put the square as close to shapes,
    #               without exceeding any bounds.
    # -------------------------------------------------------------------------
    def static_square_moves(self):
        possible_moves = self.list_vertices[:]
        # derive x limit due to shape geometry.
        if self.list_vertices != []:
            for i in range(0, self.list_vertices):
                for j in range(0, self.list_vertices[i]):
                    if self.max_x < self.list_vertices[i][j][0]:
                        self.max_x = self.list_vertices[i][j][0]
                        self.max_x_idx = [i, j]
        # derive x limits due to tangram perimeter geometry.
        self.min_x_tangram = self.tangram_vertices[1][0]
        self.max_x_tangram_static = self.tangram_vertices[2][0] - self.square_vertices[1][0]

        # compute possibilities if within limits of tangram geometry.
        if self.max_x_tangram_static > self.max_x > self.min_x_tangram:

            for i in range(0, len(self.square_vertices)):
                self.square_vertices[i][0] = self.square_vertices[i][0] + self.max_x
            possible_moves.append(self.square_vertices)

            for i in range(0, len(self.square_vertices)):
                self.square_vertices[i][1] = self.square_vertices[i][1] + self.square_vertices[1][0] * (sqrt(2) - 1)
            possible_moves.append(self.square_vertices)

        # update class variable possible moves.
        self.possible_moves = possible_moves

    # -------------------------------------------------------------------------
    #  Function: get_possible_moves
    #  Description: Derives all possible moves for a square.
    # -------------------------------------------------------------------------
    def get_possible_moves(self):
        # container to return possible moves
        self.possible_moves = self.list_vertices[:]

        ####### First do all possible moves assuming non-rotated square ######
        self.static_square_moves()

        ####### Second do all possible moves assuming rotated square ######
        self.rotated_square_moves()

        # reset square back to normal vertices.
        self.reset()
        return self.possible_moves
