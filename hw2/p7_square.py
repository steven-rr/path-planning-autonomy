
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
    # default constructor for square
    def __init__(self, list_vertices, tangram_vertices):
        self.square_vertices = [[0,0],[1,0],[0,1],[1,1]]
        self.tangram_vertices = tangram_vertices
        self.list_vertices = list_vertices
    # reset square vertices to original values.
    def reset(self):
        self.square_vertices = [[0,0],[1,0],[0,1],[1,1]]
        return 0
    def rotate(self):
        self.square_vertices = [[0, sqrt(2)/2],[sqrt(2)/2,0],[sqrt(2)/2,sqrt(2)],[sqrt(2),sqrt(2)/2]]
    def get_possible_moves(self):
        # container to return possible moves
        possible_moves = list_vertices[:]

        ####### First do all possible moves assuming non-rotated square ######
        # derive x limit due to shape geometry.
        max_x = 0
        max_x_idx = []
        if list_vertices != []:
            for i in range(0,list_vertices):
                for j in range(0,list_vertices[i]):
                    if max_x < list_vertices[i][j][0]:
                        max_x = list_vertices[i][j][0]
                        max_x_idx = [i, j]
        # derive x limits due to tangram perimeter geometry.
        min_x_tangram = tangram_vertices[1][0]
        max_x_tangram = tangram_vertices[2][0] - square_vertices[1][0]

        #compute possibilities
        if max_x_tangram > max_x > min_x_tangram:

            for i in range(0, len(square_vertices)):
                square_vertices[i][0] = square_vertices[i][0] + max_x
            possible_moves.append(square_vertices)

            for i in range(0, len(square_vertices)):
                square_vertices[i][1] = square_vertices[i][1]  + square_vertices[1][0]*(sqrt(2) - 1)
            possible_moves.append(square_vertices)

        ####### Second do all possible moves assuming rotated square ######
        #reset and rotate vertices
        self.reset()
        self.rotate()
        #use x limit found above, try to fit square in, if tangram geometry allows for it.
        if max_x_tangram > max_x > min_x_tangram:
            # interaction between rotated square and triangle
            if len(list_vertices[max_x_idx[0]]) == 3:
                counter = 0
                for i in range(list_vertices[max_x_idx[0]]):
                    if list_vertices[max_x_idx[0]][i][0] == max_x:
                        counter = counter + 1
                # if triangle is facing the square with the pointy end.
                if counter == 1:
                    # triangle above the half way line. have square top vertex alignment
                    if list_vertices[max_x_idx[0]][max_x_idx[1]][1] > sqrt(2)/2:
                            diffx = list_vertices[max_x_idx[0]][max_x_idx[1]][0] - square_vertices[2][0]
                            diffy = list_vertices[max_x_idx[0]][max_x_idx[1]][1] - square_vertices[2][1]
                            for i in range(0, square_vertices):
                                square_vertices[i][0] = square_vertices[i][0] + diffx
                                square_vertices[i][1] = square_vertices[i][1] + diffy
                                # if out of bounds due to tangram geometry, no more possible moves possible.
                                if square_vertices[i][1] < 0 or square_vertices[i][1] > sqrt(2):
                                    self.reset()
                                    return possible_moves

                            possible_moves.append(square_vertices)
                    # triangle below halfway line. have square bottom vertex alignment.
                    else:
                        diffx = list_vertices[max_x_idx[0]][max_x_idx[1]][0] - square_vertices[1][0]
                        diffy = list_vertices[max_x_idx[0]][max_x_idx[1]][1] - square_vertices[1][1]
                        for i in range(0, square_vertices):
                            square_vertices[i][0] = square_vertices[i][0] + diffx
                            square_vertices[i][1] = square_vertices[i][1] + diffy
                            # if out of bounds due to tangram geometry, no more possible moves possible.
                            if square_vertices[i][1] < 0 or square_vertices[i][1] > sqrt(2):
                                self.reset()
                                return possible_moves
                        possible_moves.append(square_vertices)

                # triangle has a flat surface facing towards square.
                else:





        elif max_x == 0:
            possible_moves.append(square_vertices)



        return possible_moves