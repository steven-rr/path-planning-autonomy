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
from copy import deepcopy


class Triangle1:
    # default constructor for triangle1:
    def __init__(self, list_vertices, tangram_vertices):
        self.triag1_vertices = [[0, 0], [1, 0], [0, 1]]
        self.tangram_vertices = tangram_vertices
        self.list_vertices = list_vertices
        self.possible_moves = []
        self.max_x = 0
        self.max_x_idx = [0, 0]
        self.max_x_tangram = 0
        self.min_x_tangram = 0
        self.max_y = 0
        self.max_y_idx = 0
        self.max_y_idx_2 = [0, 0]
        self.centroid = 0
        self.area = 0
    def calc_centroid(self):
        self.centroid = [(self.triag1_vertices[0][0] + self.triag1_vertices[1][0] + self.triag1_vertices[2][0]) / 3,
                         (self.triag1_vertices[0][1] + self.triag1_vertices[1][1] + self.triag1_vertices[2][1]) / 3]

    def norm(self, vertex1, vertex2):
        return sqrt( (vertex1[0] - vertex2[0])**2 + (vertex1[1] - vertex2[1])**2 )
    def calc_area(self, triag1_vertices):
        l1 = self.norm(triag1_vertices[0], triag1_vertices[1])
        l2 = self.norm(triag1_vertices[1], triag1_vertices[2])
        l3 = self.norm(triag1_vertices[2], triag1_vertices[0])
        s = (l1 + l2 + l3) / 2
        self.area = sqrt(s * (s - l1) * (s - l2) * (s - l3))
        return self.area

    def rotate45(self):
        self.calc_centroid()
        triag1_vertices = deepcopy(self.triag1_vertices)

        # shift origin to centroid
        for i in range(0, len(triag1_vertices)):
            triag1_vertices[i][0] = triag1_vertices[i][0] - self.centroid[0]
            triag1_vertices[i][1] = triag1_vertices[i][1] - self.centroid[1]

        # rotate by using rotate matrix.
        for i in range(0, len(triag1_vertices)):
            self.triag1_vertices[i][0] = (sqrt(2)/2)*(triag1_vertices[i][0] + triag1_vertices[i][1])
            self.triag1_vertices[i][1] = (sqrt(2)/2)*(triag1_vertices[i][1] - triag1_vertices[i][0])

        # shift origin back.
        for i in range(0, len(triag1_vertices)):
            self.triag1_vertices[i][0] = self.centroid[0] + self.triag1_vertices[i][0]
            self.triag1_vertices[i][1] = self.centroid[1] + self.triag1_vertices[i][1]
        return self.triag1_vertices

    # for top, align top vertex with the top vertex for shape. Then see if there is interference. If not, that's good.
    def triag_interaction_flat(self):
        list_shapes = deepcopy(self.list_vertices)

    #if no
    # to modify y: get top vertex of triangle, and align it with top of tangram.
    # to modify x: get top, outermost, vertex of shape vertices and align it with triangle.
    # if it fits, put in the triangle, if not, don't.
    # if empty, fit pointy vertex  with left most tangram, else just quit.
    def triag_interaction_slanted(self):
        list_shapes = deepcopy(self.list_vertices)
        # just work with tangram limits:
        if list_shapes == []:

        else:
            # get the max value of y across all vertices.
            for i in range(0,len(self.triag1_vertices)):
                if self.max_y < self.triag1_vertices[i][1]:
                    self.max_y = self.triag1_vertices[i][1]

            # check if more than one vertex are aligned with a max y.
            counter = 0
            for i in range(0, len(self.triag1_vertices)):
                if self.triag1_vertices[i][1] == self.max_y:
                    counter = counter + 1
                    self.max_y_idx = i
                    if counter > 1:
                        if self.triag1_vertices[i][0] < self.triag1_vertices[self.max_y_idx][0]:
                            self.max_y_idx = i

            # get max value of y across all shapes:


    def triag_interaction(self):


    def get_possible_moves(self):
        self.possible_moves = deepcopy(self.list_vertices)
        # rotating triangle has 8 moves.


        return self.possible_moves
