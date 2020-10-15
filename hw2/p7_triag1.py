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
        self.max_x_tangram = 0
        self.min_x_tangram = 0
        # right most point for the triangle.
        self.min_x = 0
        # left most point for the triangle.
        self.max_x = 0
        # max y vertex for the triangle.
        self.max_y = 0
        self.max_y_idx = 0
        self.max_y_idx_2 = [0, 0]
        # max y vertex across all shapes
        self.max_y_shape = 0
        self.max_y_shape_idx = [0, 0]
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

    # get triag vertices extreme. max y. left most point. right most point.
    def get_triag_vertices_extremes(self):
        for i in range(0, len(self.triag1_vertices)):
            if self.max_y < self.triag1_vertices[i][1]:
                self.max_y = self.triag1_vertices[i][1]

            if self.max_x < self.triag1_vertices[i][0]:
                self.max_x = self.triag1_vertices[i][0]

            if self.min_x > self.triag1_vertices[i][0]:
                self.min_x = self.triag1_vertices[i][0]

    # -------------------------------------------------------------------------
    #  Function: check_triag_tangram_bound
    #  Description: Check if triangle is out of bounds from tangram.
    #               If out of bounds, return false. else return true.
    # -------------------------------------------------------------------------
    def check_triag_tangram_bound(self):
        y0_max = tangram_vertices[4][1]
        y0_min = 0
        # check if vertices are before or after the middle portion , else just eliminate any triangles
        for i in range(0, len(self.triag1_vertices)):

            # if within later portion of tangram, calc ymax ymin dynamically. if exceeds bound, invalid.
            if self.triag_vertices[i][0] > self.tangram_vertices[2][0]:
                if self.triag_vertices[i][0] > self.tangram_vertices[3][0]:
                    return False
                else:
                    deltax = self.triag1_vertices[2][0] - self.triag1_vertices[i][0]
                    ymax = y0_max - deltax
                    ymin = y0_min + deltax
                    if self.triag1_vertices[i][1] > ymax:
                        return False
                    elif self.triag1_vertices[i][1] < ymin:
                        return False

            # if within later portion of tangram, calc ymax ymin dynamically. if exceeds bound, invalid.
            elif self.triag1_vertices[i][0] < self.tangram_vertices[1][0]:
                if self.triag1_vertices[i][0] < self.tangram_vertices[0][0]:
                    return False
                else:
                    deltax = self.triag1_vertices[1][0] - self.triag1_vertices[i][0]
                    ymax = y0_max - deltax
                    ymin = y0_min + deltax
                    if self.triag1_vertices[i][1] > ymax:
                        return False
                    elif self.triag1_vertices[i][1] < ymin:
                        return False
            # if in middle portion, use static y0_max and y0_min. if exceeds bound, invalid.
            else:
                if self.triag1_vertices[i][1] > y0_max:
                    return False
                elif self.triag1_vertices[i][1] < y0_min:
                    return False

        return True
    # -------------------------------------------------------------------------
    #  Function: check_triag_shape_bound
    #  Description: Check if triangle is out of bounds based on shape configuration.
    #               If out of bounds, return false. else return true.
    # -------------------------------------------------------------------------
    def check_triag_shape_bound(self):
        for i in range(0, len(self.list_vertices)):
            # square interaction
            if len(self.list_vertices[i]]) == 4:
                for j in range(0, len(self.list_vertices[i])):
            # triangle interaction:
            elif len(self.list_verices[i]) == 3:



    # -------------------------------------------------------------------------
    #  Function: triag_interaction_flat
    #  Description: Return all possible moves for a flat triangle. For top, align top vertex ith top vertex of shape.
    #               See if there is interferance. If not, then it's a possible move.
    # -------------------------------------------------------------------------
    def triag_interaction_flat(self):
        list_shapes = deepcopy(self.list_vertices)

    # -------------------------------------------------------------------------
    #  Function: triag_interaction_slanted
    #  Description: Return all possible moves for a slanted triangle.
    #               Align top left most vertex of triangle with top right most vertex of shapes.
    #               Do the same for the bottom.
    #               If doesn't break intersect with another shape, then it's good.
    # -------------------------------------------------------------------------
    def triag_interaction_slanted(self):
        list_shapes = deepcopy(self.list_vertices)
        # just work with tangram limits:
        if list_shapes == []:
            #place the triangle in the tangram .
        else:
            # get top and left most vertex of the triangle
            # check if more than one vertex are aligned with a max y. if so, choose the vertex with smaller x. (left most)
            counter = 0
            for i in range(0, len(self.triag1_vertices)):
                if self.triag1_vertices[i][1] == self.max_y:
                    counter = counter + 1
                    self.max_y_idx = i
                    if counter > 1:
                        if self.triag1_vertices[i][0] < self.triag1_vertices[self.max_y_idx][0]:
                            self.max_y_idx = i

            # get index for highest, and right most vertex (closest to edge):
            counter = 0
            for i in range(0, len(self.list_vertices)):
                for j in range(0, len(self.list_vertices[i])):
                    if self.list_vertices[i][j][1] > self.max_y_shape:
                        self.max_y_shape = self.list_vertices[i][j][1]
                        self.max_y_shape_idx = [i,j]
                        counter = counter + 1
                        # if x coordinate is more towards the right, then get that vertex, else just keep the old vertex.
                        if counter > 1:
                            if self.list_vertices[i][j][0] > self.list_vertices[self.max_y_shape_idx[0]][self.max_y_shape_idx[1]][0]:
                                self.max_y_shape_idx = [i, j]

            # align highest, left most vertex of triangle to highest, right most vertex from the shapes.
            diffx = self.list_vertices[self.max_y_shape_idx[0]][self.max_y_shape_idx[1]][0] - self.triag1_vertices[self.max_y_shape_idx][0]
            diffy = self.list_vertices[self.max_y_shape_idx[0]][self.max_y_shape_idx[1]][1] - self.triag1_vertices[self.max_y_shape_idx][1]

            #translate all triangle vertices.
            for i in range(0, len(self.triag1_vertices)):
                self.triag1_vertices[i][0] = self.triag1_vertices[i][0] + diffx
                self.triag1_vertices[i][1] = self.triag1_vertices[i][1] + diffy

            # check if triangle vertices are out of bound. if out of bounds, return no possible moves.
            check1 = check_triag_tangram_bound()
            if not check1:
                return

            # check if triangle vertices intersect any other vertex.

    def triag_interaction(self):
        self.get_triag_vertices_extremes()

    def get_possible_moves(self):
        self.possible_moves = deepcopy(self.list_vertices)
        # rotating triangle has 8 moves.


        return self.possible_moves
