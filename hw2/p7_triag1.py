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
        self.triag1_vertices = [[0, 0], [1, 0], [0, 1]]
        self.tangram_vertices = tangram_vertices
        self.list_vertices = list_vertices
        self.possible_moves = []
        self.max_x = 0
        self.max_x_idx = [0, 0]
        self.max_x_tangram = 0
        self.min_x_tangram = 0
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
        triag1_vertices = self.triag1_vertices[:]
        print("area1: " , self.calc_area(triag1_vertices) )

        # shift origin to centroid
        for i in range(0, len(triag1_vertices)):
            triag1_vertices[i][0] = triag1_vertices[i][0] - self.centroid[0]
            triag1_vertices[i][1] = triag1_vertices[i][1] - self.centroid[1]
        print("area2: " , self.calc_area(triag1_vertices))

        # rotate by using rotate matrix.
        for i in range(0, len(triag1_vertices)):
            print("mag0: ", sqrt( (triag1_vertices[i][0] )**2 + (triag1_vertices[i][1])**2))
            self.triag1_vertices[i][0] = (sqrt(2)/2)*(triag1_vertices[i][0] - triag1_vertices[i][1])
            self.triag1_vertices[i][1] = (sqrt(2)/2)*(triag1_vertices[i][1] + triag1_vertices[i][0])
            print("mag0: ", sqrt( (triag1_vertices[i][0] )**2 + (triag1_vertices[i][1])**2))
        print("area3: " , self.calc_area(self.triag1_vertices))

        # shift origin back.
        for i in range(0, len(triag1_vertices)):
            self.triag1_vertices[i][0] = self.centroid[0] + self.triag1_vertices[i][0]
            self.triag1_vertices[i][1] = self.centroid[1] + self.triag1_vertices[i][1]
        print("area4: " , self.calc_area(self.triag1_vertices))
        print("")
        return self.triag1_vertices


    def get_possible_moves(self):
        self.possible_moves = self.list_vertices[:]
        return self.possible_moves
