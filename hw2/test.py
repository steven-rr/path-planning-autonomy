from math import sqrt
# assume all

class Square:
    # default constructor for square
    def __init__(self, tangram_vertices):
        self.vertices = [[0,0],[1,0],[0,1],[1,1]]
        self.tangram_vertices = tangram_vertices

    def get_centroid(self):
        return 0

    def get_possible_moves(self):
        return 0

class Triangle1:
    # default constructor for triangle1:
    def __init__(self, tangram_vertices):
        self.vertices = [[0,0],[1,0],[0,1]]
        self.tangram_vertices = tangram_vertices
    def get_centroid(self):
        return 0
    def get_possible_moves(self,list_vertices):
        list_possible_moves = []

def main():
    list = [[0,0]]
    list.append([1,1])
    print(list)
    tangram_vertices = [[0,0],[2,0],[0,2],[2,2]]
    triag1 = Triangle1(tangram_vertices)
    square = Square(tangram_vertices)

    # expand? need current state of vertices to get possible moves.
    # also what shapes have already been used? the expansion should include all possible rotations of all remaining shapes.


if __name__ == "__main__":
    main()