from math import sqrt
from p7_triag1 import Triangle1
import matplotlib.pyplot as plt
# assume all


def main():
    list_tangram       = [[0,sqrt(2)/2]        , [sqrt(2)/2,0]               ,[sqrt(2)/2+sqrt(2), 0],
                          [2*sqrt(2),sqrt(2)/2], [sqrt(2)/2+sqrt(2), sqrt(2)],[sqrt(2)/2,sqrt(2)]]
    plt.figure(1)
    triag1 = Triangle1([],list_tangram)
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)

    triag1.rotate45()
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)

    triag1.rotate45()
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)

    triag1.rotate45()
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)

    plt.figure(2)
    triag1.rotate45()
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)

    triag1.rotate45()
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)

    triag1.rotate45()
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)

    triag1.rotate45()
    x_vals = [triag1.triag1_vertices[0][0], triag1.triag1_vertices[1][0], triag1.triag1_vertices[2][0], triag1.triag1_vertices[0][0]]
    y_vals = [triag1.triag1_vertices[0][1], triag1.triag1_vertices[1][1], triag1.triag1_vertices[2][1], triag1.triag1_vertices[0][1]]
    plt.plot(x_vals, y_vals)
    plt.show()

if __name__ == "__main__":
    main()