from math import sqrt
# assume all


def main():
    w = [1, 2]
    v = [3, 5]

    norm1 = w[0] + (w[1]*v[0] - w[0]*v[1])/(v[0]**2 + v[1]**2)
    norm2 = w[0]*v[0] + w[1]*v[1]/ (v[0]**2 + v[1]**2)
    print(norm1)
    print(norm2)
    return 0
if __name__ == "__main__":
    main()