
# ////////////////////////////////////////////////////////////////////////////
# //|
# //| Author : Steven Rivadeneira
# //|
# //| File Name : test.py
# //|
# //| Description : Solves MDP using value iteration
# //|
# //| Notes :
# //|
# //|
# //|
# //|
# ////////////////////////////////////////////////////////////////////////////

def main():

    dict = {(2,3): "D", (1,1): "L"}
    dict[(2,1)] = "U"
    state = [2,3]
    # print(dict[(state[0], state[1])])
    # print(dict[(2,1)])
    x = ["D","U", "R"]
    y = "U"
    if y in x:
        print("this works!", y)

if __name__ == "__main__":
    main()