
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

import numpy as np
class testing:
    def __init__(self):
        self.state = 1000

    def set_state(self , state_number):
        self.state = state_number

    def execute_function(self):
        print("it worked! ", self.state)

def test_fx(x_in):
    x_in.execute_function()

def main():

    x = testing()
    x.set_state(9001)
    X = [1, 2, 3, 4, 5]
    npx = np.array(X)
    print(npx)





if __name__ == "__main__":
    main()