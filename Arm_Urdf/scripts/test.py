import numpy as np

def ee_velo_base(e,r):
    zero_matrix = np.zeros((3, 3))
    ee_b=np.block([
        [r, zero_matrix], 
        [zero_matrix, r]
    ])@e
    print(ee_b.type)
def h(ee_b):
    b=ee_b.flatten()
    print(b)
r=np.ones((3,3))
e=np.ones((6,1))
ee_b=ee_velo_base(e,r)
h(ee_b)