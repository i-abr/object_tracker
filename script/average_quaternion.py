import numpy as np
import numpy.matlib as npm

def average_quaterion(Q):
    '''
    Here, Q is of shape Nx4 where N is the number of
    quaternions that we are averaging
    '''
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0, M):
        q = Q[i,:]
        A = np.outer(q, q) + A

    # scale this
    A = (1.0/M) * A
    # ccompute all the eigenvalues and vectors and whatnot
    e_vals, e_vecs = np.linalg.eig(A)
    e_vecs = e_vecs[:, e_vals.argsort()[::-1]]
    return np.real(e_vecs[:,0].A1)
