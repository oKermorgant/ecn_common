import numpy as np
from numpy.linalg import pinv, norm, eig
from matplotlib.mlab import find


class SolveQP:
    def __init__(self, n, l=0):
        self.act = []
        self.n = n
        self.I = np.matrix(np.eye(n))
        
    def SolveH(self, H, c, A, b, C, d):
        '''
        min 1/2 x.H.x - c.x
            st Ax  = b
            st Cx <= d
        '''    
        D, P = eig(H)
        Q = np.matrix(np.diag(np.sqrt(D))) * P.transpose()
        r = pinv(Q.transpose())*c
        return self.Solve(Q, r, A, b, C, d)
        

    def Solve(self, Q, r, A, b, C, d):
        '''
        min ||Qx - r||^2
            st Ax  = b
            st Cx <= d
        '''
        
                
        found = False
        #self.act = []
        eps = 1e-6
        cnt = 0
        
        m = A.shape[0]
        
        while not found:
            cnt += 1

            if m:
                AC = np.concatenate((A, C[self.act,:]))
                bd = np.concatenate((b, d[self.act,:]))
            else:
                AC = C[self.act,:]
                bd = d[self.act,:]
                
            # solve with pseudo inverse
            if AC.shape[0] == 0:
                x = pinv(Q)*r
            else:
                ACp = pinv(AC)
                x = ACp * bd
                P = self.I - ACp*AC
                
                for i in xrange(P.shape[0]):
                    for j in xrange(P.shape[1]):
                        if np.abs(P[i,j]) < eps:
                            P[i,j] = 0
                x += P*pinv(Q*P) * (r - Q*x)
                
                print np.amax(P),np.amin(P)
                
                # check if infeasible
                if norm(P) < eps:   # full rank constraints                    
                    cons = abs(AC*x - bd)
                    if max(cons) > eps:
                        print('Constraints seem infeasible')
                        return 0*r
            
            # check constraints
            found = True
            if C.shape[0] > 0:
                ineq = C*x-d
                if max(ineq) > eps:
                    # add constraint
                    idx = find(ineq == max(ineq))[0]
                    if idx not in self.act:
                        self.act.append(idx)
                        print('Activating ineq. {} because is {}'.format(self.act[-1], max(ineq)[0,0]))
                        found = False
                if AC.shape[0] != 0:
                    # compute Lagrange coefs
                    mu = -ACp[:,m:].transpose() * Q.transpose() * (Q*x - r)
                    if mu.shape[0] > 0:
                        if min(mu) < -eps:
                            idx = find(mu == min(mu))[0]
                            print('Deactivating ineq. {} because is {}'.format(self.act[idx],min(mu)[0,0]))
                            self.act.pop(idx)
                            found = False

        print('Found in {} iterations'.format(cnt))
        return x
