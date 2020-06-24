#include<visp/vpQuadProg.h>

using std::cout;
using std::endl;

void summary(std::string legend,
                vpMatrix Q, vpColVector r,
                vpMatrix A, vpColVector b,
                vpMatrix C, vpColVector d,
                vpColVector x)
{
    cout << legend << "\n";
    cout << "    Minimum at " << x.t() << '\n';
    cout << "    Objective: " << (Q*x - r).frobeniusNorm() << '\n';
    cout << "    Equalities (should be = 0 if considered): " << (A*x-b).t() << '\n';
    cout << "    Inequalities (should be <= 0 if considered): " << (C*x-d).t() << '\n' << '\n';
}

int main()
{

    // shows how to use the solvers

    /* 2 dim with inequalities only
     *
     * min_x (x1+3)^2 + x2^2
     * s.t. x1 = x2
     *      x1 >= 0
     *      x2 >= 0
     *      x1+x2 >= 15
     *      2.x1 + 5.x2 <= 100
     *      3.x1 + 4.x2 <= 80
     *
     * written as:
     * min_x ||Q.x - r||^2
     * s.t   A.x = b
     * s.t.  C.x <= d
     */

    // objective Q & r
    vpMatrix Q;Q.eye(2);
    vpColVector r(2);   r[0] = -3;

    // equality constraint A & b
    vpMatrix A(1,2);
    A[0][0] = 1;    A[0][1] = -1;
    vpColVector b(1);

    // inequality constraint C & d
    vpMatrix C(5,2);
    C[0][0] =       C[1][1] = -1;
    C[2][0] = -1;   C[2][1] = -1;
    C[3][0] = 2;    C[3][1] = 5;
    C[4][0] = 3;    C[4][1] = 4;
    vpColVector d(5);
    d[2] = -15; d[3] = 100; d[4] = 80;

    // solution vector
    vpColVector x;

    // solver
    vpQuadProg qp;

    // without constraints: no need for a solver
    x = Q.solveByQR(r);
    summary("No constraints", Q, r, A, b, C, d, x);


    // with only equalities
    qp.solveQPe(Q, r, A, b, x);
    summary("Equality constraints", Q, r, A, b, C, d, x);

    // with only inequalities
    qp.solveQPi(Q, r, C, d, x);
    summary("Inequality constraints", Q, r, A, b, C, d, x);

    // with equalities + only inequalities
    qp.solveQP(Q, r, A, b, C, d, x);
    summary("Both constraints", Q, r, A, b, C, d, x);
}
