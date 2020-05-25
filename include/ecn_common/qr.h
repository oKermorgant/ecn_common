#ifndef ECNQR_H
#define ECNQR_H

#include <visp/vpMatrix.h>
#include <visp/vpMatrixException.h>

namespace ecn
{

unsigned int QR(const vpMatrix &M, vpMatrix &Q, vpMatrix &R,
       bool full = false, bool squareR = false, double tol = 1e-6);
unsigned int QRPivot(const vpMatrix &M, vpMatrix &Q, vpMatrix &R, vpMatrix &P,
            bool full = false, bool squareR = false, double tol = 1e-6);
vpMatrix inverseTriangular(const vpMatrix &A, bool upper = true);

vpColVector solveByQR(const vpMatrix &A, const vpColVector &b);
}

#endif // ECNQR_H
