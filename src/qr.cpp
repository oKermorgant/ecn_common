#include <ecn_common/qr.h>

#ifdef VISP_HAVE_LAPACK
#ifdef VISP_HAVE_LAPACK_BUILT_IN
typedef long int integer;
#else
typedef int integer;
#endif
#endif

namespace ecn {

#ifdef VISP_HAVE_LAPACK
extern "C" integer dgeqrf_(integer *m, integer *n, double*a, integer *lda, double *tau, double *work, integer *lwork, integer *info);
extern "C" integer dgeqp3_(integer *m, integer *n, double*a, integer *lda, integer *p, double *tau, double *work, integer* lwork, integer *info);
extern "C" integer dorgqr_(integer *, integer *, integer *, double *, integer *,
                           double *, double *, integer *, integer *);
extern "C" int dtrtri_(char *uplo, char *diag, integer *n, double *a, integer *lda, integer *info);

int allocate_work(double **work)
{
  unsigned int dimWork = (unsigned int)((*work)[0]);
  delete[] * work;
  *work = new double[dimWork];
  return (int)dimWork;
}

#endif

unsigned int QR(const vpMatrix &M, vpMatrix &Q, vpMatrix &R, bool full, bool squareR, double tol)
{
#ifdef VISP_HAVE_LAPACK_C
  // do the decomposition here...
  integer m = (integer) M.getRows();     // also rows of Q
  integer n = (integer) M.getCols();     // also columns of R
  integer r = std::min(n,m);  // a priori non-null rows of R = rank of R
  integer q = r;              // columns of Q and rows of R
  integer na = n;             // columns of A

  // cannot be full decomposition if m < n
  if(full && m > n)
  {
    q = m;              // Q is square
    na = m;             // A is square
  }

  // prepare matrices and deal with r = 0
  Q.resize(m,q);
  if(squareR)
    R.resize(r,r);
  else
    R.resize(r,n);
  if(r == 0)
    return 0;

  integer dimWork = -1;
  double * qrdata = new double[m*na];
  double *tau = new double[std::min(m,q)];
  double *work = new double[1];
  integer info;

  // copy this to qrdata in Lapack convention
  for(int i = 0; i < m; ++i)
  {
    for(int j = 0; j < n; ++j)
      qrdata[i+m*j] = M.data[j + n*i];
    for(int j = n; j < na; ++j)
      qrdata[i+m*j] = 0;
  }

  //   work = new double[1];
  //1) Extract householder reflections (useful to compute Q) and R
  dgeqrf_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,
        &m,
        tau,
        work,           //Internal working array. dimension (MAX(1,LWORK))
        &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
        &info           //status
        );

  if(info != 0){
    std::cout << "dgeqrf_:Preparation:" << -info << "th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    throw vpMatrixException::badValue;
  }
  dimWork = allocate_work(&work);

  dgeqrf_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,
        &m,            //The leading dimension of the array A.  LDA >= max(1,M).
        tau,
        work,           //Internal working array. dimension (MAX(1,LWORK))
        &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
        &info           //status
        );

  if(info != 0){
    std::cout << "dgeqrf_:" << -info << "th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    throw vpMatrixException::badValue;
  }

  // data now contains the R matrix in its upper triangular (in lapack convention)
  // update rank

  // copy useful part of R from Q and update rank
  na = std::min(m,n);
  if(squareR)
  {
    for(int i=0;i<na;i++)
    {
      for(int j=i;j<na;j++)
        R[i][j] = qrdata[i+m*j];
      if(std::abs(qrdata[i+m*i]) < tol)
        r--;
    }
  }
  else
  {
    for(int i=0;i<na;i++)
    {
      for(int j=i;j<n;j++)
        R[i][j] = qrdata[i+m*j];
      if(std::abs(qrdata[i+m*i]) < tol)
        r--;
    }
  }

  // extract Q
  dorgqr_(&m, // The number of rows of the matrix Q. M >= 0.
          &q, // The number of columns of the matrix Q. M >= N >= 0.
          &q,
          qrdata,
          &m,            //The leading dimension of the array A.  LDA >= max(1,M).
          tau,
          work,           //Internal working array. dimension (MAX(1,LWORK))
          &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
          &info           //status
          );

  // write qrdata into Q
  for(int i = 0; i < m; ++i)
    for(int j = 0; j < q; ++j)
      Q[i][j] = qrdata[i+m*j];

  delete[] qrdata;
  delete[] work;
  delete[] tau;
  return (unsigned int) r;

#else
  throw(vpException(vpException::fatalError, "Cannot perform QR decomposition. Install Lapack 3rd party"));
#endif
}


unsigned int QRPivot(const vpMatrix &M, vpMatrix &Q, vpMatrix &R, vpMatrix &P, bool full, bool squareR, double tol)
{
#ifdef VISP_HAVE_LAPACK_C
  integer m = (integer) M.getRows();     // also rows of Q
  integer n = (integer) M.getCols();     // also columns of R
  integer r = std::min(n,m);             // a priori non-null rows of R = rank of R
  integer q = r;                         // columns of Q and rows of R
  integer na = n;

  // cannot be full decomposition if m < n
  if(full && m > n)
  {
    q = m;              // Q is square
    na = m;
  }

  // prepare Q and deal with r = 0
  Q.resize(m, q);
  if(r == 0)
  {
    if(squareR)
    {
      R.resize(0, 0);
      P.resize(0, n);
    }
    else
    {
      R.resize(r, n);
      P.resize(n, n);
    }
    return 0;
  }

  integer dimWork = -1;
  double* qrdata = new double[m*na];
  double* tau = new double[std::min(q,m)];
  double* work = new double[1];
  integer* p = new integer[na];
  for(int i = 0; i < na; ++i)
    p[i] = 0;

  integer info;

  // copy this to qrdata in Lapack convention
  for(int i = 0; i < m; ++i)
  {
    for(int j = 0; j < n; ++j)
      qrdata[i+m*j] = M[i][j];
    for(int j = n; j < na; ++j)
      qrdata[i+m*j] = 0;
  }

  //1) Extract householder reflections (useful to compute Q) and R
  dgeqp3_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,    /*On entry, the M-by-N matrix A.                                                                                                                                        */
        &m,      //The leading dimension of the array A.  LDA >= max(1,M).
        p,         // Dimension N
        tau,        /*Dimension (min(M,N))                                                                                                                                        */
        work,       //Internal working array. dimension (3*N)

        &dimWork,
        &info       //status
        );

  if(info != 0){
    std::cout << "dgeqp3_:Preparation:" << -info << "th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    delete[] p;
    throw vpMatrixException::badValue;
  }

  dimWork = allocate_work(&work);

  dgeqp3_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,    /*On entry, the M-by-N matrix A.                                                                                                                                        */
        &m,      //The leading dimension of the array A.  LDA >= max(1,M).
        p,         // Dimension N
        tau,        /*Dimension (min(M,N))                                                                                                                                        */
        work,       //Internal working array. dimension (3*N)

        &dimWork,
        &info       //status
        );

  if(info != 0){
    std::cout << "dgeqp3_:" << -info << " th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    delete[] p;
    throw vpMatrixException::badValue;
  }


  // data now contains the R matrix in its upper triangular (in lapack convention)
  // get rank of R in r
  na = std::min(n,m);
  for(int i = 0; i < na; ++i)
    if(std::abs(qrdata[i+m*i]) < tol)
      r--;

  // write R
  if(squareR) // R r x r
  {
    R.resize(r, r);
    for(int i=0;i<r;i++)
      for(int j=i;j<r;j++)
        R[i][j] = qrdata[i+m*j];

    // write P
    P.resize(r,n);
    for(int i = 0; i < r; ++i)
      P[i][p[i]-1] = 1;
  }
  else        // R is min(m,n) x n of rank r
  {
    R.resize(na, n);
    for(int i=0;i<na;i++)
      for(int j=i;j<n;j++)
        R[i][j] = qrdata[i+m*j];
    // write P
    P.resize(n,n);
    for(int i = 0; i < n; ++i)
      P[i][p[i]-1] = 1;
  }

  // extract Q
  dorgqr_(&m, // The number of rows of the matrix Q. M >= 0.
          &q, // The number of columns of the matrix Q. M >= N >= 0.
          &q,
          qrdata,
          &m,            //The leading dimension of the array A.  LDA >= max(1,M).
          tau,
          work,           //Internal working array. dimension (MAX(1,LWORK))
          &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
          &info           //status
          );

  // write qrdata into Q
  for(int i = 0; i < m; ++i)
    for(int j = 0; j < q; ++j)
      Q[i][j] = qrdata[i+m*j];

  delete[] qrdata;
  delete[] work;
  delete[] tau;
  delete[] p;
  return (unsigned int) r;
#else
  throw(vpException(vpException::fatalError, "Cannot perform QR decomposition. Install Lapack 3rd party"));
#endif

}

vpMatrix inverseTriangular(const vpMatrix &A, bool upper)
{
  if(A.getCols() != A.getRows())
    throw vpMatrixException::dimensionError;

  if(A.getCols() == 0)
    return A;

  integer n = (integer) A.getCols();
  vpMatrix R = A;
  integer info;

  if(A.getRows() > 1 && upper) // upper
    dtrtri_((char *)"L", (char *)"N", &n, R.data, &n, &info);
  else
    dtrtri_((char *)"U", (char *)"N", &n, R.data, &n, &info);

  if (info != 0) {
    if (info < 0)
      std::cout << "dtrtri_:" << -info << "th element had an illegal value" << std::endl;
    else if (info > 0) {
      std::cout << "dtrtri_:R(" << info << "," << info << ")"
                << " is exactly zero.  The triangular matrix is singular "
                   "and its inverse can not be computed."
                << std::endl;
      std::cout << "R=" << std::endl << A << std::endl;
    }
    throw vpMatrixException::divideByZeroError;
  }
  return R;
}

vpColVector solveByQR(const vpMatrix &A, const vpColVector &b)
{
  vpMatrix Q, R, P;
  // we use QR Pivot to be sure to get an invertible R
  uint r = ecn::QRPivot(A.transpose(), Q, R, P, false, true);
  return  Q.extract(0, 0, A.getCols(), r)
      * ecn::inverseTriangular(R).transpose()
      * P * b;
}

}
