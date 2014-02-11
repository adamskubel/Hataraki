
#include "SavitzkyGolaySmooth.hpp"

//default convergence
static const double TINY_FLOAT = 1.0e-300;


void sgs_error(const char * errorString)
{
	AsyncLogger::getInstance().postConsoleOutTask(std::string(errorString));
}

// constructor with sizes
SGMatrix::SGMatrix(const size_t rows,const size_t cols,const double defval) 
        : std::vector<std::vector<double>>(rows) {
    int i;
    for (i = 0; i < rows; ++i) {
        (*this)[i].resize(cols, defval);
    }
    if ((rows < 1) || (cols < 1)) {
        char buffer[1024];
        
        sprintf(buffer, "cannot build matrix with %d rows and %d columns\n",
                rows, cols);
        sgs_error(buffer);
    }
}

// copy constructor for matrix
SGMatrix::SGMatrix(const SGMatrix &m) : std::vector<std::vector<double>>(m.size()) {
                
    SGMatrix::iterator inew = begin();
    SGMatrix::const_iterator iold = m.begin();
    for (/* empty */; iold < m.end(); ++inew, ++iold) {
        const size_t oldsz = iold->size();
        inew->resize(oldsz);
        const std::vector<double> oldvec(*iold);
        *inew = oldvec;
    }
}

// copy constructor for vector
SGMatrix::SGMatrix(const std::vector<double> &v) 
        : std::vector<std::vector<double>>(1) {

    const size_t oldsz = v.size();
    front().resize(oldsz);
    front() = v;
}

//////////////////////
// Helper functions //
//////////////////////


//! permute() orders the rows of A to match the integers in the index array.
void permute(SGMatrix &A, std::vector<int> &idx) 
{
    std::vector<int> i(idx.size());
    int j,k;
    
    for (j = 0; j < A.nr_rows(); ++j) {
        i[j] = j;
    }
  
    // loop over permuted indices
    for (j = 0; j < A.nr_rows(); ++j) { 
        if (i[j] != idx[j]) {

            // search only the remaining indices
            for (k = j+1; k < A.nr_rows(); ++k) { 
                if (i[k] ==idx[j]) {
                    std::swap(A[j],A[k]); // swap the rows and
                    i[k] = i[j];     // the elements of
                    i[j] = idx[j];   // the ordered index.
                    break; // next j
                }
            }
        }
    }
}

/*! \brief Implicit partial pivoting.  
 *
 * The function looks for pivot element only in rows below the current
 * element, A[idx[row]][column], then swaps that row with the current one in
 * the index map. The algorithm is for implicit pivoting (i.e., the pivot is
 * chosen as if the max coefficient in each row is set to 1) based on the
 * scaling information in the vector scale. The map of swapped indices is
 * recorded in swp. The return value is +1 or -1 depending on whether the
 * number of row swaps was even or odd respectively. */
static int partial_pivot(SGMatrix &A, const size_t row, const size_t col, 
                         std::vector<double> &scale, std::vector<int> &idx, double tol)
{
    if (tol <= 0.0)
        tol = TINY_FLOAT;

    int swapNum = 1;

    // default pivot is the current position, [row,col]
    int pivot = row; 
    double piv_elem = fabs(A[idx[row]][col]) * scale[idx[row]];

    // loop over possible pivots below current
    int j;
    for (j = row + 1; j < A.nr_rows(); ++j) { 

        const double tmp = fabs(A[idx[j]][col]) * scale[idx[j]];  

        // if this elem is larger, then it becomes the pivot 
        if (tmp > piv_elem) {     
            pivot = j;
            piv_elem = tmp;
        }
    }

#if 0
    if(piv_elem < tol) {
        sgs_error("partial_pivot(): Zero pivot encountered.\n")
#endif

    if(pivot > row) {           // bring the pivot to the diagonal
        j = idx[row];           // reorder swap array
        idx[row] = idx[pivot];
        idx[pivot] = j;
        swapNum = -swapNum;     // keeping track of odd or even swap
    }
    return swapNum;
}

/*! \brief Perform backward substitution.
 *
 * Solves the system of equations A*b=a, ASSUMING that A is upper
 * triangular. If diag==1, then the diagonal elements are additionally
 * assumed to be 1.  Note that the lower triangular elements are never
 * checked, so this function is valid to use after a LU-decomposition in
 * place.  A is not modified, and the solution, b, is returned in a. */
static void lu_backsubst(SGMatrix &A, SGMatrix &a, bool diag=false) 
{
    int r,c,k;
    
    for (r = (A.nr_rows() - 1); r >= 0; --r) {
        for (c = (A.nr_cols() - 1); c > r; --c) {
            for (k = 0; k < A.nr_cols(); ++k) {
                a[r][k] -= A[r][c] * a[c][k];
            }
        }
        if(!diag) {
            for (k = 0; k < A.nr_cols(); ++k) {
                a[r][k] /= A[r][r];
            }
        }
    }
}

/*! \brief Perform forward substitution.
 *
 * Solves the system of equations A*b=a, ASSUMING that A is lower
 * triangular. If diag==1, then the diagonal elements are additionally
 * assumed to be 1.  Note that the upper triangular elements are never
 * checked, so this function is valid to use after a LU-decomposition in
 * place.  A is not modified, and the solution, b, is returned in a. */
static void lu_forwsubst(SGMatrix &A, SGMatrix &a, bool diag=true) 
{
    int r,k,c;
    for (r = 0;r < A.nr_rows(); ++r) {
        for(c = 0; c < r; ++c) {
            for (k = 0; k < A.nr_cols(); ++k) {
                a[r][k] -= A[r][c] * a[c][k];
            }
        }
        if(!diag) {
            for (k = 0; k < A.nr_cols(); ++k) {
                a[r][k] /= A[r][r];
            }
        }
    }
}

/*! \brief Performs LU factorization in place.  
 *
 * This is Crout's algorithm (cf., Num. Rec. in C, Section 2.3).  The map of
 * swapped indeces is recorded in idx. The return value is +1 or -1
 * depending on whether the number of row swaps was even or odd
 * respectively.  idx must be preinitialized to a valid set of indices
 * (e.g., {1,2, ... ,A.nr_rows()}). */
static int lu_factorize(SGMatrix &A, std::vector<int> &idx, double tol=TINY_FLOAT)
{
    if ( tol <= 0.0) 
        tol = TINY_FLOAT;

    if ((A.nr_rows() == 0) || (A.nr_rows() != A.nr_cols())) {
        sgs_error("lu_factorize(): cannot handle empty "
                  "or nonsquare matrices.\n");
        
        return 0;
    }

    std::vector<double> scale(A.nr_rows());  // implicit pivot scaling
    int i,j;
    for (i = 0; i < A.nr_rows(); ++i) {
        double maxval = 0.0;
        for (j = 0; j < A.nr_cols(); ++j) {
            if (fabs(A[i][j]) > maxval)
                maxval = fabs(A[i][j]);
        }
        if (maxval == 0.0) {
            sgs_error("lu_factorize(): zero pivot found.\n");
            return 0;
        }
        scale[i] = 1.0 / maxval;
    }

    int swapNum = 1;
    int c,r;
    for (c = 0; c < A.nr_cols() ; ++c) {            // loop over columns
        swapNum *= partial_pivot(A, c, c, scale, idx, tol); // bring pivot to diagonal
        for(r = 0; r < A.nr_rows(); ++r) {      //  loop over rows
            int lim = (r < c) ? r : c;
            for (j = 0; j < lim; ++j) {
                A[idx[r]][c] -= A[idx[r]][j] * A[idx[j]][c];
            }
            if (r > c) 
                A[idx[r]][c] /= A[idx[c]][c];
        }
    }
    permute(A,idx);
    return swapNum;
}

/*! \brief Solve a system of linear equations. 
 * Solves the inhomogeneous matrix problem with lu-decomposition. Note that
 * inversion may be accomplished by setting a to the identity_matrix. */
static SGMatrix lin_solve(const SGMatrix &A, const SGMatrix &a, 
                           double tol=TINY_FLOAT) 
{
    SGMatrix B(A);
    SGMatrix b(a);
    std::vector<int> idx(B.nr_rows());
    int j;
    
    for (j = 0; j < B.nr_rows(); ++j) {
        idx[j] = j;  // init row swap label array
    }
    lu_factorize(B,idx,tol); // get the lu-decomp.
    permute(b,idx);          // sort the inhomogeneity to match the lu-decomp
    lu_forwsubst(B,b);       // solve the forward problem
    lu_backsubst(B,b);       // solve the backward problem
    return b;
}

///////////////////////
// related functions //
///////////////////////

//! Returns the inverse of a matrix using LU-decomposition. 
static SGMatrix invert(const SGMatrix &A) 
{
    const int n = A.size();
    SGMatrix E(n, n, 0.0);
    SGMatrix B(A);
    int i;

    for (i = 0; i < n; ++i) {
        E[i][i] = 1.0;
    }

    return lin_solve(B, E);
}

//! returns the transposed matrix.
static SGMatrix transpose(const SGMatrix &a)
{
    SGMatrix res(a.nr_cols(), a.nr_rows());
    int i,j;
    
    for (i = 0; i < a.nr_rows(); ++i) {
        for (j = 0; j < a.nr_cols(); ++j) {
            res[j][i] = a[i][j];
        }
    }
    return res;
}

//! matrix multiplication.
SGMatrix operator *(const SGMatrix &a, const SGMatrix &b)
{
    SGMatrix res(a.nr_rows(), b.nr_cols());
    if (a.nr_cols() != b.nr_rows()) {
        sgs_error("incompatible matrices in multiplication\n");
        return res;
    }

    int i,j,k;
    
    for (i = 0; i < a.nr_rows(); ++i) {
        for (j = 0; j < b.nr_cols(); ++j) {
            double sum(0.0);
            for (k = 0; k < a.nr_cols(); ++k) {
                sum += a[i][k] * b[k][j];
            }
            res[i][j] = sum;
        }
    }
    return res;
}


//! calculate savitzky golay coefficients.
static std::vector<double> SGCoefficients(const std::vector<double> &b, const size_t deg)
{
    const size_t rows(b.size());
    const size_t cols(deg + 1);
    SGMatrix A(rows, cols);
    std::vector<double> res(rows);
        
    // generate input matrix for least squares fit
    int i,j;
    for (i = 0; i < rows; ++i) {
        for (j = 0; j < cols; ++j) {
            A[i][j] = pow(double(i), double(j));
        }
    }

    SGMatrix c(invert(transpose(A) * A) * (transpose(A) * transpose(b)));

    for (i = 0; i < b.size(); ++i) {
        res[i] = c[0][0];
        for (j = 1; j <= deg; ++j) {
            res[i] += c[j][0] * pow(double(i), double(j));
        }
    }
    return res;
}

/*! \brief savitzky golay smoothing.  
 *
 * This method means fitting a polynome of degree 'deg' to a sliding window
 * of width 2w+1 throughout the data.  The needed coefficients are
 * generated dynamically by doing a least squares fit on a "symmetric" unit
 * vector of size 2w+1, e.g. for w=2 b=(0,0,1,0,0). evaluating the polynome
 * yields the sg-coefficients.  at the border non symmectric vectors b are
 * used. */
std::vector<double> SGSmoothUtil::SGSmooth(const std::vector<double> &v, const int width, const int deg)
{
    std::vector<double> res(v.size(), 0.0);
    if ((width < 1) || (deg < 1) || (v.size() < (2 * width + 2))) {
        sgs_error("sgsmooth: parameter error.\n");
        return res;
    }

    const int window = 2 * width + 1;
    const int endidx = v.size() - 1;

    // handle border cases first because we need different coefficients
    int i,j;
#if defined(_OPENMP)
#pragma omp parallel for private(i,j) schedule(static)
#endif    
    for (i = 0; i < width; ++i) {
        std::vector<double> b1(window, 0.0);
        b1[i] = 1.0;

        const std::vector<double> c1(SGCoefficients(b1, deg));
        for (j = 0; j < window; ++j) {
            res[i]          += c1[j] * v[j];
            res[endidx - i] += c1[j] * v[endidx - j];
        } 
    }

    // now loop over rest of data. reusing the "symmetric" coefficients.
    std::vector<double> b2(window, 0.0);
    b2[width] = 1.0;
    const std::vector<double> c2(SGCoefficients(b2, deg));

#if defined(_OPENMP)
#pragma omp parallel for private(i,j) schedule(static)
#endif    
    for (i = 0; i <= (v.size() - window); ++i) {
        for (j = 0; j < window; ++j) {
            res[i + width] += c2[j] * v[i + j];
        } 
    }
    return res;
}

/*! least squares fit a polynome of degree 'deg' to data in 'b'.
 *  then calculate the first derivative and return it. */
static std::vector<double> lsqr_fprime(const std::vector<double> &b, const int deg)
{
    const int rows(b.size());
    const int cols(deg + 1);
    SGMatrix A(rows, cols);
    std::vector<double> res(rows);

    // generate input matrix for least squares fit
    int i,j;
    for (i = 0; i < rows; ++i) {
        for (j = 0; j < cols; ++j) {
            A[i][j] = pow(double(i), double(j));
        }
    }

    SGMatrix c(invert(transpose(A) * A) * (transpose(A) * transpose(b)));
        
    for (i = 0; i < b.size(); ++i) {
        res[i] = c[1][0];
        for (j = 1; j < deg; ++j) {
            res[i] += c[j + 1][0] * double(j+1)
                * pow(double(i), double(j));
        }
    }
    return res;
}

/*! \brief savitzky golay smoothed numerical derivative.  
 *
 * This method means fitting a polynome of degree 'deg' to a sliding window
 * of width 2w+1 throughout the data.  
 *
 * In contrast to the SGSmooth function we do a brute force attempt by
 * always fitting the data to a polynome of degree 'deg' and using the
 * result. */
std::vector<double> SGSmoothUtil::SGDerivative(const std::vector<double> &v, const int width, const int deg, const double h)
{
    std::vector<double> res(v.size(), 0.0);
    if ((width < 1) || (deg < 1) || (v.size() < (2 * width + 2))) {
        sgs_error("sgsderiv: parameter error.\n");
        return res;
    }

    const int window = 2 * width + 1;

    // handle border cases first because we do not repeat the fit
    // lower part
    std::vector<double> b(window, 0.0);
    int i,j;
    
    for (i = 0; i < window; ++i) {
        b[i] = v[i] / h;
    }
    const std::vector<double> c(lsqr_fprime(b, deg));
    for (j = 0; j <= width; ++j) {
        res[j] = c[j];
    }
    // upper part. direction of fit is reversed
    for (i = 0; i < window; ++i) {
        b[i] = v[v.size() - 1 - i] / h;
    }
    const std::vector<double> d(lsqr_fprime(b, deg));
    for (i = 0; i <= width; ++i) {
        res[v.size() - 1 - i] = -d[i];
    }

    // now loop over rest of data. wasting a lot of least squares calcs
    // since we only use the middle value.
#if defined(_OPENMP)
#pragma omp parallel for private(i,j) schedule(static)
#endif    
    for (i = 1; i < (v.size() - window); ++i) {
        for (j = 0; j < window; ++j) {
            b[j] = v[i + j] / h;
        }
        res[i + width] = lsqr_fprime(b, deg)[width];
    }
    return res;
}

//}