/* common definitions for the sgsmooth plugin */
#ifndef _SGSMOOTH_PLUGIN_H
#define _SGSMOOTH_PLUGIN_H

#include <stdio.h>
#include <stddef.h>             // for size_t
#include <math.h>               // for fabs  
#include <vector>

#include "AsyncLogger.hpp"

/* Matrix class.
 *
 * This is a matrix class derived from a vector of std::vector<double>s.  Note that
 * the matrix elements indexed [row][column] with indices starting at 0 (c
 * style). Also note that because of its design looping through rows should
 * be faster than looping through columns.  
 *
 * \brief two dimensional floating point array
 */
class SGMatrix : public std::vector<std::vector<double>> {
private:
    //! disable the default constructor
    explicit SGMatrix() {};
    //! disable assignment operator until it is implemented.
    SGMatrix &operator =(const SGMatrix &) { return *this; };
public:
    //! constructor with sizes
    SGMatrix(const size_t rows, const size_t cols, const double def=0.0);
    //! copy constructor for matrix
    SGMatrix(const SGMatrix &m);
    //! copy constructor for vector
    SGMatrix(const std::vector<double> &v);

    //! use default destructor
    // ~SGMatrix() {};

    //! get size
    size_t nr_rows(void) const { return size(); };
    //! get size
    size_t nr_cols(void) const { return front().size(); };
};

class SGSmoothUtil {

public:
	static std::vector<double> SGSmooth(const std::vector<double> &v, const int w, const int deg);
	static std::vector<double> SGDerivative(const std::vector<double> &v, const int w, const int deg, const double h=1.0);

};

#endif