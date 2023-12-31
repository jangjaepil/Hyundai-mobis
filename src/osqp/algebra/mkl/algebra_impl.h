#ifndef ALGEBRA_IMPL_H
#define ALGEBRA_IMPL_H

#include <mkl_spblas.h>

#include "csc_math.h"

#ifdef __cplusplus
extern "C" {
#endif

/*********************************************
*   Internal definition of OSQPVector types
*   and supporting definitions
*********************************************/

struct OSQPVectori_ {
  OSQPInt* values;
  OSQPInt  length;
};

struct OSQPVectorf_ {
  OSQPFloat* values;
  OSQPInt    length;
};


/*********************************************
*   Internal definition of OSQPMatrix type
*   and supporting definitions
*********************************************/

/**
 *  An enum used to indicate whether a matrix is symmetric.   Options
 *  NONE : matrix is fully populated
 *  TRIU : matrix is symmetric and only upper triangle is stored
 */
typedef enum OSQPMatrix_symmetry_type {NONE,TRIU} OSQPMatrix_symmetry_type;

struct OSQPMatrix_ {
  /* The memory in this matrix should be allocated using the MKL memory routines, so it should
     never be created or destroyed using the normal csc deletion routines. */
  OSQPCscMatrix*           csc;       /* Shadow matrix */
  sparse_matrix_t          mkl_mat;   /* Opaque object for MKL matrix */
  OSQPMatrix_symmetry_type symmetry;
};

#ifdef __cplusplus
}
#endif

#endif // ifndef ALGEBRA_IMPL_H
