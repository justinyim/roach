/* 
 * File:   estimator.h
 * Author: Hersh Sanghvi
 *
 * Created on February 5, 2019, 8:59 PM
 */
#include <stdint.h>
#include "fixed_point.h"

#ifndef ESTIMATOR_H
#define	ESTIMATOR_H

#define Q_SCALE 14
#define W_SCALE 10
#define EPSILON 0.005

// will define later
#define PI 3.1415 
#define HALF_PI 3.1415/2
#define PI_SQUARED 3.1415 * 3.1415

int32_t fix_sign_approx(int32_t num, int32_t scale);
fixp_t cos_approx(fixp_t x);
fixp_t sin_approx(fixp_t x);
fixp_t fix_sqrt_approx(fixp_t num);
fixp_t fix_l2norm(fixp_t *vector, int num_elements);

void fix_qmult(fixp_t *q1, fixp_t *q2, fixp_t *qout);
void fill_fixp_array(double *nums, int num_elements, fixp_t *array, int32_t scale);
void create_fixp_array(int32_t *nums, int num_elements, fixp_t *array, int32_t scale);
void fix_dynamics_predict(fixp_t *x, fixp_t *u, fixp_t *xout);
void fix_sliding_mode_update(fixp_t *x, fixp_t *w_meas, fixp_t *xout);


#endif	/* ESTIMATOR_H */

