
/*
 * Author: Hersh Sanghvi
 */

#include "estimator.h"

// TODO: convert all to fixed point

int32_t fix_sign_approx(int32_t number, int32_t scale) {
    int32_t thresh = (int32_t)(0.5 * (1 << scale));
    if (number >= thresh) {
        return 1;
    } else if (number< thresh && number> -thresh) {
        return number;
    } else if (number<= -thresh) {
        return -1;
    } else {
        return 0;
    }
}

fixp_t cos_approx(fixp_t x) {
    if (x.val < 0) {
        x.val = -x.val;
    }
    x.val = (HALF_PI_SCALED) + x.val;
    x.val = (x.val + PI_SCALED) % (PI_SCALED << 1) - PI_SCALED;
    int32_t result1 = (int64_t)x.val * (int64_t)(PI_SCALED - x.val)/(1 << x.scaling);
    int32_t result = 16 * (int64_t)result1 * (1 << x.scaling) / (5 * PI_SQ_SCALED - 4 * result1);
    return (fixp_t) {result, x.scaling};
}

fixp_t sin_approx(fixp_t x) {
    fixp_t new_x = (fixp_t){x.val - HALF_PI_SCALED, x.scaling};
    return cos_approx(new_x);
}

fixp_t fix_sqrt_approx(fixp_t num) {
    int32_t guess = num.val/2;  // don't need fixed point division
    int32_t guess_prev = num.val;
    int32_t epsilon_scaled = (int32_t) (EPSILON * (1 << num.scaling));
    while (guess - guess_prev > epsilon_scaled || guess_prev - guess > epsilon_scaled) {
        guess_prev = guess;
        guess = (guess + ((int64_t)num.val * (1 << num.scaling))/guess)/2;
    }
    return (fixp_t){guess, num.scaling};
}

// Assumes all elements of vector have the same 
// scaling factor, and the same output scaling
// factor is used.
fixp_t fix_l2norm(fixp_t *vector, int num_elements) {
    int32_t sum = 0; 
    for (int i = 0; i < num_elements; i++) {
        sum = sum + ((int64_t)vector[i].val * (int64_t)vector[i].val) / (1 << 16); 
    }
    return (fixp_t){sum, vector[0].scaling};
}

void fix_qmult(fixp_t *q1, fixp_t *q2, fixp_t *qout) {
    int32_t scaling = q1[0].scaling;
    qout[0].val = ((int64_t)q1[0].val * (int64_t)q2[0].val - (int64_t)q1[1].val * (int64_t)q2[1].val - 
                    (int64_t)q1[2].val * (int64_t)q2[2].val - (int64_t)q1[3].val * (int64_t)q2[3].val)/(1 << scaling);

    qout[0].scaling = scaling;
    qout[1].val = ((int64_t)q1[0].val * (int64_t)q2[1].val + (int64_t)q1[1].val * (int64_t)q2[0].val + 
                    (int64_t)q1[2].val * (int64_t)q2[3].val - (int64_t)q1[3].val * (int64_t)q2[2].val)/(1 << scaling);
    qout[1].scaling = scaling;

    qout[2].val = ((int64_t)q1[0].val * (int64_t)q2[2].val - (int64_t)q1[1].val * (int64_t)q2[3].val + 
                    (int64_t)q1[2].val * (int64_t)q2[0].val + (int64_t)q1[3].val * (int64_t)q2[1].val)/(1 << scaling);

    qout[2].scaling = scaling;
    qout[3].val = ((int64_t)q1[0].val * (int64_t)q2[3].val + (int64_t)q1[1].val * (int64_t)q2[2].val - 
                    (int64_t)q1[2].val * (int64_t)q2[1].val + (int64_t)q1[3].val * (int64_t)q2[0].val)/(1 << scaling);
    
    qout[3].scaling = scaling;
}

void fill_fixp_array(double *nums, int num_elements, fixp_t *array, int32_t scale) {
    for (int i = 0; i < num_elements; i++) {
        array[i] = (fixp_t) {nums[i] * (1 << scale), scale};
    }
}

void create_fixp_array(int32_t *nums, int num_elements, fixp_t *array, int32_t scale) {
    for (int i = 0; i < num_elements; i++) {
        array[i] = (fixp_t) {nums[i], scale};
    }
}

void fix_dynamics_predict(fixp_t *x, fixp_t *u, fixp_t *xout) {
    int32_t scale = x[0].scaling;
    int32_t delta_t = 0.001 * (1 << scale);
    fixp_t qout[4];
    fixp_t q2[4];

    int32_t temp[4];
    // angular velocity and tail angular momentum
    temp[0] = x[4].val + (int64_t)delta_t * (-u[0].val - (int64_t)x[7].val * (int64_t)x[6].val/(1 << scale))/(1 << scale);
    temp[1] = x[5].val + (int64_t)delta_t * (int64_t)(-u[1].val)/(1 << scale);
    temp[2] = x[6].val + (int64_t)delta_t * (-u[2].val - (int64_t)x[7].val * (int64_t)x[4].val/(1 << scale))/(1 << scale);
    temp[3] = x[7].val + (int64_t)delta_t * (int64_t)(-u[1].val)/(1 << scale);
    create_fixp_array(temp, 4, xout+4, scale);

    fixp_t norm = fix_l2norm(x+4, 3);
    if (norm.val < EPS_SCALED) {
        norm.val = EPS_SCALED;
    }

    fixp_t term = {(int64_t)norm.val * (int64_t)delta_t/(2 << scale), scale};
    fixp_t sin_term = sin_approx(term);
    sin_term.val = (int64_t)sin_term.val * (1 << scale)/norm.val;

    q2[0] = cos_approx(term);
    for (int i = 1; i < 4; i++) {
        q2[i] = (fixp_t){(int64_t)x[i+3].val * (int64_t)sin_term.val/(1 << scale), scale};
    }
    fix_qmult(x, q2, qout);
    
    for (int i = 0; i < 4; i++) {
        xout[i] = qout[i];
    }
}

void fix_sliding_mode_update(fixp_t *x, fixp_t *w_meas, fixp_t *xout) {
    // the quaternion and tail terms do not change
    xout[0] = x[0];
    xout[1] = x[1];
    xout[2] = x[2];
    xout[3] = x[3];
    xout[7] = x[7];
    
    int32_t scale = x[0].scaling;

    // incorporate error term for angular velocity
    for (int i = 4; i < 7; i++) {
        xout[i] = (fixp_t){x[i].val - DAMPING_COEFF * (x[i].val - w_meas[i-4].val) - 
                    SLIDING_COEFF * fix_sign_approx(x[i].val - w_meas[i-4].val, scale), scale};
    }
}
