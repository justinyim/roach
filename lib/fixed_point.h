/* 
 * File:   fixed_point.h
 * Author: Hersh Sanghvi
 *
 * Created on February 5, 2019, 8:56 PM
 */

#ifndef FIXED_POINT_H
#define	FIXED_POINT_H

/* 
    The value represented by this struct is 
    val / (2^scaling)
*/
typedef struct fixed_point {
    int32_t val;
    int32_t scaling;
} fixp_t;

fixp_t fix_add(fixp_t a, fixp_t b) {
    fixp_t c = {a.val + b.val, a.scaling};
    return c;
}

fixp_t fix_sub(fixp_t a, fixp_t b) {
    fixp_t c = {a.val - b.val, a.scaling};
    return c;
}

fixp_t fix_mul(fixp_t a, fixp_t b) {
    int32_t val = ((int64_t)a.val * (int64_t)b.val) / (1 << a.scaling);
    fixp_t c = {val, a.scaling};
    return c;
}

fixp_t fix_div(fixp_t a, fixp_t b) {
    int32_t val = ((int64_t)a.val * (1 << a.scaling))/b.val;
    fixp_t c = {val, a.scaling};
    return c;
}

#endif	/* FIXED_POINT_H */

