// Here the user may define their own model
// "Virtual" functions are:
// encodeGen(), decodeGen()

/* This program will find the minimum value of a n-order rastrigin function.
 * Every core will behave as a cell that carries several chromosomes.
 * For visualization:
 * I created a simple matlab in ~/Projects/Matlab/try_rastrigin.m
 * */


#include "spinGA.h"
#include <stdfix.h>
#include <math.h>


/* encodeGen() and decodeGen() use linear regression to convert from real value to integer and vice versa
 * */
uint encodeGen(REAL rVal)
{
    //Use y = mx + b, where m = (y_max - y_min) / (x_max-x_min): y_max = 65535, y_min = 0, x_max = MAX_PARAM, x_min = MIN_PARAM
    return((uint)roundr(m*rVal+b));
}

REAL decodeGen(uint gen){
    //Use x = (y-b)/m
    REAL r = ((REAL)gen - b) / m;
    return r;
}

