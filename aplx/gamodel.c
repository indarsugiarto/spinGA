// Here the user may define their own model
// "Virtual" functions are:
// encodeGen(), decodeGen(), initPopulation
//
// encodeGen() will convert a "crisp" value (which is of type REAL) into ga value
// (which will be stored as uint32)
// decodeGen() will convert a uint32 ga value back into a "crisp" REAL value

/* This program will find the minimum value of a n-order rastrigin function.
 * Every core will behave as a cell that carries several chromosomes.
 * For visualization:
 * I created a simple matlab in ~/Projects/Matlab/try_rastrigin.m
 * */

#include <spin1_api.h>
#include "spinGA.h"
#include <stdfix.h>
#include <math.h>

REAL m, b;								// linear regression parameters



//------------------- Example template than can be modified by user ------------------
// define your own selection mechanism here
void myOwnSelection(REAL *preComputedCDF, REAL *generatedRV, uint numOfRV, REAL *selectedChromosomes)
{

}

void myOwnCrossover(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes)
{

}

void myOwnMutation(gaParams_t p, uint *chromosomes)
{

}

//-------------- make sure that this section lay on top of this c file ---------------

/* in this example model, encodeGen() and decodeGen()
 * don't perform any specific processing (just pass the value)
 * */

/*
uint encodeGen(ushort chrID, ushort genID, REAL rVal)
{
    //Use y = mx + b, where m = (y_max - y_min) / (x_max-x_min): y_max = 65535, y_min = 0, x_max = MAX_PARAM, x_min = MIN_PARAM
    return((uint)roundr(m*rVal+b));
}

// if chrID and genID is provided
REAL decodeGen(ushort chrID, ushort genID, uint gen){
    //Use x = (y-b)/m
    REAL r = ((REAL)gen - b) / m;
    return r;
}

void getRegParam(REAL *m, REAL *b)
{
	REAL M, B;
	REAL y_min = REAL_CONST(0);
	REAL y_max = REAL_CONST(65535);					// There'll be 65536 steps
	REAL x_min = MIN_PARAM;
	REAL x_max = MAX_PARAM;
	M = (y_max - y_min) / (x_max-x_min);
	B = (y_max) - M*x_max;							// if symmetric, b should be 0
	*m = M;
	*b = B;
}

*/

uint encodeGen(REAL rVal)
{
	uint r;
	uint *ptr = (uint *)&rVal;
	r = *ptr;										// basically getUintFromREAL()
	return r;
}

REAL decodeGen(uint gen)
{
	REAL r;
	REAL *ptr = (REAL *)&gen;
	r = *ptr;										// basically getREALFromUint()
	return r;
}

// doSelection() is a core part and will be called by runGA()
// here, you can change the selection mechanism
// one default selection mechanism is defRouletteWheel()
void doSelection(REAL *preComputedCDF, REAL *generatedRV, uint numOfRV, REAL *selectedChromosomes)
{
	defRouletteWheel(preComputedCDF, generatedRV, numOfRV, selectedChromosomes);
	// optionally, user can create their own selection mechanism
	// myOwnSelection(REAL *generatedRV, uint numOfRV, REAL *selectedChromosomes);
}


// Enable option below if a better chromosome has a bigger value
//#define BIGGER_IS_BETTER

// Since objective function is app-dependant,
// user must define it here.
// eg. in rastrigin 2nd order, the objFunction is
// f(x1,x2) = 10*2 + ((x1^2 - 10*cos(2*pi*x1)) + (x2^2 - 10*cos(2*pi*x2)))
#define USE_ABS_OBJECTIVE TRUE
REAL objFunction(ushort nGene, uint genes[])
{
	REAL objVal = 10.0 * nGene;
	REAL ciVal = REAL_CONST(10.0);
	REAL g;
	for(ushort i=0; i<nGene; i++) {
		g = decodeGen(genes[i]);
		objVal += (g*g - ciVal*cosf(2*PI*g));
	}
	// in rastrigin example, we might need to use absolute values
#if (USE_ABS_OBJECTIVE==TRUE)
	if(objVal < 0.0)
		objVal *= -1.0;
#endif
	return objVal;
}

// doCrossover is a core part and will be called by runGA()
// here user can change the crossover mechanism
// or use provided/default crossover mechanism:
// - defOnePointCross()
// - defTwoPointsCross() -> not yet implemented!!!
// - defUniformCross()	-> not yet implemented!!!
void doCrossover(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes)
{
	defOnePointCross(p, selectedChromosomesIdx, chromosomes);
	// or, use your own crossover mechanism:
	// myOwnCrossover(p, selectedChromosomesIdx, chromosomes);
}

// doCrossover is a core part and will be called by runGA()
// here, user can change the mutation mechanism
// or use default mutation mechanism
void doMutation(gaParams_t p, uint *chromosomes)
{
	defMutation(p, chromosomes);
	// or, use your own mutation mechanism:
	// myOwnMutation(p, chromosomes)
}

