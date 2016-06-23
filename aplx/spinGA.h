#ifndef SPINGA_H
#define SPINGA_H

#define MAJOR_VERSION		0
#define MINOR_VERSION		3

#include <spin1_api.h>
#include <stdfix.h>
#include <math.h>

#define TIMER_TICK_PERIOD_US 	1000000
#define REAL                    accum
#define REAL_CONST(x)           x##k
#define RAND_MAX_UINT           0xFFFFFFFF
#define RAND_MAX_UINT16         0xFFFF

#define PI                      3.14159265359

// basic priority definition
#define PRIORITY_MCPL           -1
#define PRIORITY_DMA            0
#define PRIORITY_TIMER          1
#define PRIORITY_SDP            2
#define PRIORITY_NORMAL         3
#define PRIORITY_SCHEDULED      3
#define PRIORITY_LOW            4
#define PRIORITY_LOWEST         5

// basic SDP mechanism
#define SDP_PORT            	1
#define SDP_TAG                 1
#define SDP_UDP_PORT            20000

// basic MCPL key allocation
#define MCPL_BCAST_PING				0xbca50001
#define MCPL_BCAST_NCHRGEN			0xbca50002	// number of chromosomes and genes
#define MCPL_BCAST_MINVAL			0xbca50003
#define MCPL_BCAST_MAXVAL			0xbca50004
#define MCPL_BCAST_EOC				0xbca50005	// broadcast end of config
#define MCPL_BCAST_CHR_ADDR			0xbca50006
#define MCPL_BCAST_OBJEVAL			0xbca50007
#define MCPL_BCAST_OBJVAL_ADDR		0xbca50008
#define MCPL_BCAST_FITVAL_ADDR		0xbca50009
#define MCPL_BCAST_UPDATE_CHR_CHUNK	0xbca5000A	// ask worker to fetch chr from sdram
#define MCPL_BCAST_TFITNESS         0xbca5000B
#define MCPL_BCAST_PROB_ADDR        0xbca5000C
#define MCPL_BCAST_NEWCHR_ADDR		0xbca5000D
#define MCPL_BCAST_CRATE			0xbca5000E	// crossover rate
#define MCPL_BCAST_EMIGRANTS		0xbca5FFFF	// send emmigrants out
#define MCPL_2LEAD_PING_RPT			0x1ead0001	// ping reply/report
#define MCPL_2LEAD_INITCHR_RPT		0x1ead0002	// init population complete
#define MCPL_2LEAD_OBJEVAL_RPT		0x1ead0003	// objVal complete
#define MCPL_2LEAD_PROB_RPT         0x1ead0004  // prob complete
#define MCPL_2LEAD_FITVAL			0x1ead0005	// send fitness value to leadAp
#define MCPL_2LEAD_OBJVAL			0x1ead0006	// the lower part is chromosome index
#define MCPL_2LEAD_BEST_CHR			0x1ead0007	// send the best chromosome to leadAp

// key contain values?
#define MCPL_BCAST_CROSS_PAR		0xbca60000	// for crossover parents info

// for direct/individual base
#define MCPL_DIRECT_BASE_MASK	0xff000000	// then we can carry more information in the key
											// then the maximum number of workers is 255

// memory stuffs
#define SDRAM_TAG_CHR			0xc
#define SDRAM_TAG_NEWCHR		0xd
#define SDRAM_TAG_OBJVAL		0xb
#define SDRAM_TAG_FITVAL		0xf
#define SDRAM_TAG_PROB          0xa

// basic DMA mechanisme
#define DMA_TAG_CHRCHUNK_W		0xc1
#define DMA_TAG_CHRCHUNK_R		0xc0
#define DMA_TAG_OBJVAL_W		0xb1
#define DMA_TAG_FITVAL_W		0xf1
#define DMA_TAG_PROBVAL_W       0xa1
#define DMA_TAG_GETPROB_R       0xa0

// does leadAp also works as a worker?
#define LEADAP_AS_WORKER		FALSE

// basic GA setup
#define DEF_MAX_CHR				5000	// default number of individuals (population size)
#define DEF_N_CHR               100
#define NUM_CORES_USED			17		// assuming that all 16 cores are available
#define DEF_MAX_ITER			1000
#define DEF_MAX_GENE			256
#define MAX_ELITE				2
#define MAX_EMIGRANT			6		// == number of external links (will use MCPL to broadcast chromosomes)
// the following modes will be carried in the lower part of MCPL key (high part is coreID)
#define WID_MODE				0		// for collecing wID
#define CP_MODE_SINGLE			1		// for crossover operation
#define CP_MODE_DOUBLE			2		// for crossover operation
#define CP_MODE_UNIFORM			3		// for crossover operation
#define CP_MODE_UNIFORM_RATIO	50		// ratio (in percentage) for uniform crossover mechanism

// simulation parameters
typedef struct Params {
	ushort nChr;						// number of chromosomes
	ushort nGen;						// number of genes in a chromosomes
	REAL minGenVal;
	REAL maxGenVal;
	REAL upperThreshold;				// for checking if solution has been found
	REAL lowerThreshold;
	REAL cRate;							// crossover rate
	uint nIter;
	// for elitism:
	ushort nElite;						// if 0, then no elitism
	ushort eliteID[MAX_ELITE];
} gaParams_t;

// worker info
typedef struct w_info {
	// info held by leadAp:
	ushort wID[18];						//in future it can be increased up to 255
	// info held by all cores:
	uchar lp;							// load pointer, will be used for load balancing (distributing load
	ushort tAvailable;					// how many workers? should be initialized to 1
	uchar subBlockID;					// worker'sID: this is specific for each worker
										// for GA ops (crossover, etc) accross workers
	ushort chrIdxStart;
	ushort chrIdxEnd;
	ushort nChrChunk;					// number of chromosomes per worker
	uint szChrChunk;					// in bytes
} w_info_t;

// basic GA parameters:
gaParams_t gaParams;					// all cores will have this

// other stuffs
uint64 tic;
uint64 toc;
static uint *chr = NULL;                // location of current chromosomes in sdram
static uint *newChr = NULL;
static uint *chrChunk = NULL;			// located in DTCM, for each worker
uint tNewGen;					// number of newly generated chromosomes (new generation)
uint myCoreID;
w_info_t workers;			// will be held by leadAp
sdp_msg_t *reportMsg;
uchar initPopCntr;	// to count, how many workers have finished pop init
uchar objValCntr;
uchar probValCntr;
ushort bestChr[17];						// collection of best Chromosomes
uchar nBestChr;
volatile uchar initPopDone;
volatile uchar objEvalDone;
volatile uchar probEvalDone;
volatile uchar dmaGetProbDone;
uint gaIter;
volatile int SolutionFound;		// -1 means not found! otherwise, it is the chr index
static REAL *objVal = NULL;		// each worker has its own objVal[]
static REAL *fitVal = NULL;		// each worker has its own fitVal[]
static REAL *prob = NULL;
static REAL *allObjVal = NULL;	// this is located in sdram, for all
static REAL *allFitVal = NULL;	//
static REAL *allProb = NULL;
static REAL *cdf = NULL;
static REAL *rv = NULL;						// holds random numbers
ushort nRV;									// total number of currently generated random numbers (rv)
static uint *selectedChr = NULL;			// holds roullette wheel result
REAL TFitness;

/*------------------------------ forward declaration ------------------------------*/
// standard spinnaker mechanism
void hTimer(uint tick, uint Unused);
void hSDP(uint mBox, uint port);
void hDMADone(uint tid, uint tag);
void hMCPL(uint key, uint payload);
void poolWorkers(uint arg0, uint arg1);
void computeWload(uint arg0, uint arg1);
void getChrChunk(uint arg0, uint arg1);
void initSDP();
void initRouter();
void execCross(uint arg0, uint arg1);
void stopGA(uint arg0, uint arg1);
void reportFinal();

// GA core engine
void initMemGA();
void releaseMemGA();
void objEval(uint arg0, uint arg1);
extern REAL objFunction(ushort nGene, uint genes[]);
//void runGA(uint iter);
void runGA(uint arg0, uint arg1);
void computeProb(uint arg0, uint arg1);
void defRouletteWheel(REAL *preComputedCDF, REAL *generatedRV, uint numOfRV, REAL *selectedChromosomes);
void defOnePointCross(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes);
void defTwoPointsCross(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes);
void defUniformCross(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes);
void defMutation(gaParams_t p, uint *chromosomes);
void doEvaluation(uint arg0, uint arg1);
// give a freedom for user to use their selection mechanism
// here we provide rv_addr and nRV, should be needed
void doSelection(REAL *preComputedCDF, REAL *generatedRV, uint numOfRV, REAL *selectedChromosomes);
void doCrossover(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes);
void doMutation(gaParams_t p, uint *chromosomes);
void cross(ushort p[2], uint mode);
int searchForSolution();

// Use mersenne-twister random generator
extern void init_genrand(unsigned long s);
extern uint genrand_int32(void);
extern REAL genrand_fixp(REAL minVal, REAL maxVal, uint seed);
extern ushort genrand_ushort(ushort min, ushort max, uint seed);

// user defined models
extern uint encodeGen(REAL rVal);
extern REAL decodeGen(uint gen);
void initPopulation();

// helper functions and debugging:
void generateRV();
uint bin2gray(uint num);
uint gray2bin(uint num);
REAL roundr(REAL inVal);
void showProbValues(uint arg0, uint arg1);
void showChromosomes(uint arg0, uint arg1);
void showObjFitValues(uint arg0, uint arg1);
uint getUintFromREAL(REAL r);
REAL getREALFromUint(uint u);
ushort getChrBitLength();	// get chromosome bit length

//---------------------- Let's have a simple test case -----------------------------
// for simple simulation using Rastrigin
#define DEF_RASTRIGIN_ORDER			2		// the dimension of the function (1..10)
#define DEF_RASTRIGIN_MINVAL		REAL_CONST(-6.0)
#define DEF_RASTRIGIN_MAXVAL		REAL_CONST(6.0)

void selfSimulation();	// for testing

#endif // SPINGA_H

