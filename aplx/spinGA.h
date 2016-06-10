#ifndef SPINGA_H
#define SPINGA_H

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
#define MCPL_BCAST_NCHRGEN			0xbca50002
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
#define MCPL_2LEAD_PING_RPT			0x1ead0001	// ping reply/report
#define MCPL_2LEAD_INITCHR_RPT		0x1ead0002	// init population complete
#define MCPL_2LEAD_OBJEVAL_RPT		0x1ead0003	// objVal complete
#define MCPL_2LEAD_PROB_RPT         0x1ead0004  // prob complete

// key with values
#define MCPL_2LEAD_OBJVAL		0x00a10000	// the lower part is chromosome index
#define MCPL_2LEAD_FITVAL		0x00a20000
#define MCPL_2LEAD_PROBVAL      0x00030000
#define MCPL_BCAST_CROSS_PAR	0x00040000	// for crossover parents info
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
#define DEF_MAX_CHR				5000
#define DEF_N_CHR               100
#define NUM_CORES_USED			17		// assuming that all 16 cores are available
#define DEF_MAX_ITER			1000
#define DEF_MAX_GENE			256
#define MAX_ELITE				2
// the following modes will be carried in the lower part of MCPL key (high part is coreID)
#define WID_MODE				0		// for collecing wID
#define CP_MODE_SINGLE			1		// for crossover operation
#define CP_MODE_DOUBLE			2		// for crossover operation
#define CP_MODE_UNIFORM			3		// for crossover operation
#define CP_MODE_UNIFORM_RATIO	50		// ratio (in percentage) for uniform crossover mechanism

// for simple simulation using Rastrigin
#define DEF_RASTRIGIN_ORDER			2		// the dimension of the function (1..10)
#define DEF_RASTRIGIN_MINVAL		(REAL_CONST(-6.0))
#define DEF_RASTRIGIN_MAXVAL		(REAL_CONST(6.0))

// worker info
typedef struct w_info {
	//ushort wID[NUM_CORES_USED];			// this is the coreID, maximum worker is 17
	ushort wID[18];			// this is the coreID, maximum worker is 17,
							//in future it can be increased up to 255
    //uchar subBlockID[17];	// just a helper, this maps subBlockID of workers
	ushort tAvailable;		// how many workers? should be initialized to 1
	uchar lp;				// load pointer, will be used for load balancing (distributing load
							// for GA ops (crossover, etc) accross workers
} w_info_t;

static uint *chr = NULL;                      // location of current chromosomes in sdram
static uint *newChr = NULL;
static uint *chrChunk = NULL;			// located in DTCM, for each worker
uint szChrChunk;
uint nChrChunk;
uint nChr;                      // number of chromosomes
uint nGen;                      // number of genes in a chromosomes
uint tNewGen;					// number of newly generated chromosomes (new generation)
REAL minGenVal;
REAL maxGenVal;
uint myCoreID;
ushort chrIdxStart;
ushort chrIdxEnd;
w_info_t workers;			// will be held by leadAp
sdp_msg_t *reportMsg;
uchar initPopCntr;	// to count, how many workers have finished pop init
uchar objValCntr;
uchar probValCntr;
uint nIter;
volatile uchar initPopDone;
volatile uchar objEvalDone;
volatile uchar probEvalDone;
volatile uchar dmaGetProbDone;
static REAL *objVal = NULL;		// each worker has its own objVal[]
static REAL *fitVal = NULL;		// each worker has its own fitVal[]
static REAL *prob = NULL;
static REAL *allObjVal = NULL;	// this is located in sdram
static REAL *allFitVal = NULL;	//
static REAL *allProb = NULL;
static REAL *cdf = NULL;
static ushort *selectedChr = NULL;			// holds roullette wheel result
REAL *probBuf;
REAL TFitness;
REAL cRate;						// crossover rate
// for elitism:
ushort nElite;					// if 0, then no elitism
ushort eliteID[MAX_ELITE];

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

// GA core engine
void initMemGA();
void objEval(uint arg0, uint arg1);
extern REAL objFunction(ushort nGene, uint genes[]);
void runGA(uint iter);
void computeProb(uint arg0, uint arg1);
void defRouletteWheel(uint arg0, uint arg1);
void doSelection(uint arg0, uint arg1);		// give a freedom for user to use their selection mechanism
void doCrossover(uint cRate, uint selChr);
void cross(ushort p[2], uint mode);

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
uint bin2gray(uint num);
uint gray2bin(uint num);
REAL roundr(REAL inVal);
void showProbValues(uint arg0, uint arg1);
void showChromosomes(uint arg0, uint arg1);
void showFitValues(uint arg0, uint arg1);
ushort getChrBitLength();	// get chromosome bit length



#endif // SPINGA_H

