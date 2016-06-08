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
#define MCPL_BCAST_PING         0xbca50001
#define MCPL_2LEAD_PING_RPT		0x1ead0001	// ping reply/report
#define MCPL_2LEAD_INITCHR_RPT	0x1ead0002	// operation completion init population
#define MCPL_BCAST_NCHRGEN	0xbca50002
#define MCPL_BCAST_MINVAL	0xbca50003
#define MCPL_BCAST_MAXVAL	0xbca50004
#define MCPL_BCAST_EOC		0xbca50005	// broadcast end of config

// basic DMA mechanisme
#define DMA_TAG_CHRCHUNK_W		0xc1

// does leadAp also works as a worker?
#define LEADAP_AS_WORKER		FALSE

// basic GA setup
#define DEF_N_CHR               100
#define NUM_CORES_USED			16		// assuming that all 16 cores are available
#define DEF_MAX_ITER			1000

// for simple simulation using Rastrigin
#define DEF_RASTRIGIN_ORDER			2		// the dimension of the function (1..10)
#define DEF_RASTRIGIN_MINVAL		(REAL_CONST(-6.0))
#define DEF_RASTRIGIN_MAXVAL		(REAL_CONST(6.0))

// worker info
typedef struct w_info {
	ushort wID[NUM_CORES_USED];			// this is the coreID, maximum worker is 17
    //uchar subBlockID[17];	// just a helper, this maps subBlockID of workers
	ushort tAvailable;		// how many workers? should be initialized to 1
} w_info_t;

static uint *chr = NULL;                      // location of current chromosomes in sdram
static uint *chrChunk = NULL;			// located in DTCM, for each worker
uint szChrChunk;
uint nChr;                      // number of chromosomes
uint nGen;                      // number of genes in a chromosomes
REAL minGenVal;
REAL maxGenVal;
uint myCoreID;
ushort chrIdxStart;
ushort chrIdxEnd;
w_info_t workers;			// will be held by leadAp
sdp_msg_t *reportMsg;
static uint initPopCntr = 0;


/*------------------------------ forward declaration ------------------------------*/
// standard spinnaker mechanism
void hTimer(uint tick, uint Unused);
void hSDP(uint mBox, uint port);
void hDMADone(uint tid, uint tag);
void hMCPL(uint key, uint payload);
void poolWorkers(uint arg0, uint arg1);
void computeWload(uint arg0, uint arg1);
void initSDP();
void initRouter();

// GA core engine
void initMemGA();
void objEval();
void showMyChromosomes();
// Use mersenne-twister random generator
extern void init_genrand(unsigned long s);
extern uint genrand_int32(void);
extern REAL genrand_fixp(REAL minVal, REAL maxVal, uint seed);
// user defined models
extern uint encodeGen(REAL rVal);
extern REAL decodeGen(uint gen);
void initPopulation();
// helper functions and debugging:
uint bin2gray(uint num);
uint gray2bin(uint num);
REAL roundr(REAL inVal);
void reportInitChr(uint arg0, uint arg1);

#endif // SPINGA_H

