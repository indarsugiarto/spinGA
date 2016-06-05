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
#define MCPL_2LEAD_RPT          0x1ead0001

// basic GA setup
#define DEF_N_CHR               100
#define DEF_RASTRIGIN_ORDER			2		// the dimension of the function (1..10)
#define NUM_CORES_USED				16		// assuming that all 16 cores are available
#define TOTAL_GENES					DEF_N_CHR_PER_CORE * DEF_RASTRIGIN_ORDER * NUM_CORES_USED
#define TOTAL_CHROMOSOMES			DEF_N_CHR_PER_CORE * NUM_CORES_USED
#define MAX_ITER					100
#define MIN_PARAM					-5.12
#define MAX_PARAM					5.12

// worker info
typedef struct w_info {
    uchar wID[17];			// this is the coreID, maximum worker is 17
    //uchar subBlockID[17];	// just a helper, this maps subBlockID of workers
    uchar tAvailable;		// how many workers? should be initialized to 1
} w_info_t;


uint *chr = NULL;                      // location of chromosomes in sdram
uint nChr;                      // number of chromosomes
uint nGen;                      // number of genes in a chromosomes
uint myCoreID;
w_info_t workers;			// will be held by leadAp
sdp_msg_t *reportMsg;
uint tik = 0;
uint myCoreID;
uint myCellID;
uint genCntr = 0;
uint prevCellPacketCntr = 0;
uint iter = 1;

#define TIMER_TICK_PERIOD		1000000

/* GA parameters */
uint Chromosomes[DEF_N_CHR_PER_CORE * NUM_CORES_USED][DEF_RASTRIGIN_ORDER] = {0};
uint collectedGenes;					// a counter to count how many chromosomes are collected so far
                                        // if Chromosomes buffer is full, then start the calculation
REAL m, b;								// linear regression parameters


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
void initGA();
void objEval();
void getRegParam(REAL *m, REAL *b);
uint encodeGen(REAL rVal);
REAL decodeGen(uint gen);
void showMyChromosomes();
void checkMyTurn(uint cellID);
void bcastMyChromosomes(uint arg0, uint arg1);
// Use mersenne-twister random generator
extern void init_genrand(unsigned long s);
extern uint genrand_int32(void);
extern REAL genrand_fixp(REAL minVal, REAL maxVal, uint seed);
// helper functions:
uint bin2gray(uint num);
uint gray2bin(uint num);
REAL roundr(REAL inVal);

#endif // SPINGA_H

