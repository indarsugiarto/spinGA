#include "spinGA.h"

/*_______________________ Helper & Debuggin functions _________________________*/

inline uint getUintFromREAL(REAL r)
{
	uint u;
	sark_memcpy((void *)&u, (void *)&r, sizeof(uint));
	return u;
}

inline REAL getREALFromUint(uint u)
{
	REAL r;
	sark_mem_cpy((void *)&r, (void *)&u, sizeof(uint));
	return r;
}

inline void generateRV()
{
	if(nRV < gaParams.nChr) {
		rv[nRV] = genrand_fixp(0.0, 1.0, 0);
		nRV++;
	}
}

// use showChromosomes() to display current chromosomes
void showChromosomes(uint arg0, uint arg1)
{
	uint c,g;	// chromosomes and genes counter
	REAL G;		// the gene in real-valued
	uint eG;	// encoded gene
	uint *pChr = chrChunk;

	// for workers, it displays chromosomes in chrChunk
	if(!leadAp) {
		io_printf(IO_BUF, "Population in wID-%d:\n-------------------\n", workers.wID);
		for(c=0; c<workers.nChrChunk; c++) {
			io_printf(IO_BUF, "Chr-%d: ", workers.chrIdxStart+c);
			for(g=0; g<gaParams.nGen; g++) {
				eG = *pChr; pChr++;
				G = decodeGen(eG);
				io_printf(IO_STD, "[ G = 0x%x = %k, gg = 0x%x ] ", G, G, eG);
			}
			io_printf(IO_STD, "\n");
		}
	}
	// for leadAp, will display all chromosomes in chr
	else {
		pChr = chr;
		io_printf(IO_BUF, "Total populations\n----------------------\n");
		for(c=0; c<gaParams.nChr; c++) {
			io_printf(IO_BUF, "Chr-%d: ", c);
			for(g=0; g<gaParams.nGen; g++) {
				eG = *pChr; pChr++;
				G = decodeGen(eG);
				io_printf(IO_STD, "[ G = 0x%x = %k, gg = 0x%x ] ", G, G, eG);
			}
			io_printf(IO_BUF, "\n");
		}
	}
}

// show objective and fitness values
void showObjFitValues(uint arg0, uint arg1)
{
	uint c;

	io_printf(IO_BUF, "Objective and Fitness values:\n-----------------------------------\n");
	if(!leadAp) {
		for(c=0; c<workers.nChrChunk; c++) {
			io_printf(IO_BUF, "Chr-%d: ov = %k, fv = %k\n",
					  workers.chrIdxStart+c, objVal[c], fitVal[c]);
		}
	}
	else {
		for(c=0; c<gaParams.nChr; c++) {
			io_printf(IO_BUF, "Chr-%d: ov = %k, fv = %k\n",
					  c, allObjVal[c], allFitVal[c]);
		}
		io_printf(IO_STD, "TFitness = %k\n", TFitness);
	}
}

// show probability values
void showProbValues(uint arg0, uint arg1)
{
	uint c;

	io_printf(IO_STD, "Probability values:\n-------------------\n");
	if(!leadAp) {
		for(c=0; c<workers.nChrChunk; c++) {
			io_printf(IO_BUF, "p(Chr-%d): %k\n",
					  workers.chrIdxStart+c, prob[c]);
		}
	}
	else {
		for(c=0; c<gaParams.nChr; c++) {
			io_printf(IO_BUF, "p(Chr-%d): %k\n -> cdf = %k",
					  c, allProb[c], cdf[c]);
		}
}


REAL roundr(REAL inVal)
{
	uint base = (uint)inVal;
	uint upper = base + 1;
	REAL conver = inVal+REAL_CONST(0.5);
	if((uint)conver == base)
		return (REAL)base;
	else
		return (REAL)upper;
}

uint bin2gray(uint num)
{
	return num ^ (num >> 1);
}

uint gray2bin(uint num)
{
	num = num ^ (num >> 16);
	num = num ^ (num >> 8);
	num = num ^ (num >> 4);
	num = num ^ (num >> 2);
	num = num ^ (num >> 1);
	return num;
}

inline ushort getChrBitLength()
{
	return nGen * 32;
}

// selfSimulation() is just for providing an alternative for GA configuration
// in final version, the GA configuration (all variables) will be sent via SDP from a host
// NOTE: only leadAp has this selfSimulation()
void selfSimulation()
{
	// first, imitate host-config
	gaParams.nChr = DEF_N_CHR;
	gaParams.nGen = DEF_RASTRIGIN_ORDER;
	gaParams.minGenVal = DEF_RASTRIGIN_MINVAL;
	gaParams.maxGenVal = DEF_RASTRIGIN_MAXVAL;
	gaParams.nIter = DEF_MAX_ITER;
	gaParams.nElite = 0;	// no elitism
	gaParams.lowerThreshold = REAL_CONST(0.0);
	gaParams.upperThreshold = REAL_CONST(0.0001);
	gaParams.cRate = 0.2;

	// then call GA simulation
	spin1_schedule_callback(runGA,0,0,PRIORITY_NORMAL);
}

/*_______________________________________________ Helper & Debugging functions _*/

