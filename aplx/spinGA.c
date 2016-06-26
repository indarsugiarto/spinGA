/* Coba lihat yang pernah dilakukan di:
 * try.with.stdfixpoint.in.embarasingly.parallel
 * */
#include <spin1_api.h>
#include "spinGA.h"

// it is safer to runGA() as a callback function
void runGA(uint arg0, uint arg1)
{
	tic = sv->clock_ms;		// sv->clock_ms is milliseconds since boot
	// first, initialize memory
	// inside initMemGA(), chr address is distributed!
	initMemGA();

	// second, distribute ga parameters to workers
	spin1_send_mc_packet(MCPL_BCAST_NCHRGEN, (gaParams.nChr << 16) + gaParams.nGen, WITH_PAYLOAD);
	spin1_send_mc_packet(MCPL_BCAST_MINVAL, getUintFromREAL(gaParams.minGenVal), WITH_PAYLOAD);
	spin1_send_mc_packet(MCPL_BCAST_MAXVAL, getUintFromREAL(gaParams.maxGenVal), WITH_PAYLOAD);
	spin1_send_mc_packet(MCPL_BCAST_CRATE, getUintFromREAL(gaParams.cRate), WITH_PAYLOAD);

	// third: send notification to start initializing population
	nRV = 0;
	initPopDone = FALSE;
	initPopCntr = 0;
	spin1_send_mc_packet(MCPL_BCAST_EOC, 0, WITH_PAYLOAD);
	// wait until pop init done
	while(initPopDone==FALSE) {
		// do something ?? generate RV!
		generateRV();
	}
	showChromosomes(0, 0);

	// then go into GA-loop
	// during which leadAp can continue generating random numbers useful for roullete wheel
	gaIter = 0;
	SolutionFound = -1;
	do {
		doEvaluation(0,0);	// will produces objVal, fitVal, probVal, cdf
		if(SolutionFound >= 0) {
			// do something....
			break;
		}
		else {
			doSelection(cdf, rv, nRV, selectedChr);
			// TODO: special for elite, put them directly into the new generation pool
			// Note: in addition, those elite also undergo crossover operation!!!
			doCrossover(gaParams, selectedChr, chr);
			doMutation(gaParams, chr);
		}
	} while(gaIter < gaParams.nIter);
	toc = sv->clock_ms;
	io_printf(IO_STD, "Simulation runs in %u-msec\n", toc-tic);
	reportFinal();
}

void reportFinal()
{
	// report the final solution
	io_printf(IO_STD, "TODO: final report!\n");
}

void doEvaluation(uint arg0, uint arg1)
{
	ushort i,j;

	nBestChr = 0;		// prepare for collecting the best chromosomes from workers
	objEvalDone = FALSE;
	objValCntr = 0;
	TFitness = 0.0;
	spin1_send_mc_packet(MCPL_BCAST_OBJEVAL, 0, WITH_PAYLOAD);
	while(objEvalDone==FALSE) {
		// do something ?? generate RV!
		generateRV();
	}
	showObjFitValues(0, 0);

	SolutionFound = searchForSolution();
	if(SolutionFound >=0 ) {
		// do something else...
		return;
	}

	// broadcast TFitness and trigger probability computation in workers
	probEvalDone = FALSE;
	probValCntr = 0;
	uint * pTFitness = (uint *)&TFitness;
	spin1_send_mc_packet(MCPL_BCAST_TFITNESS, *pTFitness, WITH_PAYLOAD);
	while(probEvalDone==FALSE) {
		// do something ?? generate RV!
		generateRV();
	}

	// then accumulate probabilities into cdf
	dmaGetProbDone = FALSE;		// will be reset in hDMADone
	REAL *probBuf;				// make a local (DTCM) copy of allProb to speed up computation
	probBuf = sark_alloc(gaParams.nChr, sizeof(uint));
	uint tid = 0;
	while(tid==0) {
		tid = spin1_dma_transfer(DMA_TAG_GETPROB_R, (void *)allProb,
									  (void *)probBuf, DMA_READ, gaParams.nChr*sizeof(uint));
		if(tid==0) {
			io_printf(IO_STD, "DMA for prob full. Retry!\n");
		}
	}

	while(dmaGetProbDone==FALSE) {
		// while waiting for dma to completed, let's continue generating rv for roulette wheel
		generateRV();
	}
	// let's finish generating RV !
	for(i=nRV; i<gaParams.nChr; i++)
		rv[i] = genrand_fixp(0.0, 1.0, 0);

	// then continue with computing cdf
	for(i=0; i<gaParams.nChr; i++) {
		cdf[i] = 0.0;
		for(j=0; j<(i+1); j++)
			cdf[i] += probBuf[i];
	}
	// just for debugging
	showProbValues(0,0);
	sark_free(probBuf);
}

// searchForSolution() return 1 if found, other wise return 0
// it is performed by leadAp
// it uses bestChr[] and allObjVal for searching
int searchForSolution()
{
	int c, result = -1;
	REAL bestOV = allObjVal[bestChr[0]];
	ushort bestC = bestChr[c];

	// select which chromosomes is the best
	for(c = 1; c<nBestChr; c++) {
#ifdef BIGGER_IS_BETTER
		if(allObjVal[bestChr[c]] > bestOV) {
			bestOV = allObjVal[bestChr[c]];
			bestC = bestChr[c];
		}
#else
		if(allObjVal[bestChr[c]] < bestOV) {
			bestOV = allObjVal[bestChr[c]];
			bestC = bestChr[c];
		}
#endif
	}

	// check if bestOV is within the threshold
	if(bestOV >= gaParams.lowerThreshold && bestOV <= gaParams.upperThreshold)
		result = (int)bestC;
	return result;
}

void getChrChunk(uint arg0, uint arg1)
{
	uint *dest = chr + (workers.chrIdxStart * gaParams.nGen);
	uint tid = spin1_dma_transfer(DMA_TAG_CHRCHUNK_R, (void *)dest,
					   (void *)chrChunk, DMA_READ, workers.szChrChunk);
	if(tid==0)
		io_printf(IO_BUF, "DMA request error!\n");
	else
		io_printf(IO_BUF, "DMA is requested with tid = %d, tag = %d!\n", tid, DMA_TAG_CHRCHUNK_W);
}


// at this point, the basic parameters such as nChr and nGen should have been defined
// here we compute chrIdxStart and chrIdxEnd
// computeWload() is triggered when worker receives MCPL_BCAST_EOC
void computeWload(uint arg0, uint arg1)
{
	uint n, r, i, j;
	ushort wID;
    // workload, start point, end poing
    ushort wl[NUM_CORES_USED], sp[NUM_CORES_USED] = {0}, ep[NUM_CORES_USED];
	wID = workers.subBlockID;    // this is my working ID
	n = gaParams.nChr / workers.tAvailable;
	r = gaParams.nChr % workers.tAvailable;
	// to make the remaining part more distributed rather than accumulated at one core:
	for(i=0; i<workers.tAvailable; i++) {
        wl[i] = n;
        if(r>0) {
            wl[i]++;
			for(j=i+1; j<workers.tAvailable; j++)
				sp[j] +=1;	// we have to shift 1 point
            r--;
        }
        sp[i] += i*n;
        ep[i] = sp[i] + wl[i] - 1;
    }

	workers.chrIdxStart = sp[wID];
	workers.chrIdxEnd = ep[wID];
	workers.nChrChunk = wl[wID];
	workers.szChrChunk = workers.nChrChunk * gaParams.nGen * sizeof(uint);

	// debugging info
	io_printf(IO_BUF, "nChr = %d, n = %d, r = %d, tAvailable = %d, wID = %d\n",
		gaParams.nChr, n, r, workers.tAvailable, wID);
	io_printf(IO_BUF, "chrIdxStart = %d, chrIdxEnd = %d, nChrChunk = %d\n",
			  workers.chrIdxStart, workers.chrIdxEnd, workers.nChrChunk);

	// then prepare DTCM memory for chrChunk
	if(chrChunk != NULL)
		sark_free(chrChunk);

	io_printf(IO_BUF, "Will allocate DTCM %d-bytes for chunk\n", workers.szChrChunk);
	chrChunk = sark_alloc(1, workers.szChrChunk);
	if(chrChunk == NULL) {
		io_printf(IO_STD, "Fatal Error allocating DTCM for chunk by core-%d\n", myCoreID);
		io_printf(IO_BUF, "Fatal Error allocating DTCM for chunk by core-%d\n", myCoreID);
		rt_error(RTE_ABORT);
	}
	else {
		io_printf(IO_BUF, "worker-%d chrChunk is @ 0x%x\n", wID, chrChunk);
	}

	uint szDTCMbuf = workers.nChrChunk * sizeof(uint);
	// then prepare DTCM memory for objVal
	if(objVal != NULL)
		sark_free(objVal);
    io_printf(IO_BUF, "Will allocate DTCM %d-bytes for objVal\n", szDTCMbuf);
	objVal = sark_alloc(workers.nChrChunk, sizeof(uint));
	if(objVal == NULL) {
		io_printf(IO_STD, "Fatal Error allocating DTCM for objVal by core-%d\n", myCoreID);
		io_printf(IO_BUF, "Fatal Error allocating DTCM for objVal by core-%d\n", myCoreID);
		rt_error(RTE_ABORT);
	}
	else {
		io_printf(IO_BUF, "worker-%d objVal is @ 0x%x\n", wID, objVal);
	}

	// then prepare DTCM for fitVal
	if(fitVal != NULL)
		sark_free(fitVal);
    io_printf(IO_BUF, "Will allocate DTCM %d-bytes for fitVal\n", szDTCMbuf);
	fitVal = sark_alloc(workers.nChrChunk, sizeof(uint));
	if(fitVal == NULL) {
		io_printf(IO_STD, "Fatal Error allocating DTCM for fitVal by core-%d\n", myCoreID);
		io_printf(IO_BUF, "Fatal Error allocating DTCM for fitVal by core-%d\n", myCoreID);
		rt_error(RTE_ABORT);
	}
	else {
		io_printf(IO_BUF, "worker-%d objVal is @ 0x%x\n", wID, fitVal);
	}

    // then prepare DTCM for prob
    if(prob != NULL)
        sark_free(prob);
    io_printf(IO_BUF, "Will allocate DTCM %d-bytes for prob\n", szDTCMbuf);
    prob = sark_alloc(nChrChunk, sizeof(uint));
    if(prob == NULL) {
		io_printf(IO_STD, "Fatal Error allocating DTCM for prob by core-%d\n", myCoreID);
		io_printf(IO_BUF, "Fatal Error allocating DTCM for prob by core-%d\n", myCoreID);
        rt_error(RTE_ABORT);
    }
    else {
		io_printf(IO_BUF, "worker-%d prob is @ 0x%x\n", wID, prob);
    }

	// then automatically initialize population
	initPopulation();
}

// poolWorkers() might be called in re-simulation model
void poolWorkers(uint arg0, uint arg1)
{
    // send ping to all workers, including the leadAp :)
    spin1_send_mc_packet(MCPL_BCAST_PING, 0, WITH_PAYLOAD);
}

// computeProb() will be call if worker receives broadcasted TFitness via MCPL_BCAST_TFITNESS
void computeProb(uint arg0, uint arg1)
{
    uint i;
    uint pv;
	for(i=0; i<workers.nChrChunk; i++)
        prob[i] = fitVal[i] / TFitness;

    // then upload to sdram
    io_printf(IO_BUF, "allprob = 0x%x\n", allProb);
	uint *dest = (uint *)allProb + workers.chrIdxStart;
	uint szDMA = workers.nChrChunk * sizeof(uint);
	uint tid = 0;
	while(tid==0) {
		tid = spin1_dma_transfer(DMA_TAG_PROBVAL_W, (void *)dest,
						   (void *)prob, DMA_WRITE, szDMA);
		if(tid==0)
			io_printf(IO_BUF, "DMA for prob full. Retry!\n");
	}

	// debugging: see the prob values
	// showProbValues(0, 0);

    // finally, tell leader that we're done objVal
    spin1_send_mc_packet(MCPL_2LEAD_PROB_RPT, 0, WITH_PAYLOAD);
}

// cross() will be called by and its main task is to broadcast message to workers
// key = 0xc502xxxx, payload=ffffmmmm,
// xxxx = num of cp, ffff (father), mmmm (mother)
// key = 0xc503yyyy, payload=zzzzuuuu
// yyyy is cp-indexer, zzzz, uuuu are the points (cp values)
// if *CP is NULL, then the cross-point will be calculated here
void cross(ushort p[2], uint mode)
{
	uint key, parent;
	spin1_memcpy(&parent, p, sizeof(uint));
	// who's the next worker to be activated?
	key = (workers.wID[workers.lp] << 24) + mode;
	workers.lp++;
	// create a ring counter
	if(workers.lp==workers.tAvailable)
		workers.lp = 0;
	// send to the worker
	spin1_send_mc_packet(key, parent, WITH_PAYLOAD);
}

// execCross() is worker's code for doing crossover operation
// arg0 contains mode, arg1 contains parents
// Note:
// - at this point, worker should already know where the newChr is
void execCross(uint arg0, uint arg1)
{
	ushort p[2];
	ushort mode = arg0 & 0xFFFF;
	ushort nBits = getChrBitLength();
	spin1_memcpy(p, &arg1, sizeof(uint));
	if(mode==CP_MODE_SINGLE) {
		// TODO: how to make sure that the crossover point is
        // within allowed bit range? -> use regression!!!
		ushort cp = genrand_ushort(0, nBits, 0);
        // prepare two children
		uint *c1 = sark_alloc(gaParams.nGen, sizeof(uint));
		uint *c2 = sark_alloc(gaParams.nGen, sizeof(uint));
        // TODO: are the parent in chrChunk?
        uint *p1, *p2;
        // if(p[0]>=chrIdxStart && p[0]<=chrIdxEnd)
    }
	else if(mode==CP_MODE_DOUBLE) {
		ushort cp[2];
		cp[0] = genrand_ushort(0, getChrBitLength(), 0);
		cp[1] = genrand_ushort(cp[0], nBits, 0);
	}
	else if(mode==CP_MODE_UNIFORM) {
	}
}

// objEval() is a callback that will be called if worker receives MCPL_BCAST_OBJEVAL
void objEval(uint arg0, uint arg1)
{
	ushort i, j;
	uint genes[DEF_MAX_GENE];
	uint *gSrc = chrChunk;			// worker still holds the last working chromosome chunk
	REAL ov, fv;					// objective value and fitness value
	uint *pfv = (uint *)&fv;
//	uint *ptrObjVal, *ptrFitVal;

	REAL bestOV;
	ushort bestChr;
	for(i=0; i<workers.nChrChunk; i++) {
		// copy from chrChunk
		spin1_memcpy((void *)genes, (void *)gSrc, gaParams.nGen*sizeof(uint));
		gSrc += gaParams.nGen;

		// call objective function defined in gamodel.c by user
		ov = objFunction(gaParams.nGen, genes);
#ifdef BIGGER_IS_BETTER
		if(i==0) {
			bestOV = ov;
			bestChr = workers.chrIdxStart;
		}
		else {
			if(bestOV < ov) {
				bestOV = ov;
				bestChr = workers.chrIdxStart + i;
			}
		}

#else
		if(i==0) {
			bestOV = ov;
			bestChr = workers.chrIdxStart;
		}
		else {
			if(bestOV > ov) {
				bestOV = ov;
				bestChr = workers.chrIdxStart + i;
			}
		}
#endif
		fv = 1.0/ov;
		objVal[i] = ov;
		//fitVal[i] = REAL_CONST(1.0)/objVal[i];
		fitVal[i] = fv;

		// let the leadAp computes the total fitness:
		//io_printf(IO_BUF, "Sending fitval-%d = %k\n", workers.chrIdxStart+c, fv);
		spin1_send_mc_packet(MCPL_2LEAD_FITVAL, *pfv, WITH_PAYLOAD);

		// inform leadAp which chromosome is the best
		spin1_send_mc_packet(MCPL_2LEAD_BEST_CHR, bestChr, WITH_PAYLOAD);
	}

	// then upload to sdram
	io_printf(IO_BUF, "allobjval = 0x%x, allfitval = 0x%x\n", allObjVal, allFitVal);
	uint *dest = (uint *)allObjVal + workers.chrIdxStart;
	uint szDMA = workers.nChrChunk * sizeof(uint);
	uint tid = 0;
	while(tid==0) {
		tid = spin1_dma_transfer(DMA_TAG_OBJVAL_W, (void *)dest,
						   (void *)objVal, DMA_WRITE, szDMA);
		if(tid==0)
			io_printf(IO_BUF, "DMA for objVal full. Retry!\n");
	}
	dest = (uint *)allFitVal + workers.chrIdxStart;
	tid = 0;
	while(tid==0) {
		tid = spin1_dma_transfer(DMA_TAG_FITVAL_W, (void *)dest,
								 (void *)fitVal, DMA_WRITE, szDMA);
		if(tid==0)
			io_printf(IO_BUF, "DMA for fitVal full. Retry!\n");
	}

	// finally, tell leader that we're done objVal
	spin1_send_mc_packet(MCPL_2LEAD_OBJEVAL_RPT, 0, WITH_PAYLOAD);

/*	TODO: find the best objVal and record it?
	REAL bestVal[2];
	uint cIdx[2] = {0};
	// let's put the best one in the first order (index-0)
	if(objVal[0] < objVal[1]) {
		bestVal[0] = objVal[0]; bestVal[1] = objVal[1];
		cIdx[0] = 0; cIdx[1] = 1;
	}
	else {
		bestVal[0] = objVal[1]; bestVal[1] = objVal[0];
		cIdx[0] = 1; cIdx[1] = 0;
	}

	for(i=2; i<TOTAL_CHROMOSOMES; i++) {
		if(objVal[i] < bestVal[0]) {
			// shift the record index-0 to index-1
			cIdx[1] = cIdx[0];
			bestVal[1] = bestVal[0];
			// save the record to index-0
			cIdx[0] = i;
			bestVal[0] = objVal[i];
		}
		else if(objVal[i] < bestVal[1]) {
			cIdx[1] = i;
			bestVal[1] = objVal[i];
		}
	}
	io_printf(IO_BUF, "bestVal = %k on chromosome-%d\n", bestVal[0],cIdx[0]);
	bestObjVal[iter] = bestVal[0];
	bestChromosomes[iter] = cIdx[0];
	// finally, put the best two chromosomes as the elite chromosomes
	eliteParent[0] = cIdx[0];
	eliteParent[1] = cIdx[1];
	//io_printf(IO_BUF, "\nElite = [%d, %d]\n\n", eliteParent[0], eliteParent[1]);
/*
 *     # first, extract x and y from the gen

	objVal = [0.0 for c in range(nChromosomes)]		# the objective function
	fitVal = [0.0 for c in range(nChromosomes)]		# the fitness function
	#total = 0.0

	# then compute objective/evaluation function
	for c in range(nChromosomes):
		x, y = decodeGen(chr[c])		# get x and y from chromosome[c]
		objVal[c] = 20 + (x**2 - 10*math.cos(2*math.pi*x)) + (y**2 - 10*math.cos(2*math.pi*y))
		#total += objVal[c]

	#average = total / nChromosomes

	# finally, compute fitness function as f_i/avg_f -> form Darrell Whitley
	# Note: use the average, not the total!!!
	#for c in range(nChromosomes):
	#    fitVal[c] = objVal[c] / average

	#return objVal, fitVal
	return objVal

 * */
	//float objVal[TOTAL_CHROMOSOMES];
	//float fitVal[TOTAL_CHROMOSOMES];
}


/*-------------------------- Default implementation of GA operators ---------------------------*/


// defRouletteWheel will use *cdf, *rv and *selectedChr
// *rv is created during doEvaluation and passed as arg0 by doSelection()
void defRouletteWheel(REAL *preComputedCDF, REAL *generatedRV, uint numOfRV, REAL *selectedChromosomes)
{
	uint rvCntr, cdfCntr;
	for(rvCntr=0; rvCntr<numOfRV; rvCntr++) {
		for(cdfCntr=0; cdfCntr<numOfRV; cdfCntr++)
			if(preComputedCDF[cdfCntr] > generatedRV[rvCntr]) {
				selectedChromosomes[rvCntr] = cdfCntr;
				break;	// skip cdfCntr and continue with rvCntr
			}
	}
}


void defOnePointCross(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes)
{
	// here is example of 1 point crossover
	// hence, no need to spread 0xc503xxxx
	REAL rv;
    REAL rhoC = p.cRate;
	ushort parent[2];
	ushort p1,p2;
    for(p1=0; p1<p.nChr; p1++) {
		rv = genrand_fixp(0.0, 1.0, 0);
		if(rv < rhoC) {
            parent[0] = selectedChromosomesIdx[p1];
			p2 = parent[0];
			// make sure that no self-crossing
			while(p2==parent[0]) {
				p2 = genrand_ushort(0, nSelectedChr, 0);
				p2 = selectedChr[p2];
			}
			parent[1] = p2;
			cross(parent, CP_MODE_SINGLE);
		}
	}
}

void defTwoPointsCross(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes)
{

}

void defUniformCross(gaParams_t p, uint *selectedChromosomesIdx, uint *chromosomes)
{

}

void defMutation(gaParams_t p, uint *chromosomes)
{

}

