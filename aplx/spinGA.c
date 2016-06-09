/* Coba lihat yang pernah dilakukan di:
 * try.with.stdfixpoint.in.embarasingly.parallel
 * */
#include <spin1_api.h>
#include "spinGA.h"

/*---------------------------- Initialization ------------------------------*/
void initSDP()
{
	spin1_callback_on(SDP_PACKET_RX, hSDP, PRIORITY_SDP);
    reportMsg->flags = 0x07;	//no reply
	reportMsg->tag = SDP_TAG;
	reportMsg->srce_port = SDP_PORT;
    reportMsg->srce_addr = sv->p2p_addr;
    reportMsg->dest_port = PORT_ETH;
    reportMsg->dest_addr = sv->eth_addr;
    //reportMsg->length = sizeof(sdp_hdr_t) + sizeof(cmd_hdr_t);
}

/* Routing strategy for sending a chromosome (broadcast):
 * - broadcast ping, for collecting workers' coreID
 *   key     : 0xbca50001
 *   payload : 0
 *   dest    : 0xFFFFFF80 -> intra chip only, excluding core-0
 * - worker reports coreID
 *   key     : 0x1ead0001
 *   payload : core-ID
 *   dest    : (1 << (leadApCore+6))
 * - individual info/cmd
 *   key     : core-ID
 *   payload : info/cmd
 *   dest    : MC_CORE_ROUTE
 * */
void initRouter()
{
    uint allRoute = 0xFFFF80;	// excluding core-0 and external links
    uint leader = (1 << (myCoreID+6));

	// do we ask leadAp to work as worker?
#if (LEADAP_AS_WORKER==FALSE)
	allRoute &= ~leader;
#endif

    uint e, i;
    // individual info/cmd, assuming 17 working core
	e = rtr_alloc(NUM_CORES_USED);
    if (e == 0)
        rt_error(RTE_ABORT);
    else {
        // each destination core might have its own key association
        // so that leadAp can tell each worker, which region is their part
		for(i=0; i<NUM_CORES_USED; i++)
            // starting from core-1 up to core-17
            rtr_mc_set(e+i, i+1, 0xFFFFFFFF, (MC_CORE_ROUTE(i+1)));
    }
    // broadcast and toward_leader MCPL
	e = rtr_alloc(15);
    if ( e== 0)
        rt_error(RTE_ABORT);
    else {
        // allRoute &= ~leader;    // remove leadAp
        rtr_mc_set(e, MCPL_BCAST_PING, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+1, MCPL_2LEAD_PING_RPT, 0xFFFFFFFF, leader);
		rtr_mc_set(e+2, MCPL_2LEAD_INITCHR_RPT, 0xFFFFFFFF, leader);
		rtr_mc_set(e+3, MCPL_BCAST_NCHRGEN, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+4, MCPL_BCAST_MINVAL, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+5, MCPL_BCAST_MAXVAL, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+6, MCPL_BCAST_EOC, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+7, MCPL_BCAST_CHR_ADDR, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+8, MCPL_BCAST_OBJEVAL, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+9, MCPL_2LEAD_OBJEVAL_RPT, 0xFFFFFFFF, leader);
		rtr_mc_set(e+10, MCPL_2LEAD_OBJVAL, 0xFFFF0000, leader);
		rtr_mc_set(e+11, MCPL_2LEAD_FITVAL, 0xFFFF0000, leader);
		rtr_mc_set(e+12, MCPL_BCAST_UPDATE_CHR_CHUNK, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+13, MCPL_BCAST_OBJVAL_ADDR, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+14, MCPL_BCAST_FITVAL_ADDR, 0xFFFFFFFF, allRoute);
    }
}

// initGA() will allocate memory in SDRAM to hold chromosomes
void initMemGA()
{
	// prepare container for chromosomes
	uint szMem = nChr * nGen * sizeof(uint);
    if(chr != NULL)
        sark_xfree(sv->sdram_heap, chr, ALLOC_LOCK);
	chr = sark_xalloc(sv->sdram_heap, szMem, SDRAM_TAG_CHR, ALLOC_LOCK);
    if(chr==NULL) {
		io_printf(IO_STD, "Fatal SDRAM allocation fail for SDRAM_TAG_CHR!\n");
        rt_error(RTE_ABORT);
    }

	// prepare container for objValues
	szMem = nChr * sizeof(uint);
	if(allObjVal != NULL)
		sark_xfree(sv->sdram_heap, allObjVal, ALLOC_LOCK);
	allObjVal = sark_xalloc(sv->sdram_heap, szMem, SDRAM_TAG_OBJVAL, ALLOC_LOCK);
	if(allObjVal==NULL) {
		io_printf(IO_STD, "Fatal SDRAM allocation fail for SDRAM_TAG_OBJVAL!\n");
		rt_error(RTE_ABORT);
	}

	// prepare container for fitValues
	if(allFitVal != NULL)
		sark_xfree(sv->sdram_heap, allFitVal, ALLOC_LOCK);
	allFitVal = sark_xalloc(sv->sdram_heap, szMem, SDRAM_TAG_FITVAL, ALLOC_LOCK);
    if(allFitVal==NULL) {
		io_printf(IO_STD, "Fatal SDRAM allocation fail for SDRAM_TAG_FITVAL!\n");
		rt_error(RTE_ABORT);
	}

    // prepare container for prob
    if(allProb != NULL)
        sark_xfree(sv->sdram_heap, allProb, ALLOC_LOCK);
    allProb = sark_xalloc(sv->sdram_heap, szMem, SDRAM_TAG_PROB, ALLOC_LOCK);
    if(allProb==NULL) {
        io_printf(IO_STD, "Fatal SDRAM allocation fail for SDRAM_TAG_PROB!\n");
        rt_error(RTE_ABORT);
    }

	// if OK, distribute this information to workers
	else {
		io_printf(IO_STD, "@chr = 0x%x, @allobjVal = 0x%x, @allfitVal = 0x%x\n",
				  chr, allObjVal, allFitVal);
		spin1_send_mc_packet(MCPL_BCAST_CHR_ADDR, (uint)chr, WITH_PAYLOAD);
		spin1_send_mc_packet(MCPL_BCAST_OBJVAL_ADDR, (uint)allObjVal, WITH_PAYLOAD);
		spin1_send_mc_packet(MCPL_BCAST_FITVAL_ADDR, (uint)allFitVal, WITH_PAYLOAD);
        spin1_send_mc_packet(MCPL_BCAST_PROB_ADDR, (uint)allProb, WITH_PAYLOAD);
	}
}

// selfSimulation() is just for providing an alternative for GA configuration
// in final version, the GA configuration (all variables) will be sent via SDP from a host
void selfSimulation()
{
	// first, imitate host-config
    nChr = DEF_N_CHR;
    nGen = DEF_RASTRIGIN_ORDER;
	minGenVal = DEF_RASTRIGIN_MINVAL;
	maxGenVal = DEF_RASTRIGIN_MAXVAL;
	nIter = DEF_MAX_ITER;

	// prepare memory allocation for chromosomes
	// inside initMemGA(), chr address is distributed!
	initMemGA();

	// then distribute the above parameters
	spin1_send_mc_packet(MCPL_BCAST_NCHRGEN, (nChr << 16) + nGen, WITH_PAYLOAD);
	spin1_send_mc_packet(MCPL_BCAST_MINVAL, (uint)minGenVal, WITH_PAYLOAD);
	spin1_send_mc_packet(MCPL_BCAST_MAXVAL, (uint)maxGenVal, WITH_PAYLOAD);

	runGA(nIter);
}

void runGA(uint iter)
{
	// step1: send notification to start initializing population
	initPopDone = FALSE;
	initPopCntr = 0;
	spin1_send_mc_packet(MCPL_BCAST_EOC, 0, WITH_PAYLOAD);
	// wait until pop init done
	while(initPopDone==FALSE) {
		// do something ??
	}
	showChromosomes(0, 0);
	objEvalDone = FALSE;
	objValCntr = 0;
	TFitness = 0.0;
	spin1_send_mc_packet(MCPL_BCAST_OBJEVAL, 0, WITH_PAYLOAD);
	while(objEvalDone==FALSE) {
		// do something ??
	}
	showFitValues(0, 0);
    io_printf(IO_STD, "TFitness = %k\n", TFitness);
    // broadcast TFitness
    probEvalDone = FALSE;
    uint * pTFitness = (uint *)&TFitness;
    spin1_send_mc_packet(MCPL_BCAST_TFITNESS, *pTFitness, WITH_PAYLOAD);
    while(probEvalDone==FALSE) {
    }
    //showProbValues(0,0)
}

void showChromosomes(uint arg0, uint arg1)
{
	uint c,g,gg;
	REAL G;
	uint *pChr = chr;
	io_printf(IO_STD, "Initial population:\n-------------------\n");
	for(c=0; c<nChr; c++) {
		for(g=0; g<nGen; g++) {
			gg = *pChr; pChr++;
			G = decodeGen(gg);
			io_printf(IO_STD, "[ G = 0x%x = %k, gg = 0x%x ] ", G, G, gg);
		}
		io_printf(IO_STD, "\n");
	}
}

void showFitValues(uint arg0, uint arg1)
{
	io_printf(IO_STD, "Obj/fit values:\n------------------\n");
	uint *pChr = chr;
	REAL *pO = allObjVal;
	REAL *pF = allFitVal;
	REAL G1, G2, O, F;
	REAL cek;
	uint g1, g2;
	for(uint i=0; i<nChr; i++) {
		g1 = *pChr; pChr++; g2 = *pChr; pChr++;
		G1 = decodeGen(g1); G2 = decodeGen(g2);
		O = *pO; pO++;
		F = *pF; pF++;
		cek = 1.0/O;
		io_printf(IO_STD, "chr-%d = [%k %k] -> %k/%k -> %k\n",
				  i, G1, G2, O, F, cek);
		spin1_delay_us(1000);
	}
}

/*------------------------------------------------------- Initialization ---*/

/*-------------------------- EVENT HANDLERS --------------------------------*/
void hTimer(uint tick, uint Unused)
{
    if(tick==1)
        io_printf(IO_STD, "Get ready!\n");
    else if(tick==2) {
        // then trigger the population
        io_printf(IO_STD, "Populating workers...\n");
        // when scheduling, DON'T USE PRIORITY LOWER THAN 1
        spin1_schedule_callback(poolWorkers, 0, 0, PRIORITY_NORMAL);
    }
    // assumming that 1sec above is enough for collecting data
	// the payload will be split into:
	// payload.high = total number of worker
	// payload.low = wID
    else if(tick==3) {
		io_printf(IO_STD, "Broadcasting wIDs\n");
		uint payload;
		for(uint i=0; i<workers.tAvailable; i++) {
			io_printf(IO_STD, "wID core-%d = %d\n", workers.wID[i], i);
			payload = (workers.tAvailable << 16) + i;
			spin1_send_mc_packet(workers.wID[i], payload, WITH_PAYLOAD);
        }
    }
	else if(tick==4) {
		io_printf(IO_STD, "Start selfSimulation\n");
		// instead of GA config from host, let's just do selfSimulation()
		selfSimulation();   // should be replaced by SDP in future
	}
}


void hSDP(uint mBox, uint port)
{
    sdp_msg_t *msg = (sdp_msg_t *)mBox;
    // just debugging:
    io_printf(IO_STD, "Receiving sdp via port-%d\n", port);
    spin1_msg_free(msg);
}

void hDMADone(uint tid, uint tag)
{
	io_printf(IO_BUF, "dma tag-%d has finished!\n", tag);
	if(tag==DMA_TAG_CHRCHUNK_W) {
		spin1_send_mc_packet(MCPL_2LEAD_INITCHR_RPT, myCoreID, WITH_PAYLOAD);
	}
}

void hMCPL(uint key, uint payload)
{
	/*______________________ leadAp part ___________________________*/
	if(key==MCPL_2LEAD_PING_RPT) {
		workers.wID[workers.tAvailable] = payload;
		workers.tAvailable++;
	}
	else if(key==MCPL_2LEAD_INITCHR_RPT) {
		initPopCntr++;
		if(initPopCntr==workers.tAvailable) {
			// TODO: what if there is dead core? then use timeout mechanism!
			initPopDone = TRUE;
			// optional, just for debugging:
			//spin1_schedule_callback(showChromosomes, 0, 0, PRIORITY_NORMAL);
		}
	}
	else if(key==MCPL_2LEAD_OBJEVAL_RPT) {
		objValCntr++;
		// TODO: alternative to counter is by using timeout
		if(objValCntr==workers.tAvailable) {
			objEvalDone = TRUE;
			//spin1_schedule_callback(showFitValues, 0, 0, PRIORITY_NORMAL);
		}
	}
    else if((key&0xFFFF0000)==MCPL_2LEAD_FITVAL) {
        REAL *f = (REAL *)&payload;
        TFitness += *f;
    }
    else if((key&0xFFFF000)==MCPL_2LEAD_PROBVAL) {
        //TODO: hitung cdf yang bener!!!
    }
	/*______________________ workers part ___________________________*/
	else if(key==MCPL_BCAST_PING) {
		// send core-ID to leadAp, including leadAp itself
		spin1_send_mc_packet(MCPL_2LEAD_PING_RPT, myCoreID, WITH_PAYLOAD);
	}
    // all cores will receives the following wID
    else if(key==myCoreID) {
		workers.tAvailable = payload >> 16;
		workers.wID[myCoreID] = payload & 0xFFFF;
	}
	else if(key==MCPL_BCAST_NCHRGEN) {
		nGen = payload & 0xFFFF;
		nChr = payload >> 16;
		io_printf(IO_BUF, "got nGen = %d, nChr = %d\n", nGen, nChr);
	}
	else if(key==MCPL_BCAST_MINVAL)
		minGenVal = (REAL)payload;
	else if(key==MCPL_BCAST_MAXVAL)
		maxGenVal = (REAL)payload;
	else if(key==MCPL_BCAST_EOC)
		spin1_schedule_callback(computeWload, 0, 0, PRIORITY_NORMAL);
	else if(key==MCPL_BCAST_CHR_ADDR)
		chr = (uint *)payload;
	else if(key==MCPL_BCAST_OBJVAL_ADDR)
		allObjVal = (REAL *)payload;
	else if(key==MCPL_BCAST_FITVAL_ADDR)
		allFitVal = (REAL *)payload;
    else if(key==MCPL_BCAST_PROB_ADDR)
        allProb = (REAL *)payload;
	else if(key==MCPL_BCAST_OBJEVAL)
		spin1_schedule_callback(objEval, 0, 0,PRIORITY_NORMAL);
	else if(key==MCPL_BCAST_UPDATE_CHR_CHUNK)
		spin1_schedule_callback(getChrChunk, 0,0, PRIORITY_NORMAL);
    else if(key==MCPL_BCAST_TFITNESS) {
        spin1_memcpy(&TFitness, &payload, sizeof(uint));
        spin1_schedule_callback(computeProb, 0, 0, PRIORITY_NORMAL);
    }
}

void getChrChunk(uint arg0, uint arg1)
{
	uint *dest = chr + (chrIdxStart * nGen);
	uint tid = spin1_dma_transfer(DMA_TAG_CHRCHUNK_R, (void *)dest,
					   (void *)chrChunk, DMA_READ, szChrChunk*sizeof(uint));
	if(tid==0)
		io_printf(IO_BUF, "DMA request error!\n");
	else
		io_printf(IO_BUF, "DMA is requested with tid = %d, tag = %d!\n", tid, DMA_TAG_CHRCHUNK_W);
}

// at this point, the basic parameters such as nChr and nGen should have been defined
// here we compute chrIdxStart and chrIdxEnd
void computeWload(uint arg0, uint arg1)
{
	uint n, r, i, j;
	ushort tAvailable, wID;
    // workload, start point, end poing
    ushort wl[NUM_CORES_USED], sp[NUM_CORES_USED] = {0}, ep[NUM_CORES_USED];
	tAvailable = workers.tAvailable;
    wID = workers.wID[myCoreID];    // this is my working ID
	n = nChr / tAvailable;
	r = nChr % tAvailable;
	io_printf(IO_BUF, "nChr = %d, n = %d, r = %d, tAvailable = %d, wID = %d\n",
		nChr, n, r, tAvailable, wID);
	// to make the remaining part more distributed rather than accumulated at one core:
    for(i=0; i<NUM_CORES_USED; i++) {
        wl[i] = n;
        if(r>0) {
            wl[i]++;
			for(j=i+1; j<NUM_CORES_USED; j++)
				sp[j] +=1;	// we have to shift 1 point
			// sp[i+1]=1;  // sama dengan sp[i+1]++; -> wrong!
            r--;
        }
        sp[i] += i*n;
        ep[i] = sp[i] + wl[i] - 1;
    }

    chrIdxStart = sp[wID];
    chrIdxEnd = ep[wID];
	io_printf(IO_BUF, "chrIdxStart = %d, chrIdxEnd = %d\n", chrIdxStart, chrIdxEnd);

	// then prepare DTCM memory for chrChunk
	if(chrChunk != NULL)
		sark_free(chrChunk);
    //szChrChunk = (chrIdxEnd - chrIdxStart + 1)*nGen;
	nChrChunk = wl[wID];
	szChrChunk = nChrChunk * nGen;
	io_printf(IO_BUF, "Will allocate DTCM %d-bytes for chunk\n", szChrChunk*sizeof(uint));
	chrChunk = sark_alloc(szChrChunk, sizeof(uint));
	if(chrChunk == NULL) {
		io_printf(IO_STD, "Fatal Error allocating DTCM for chunk by core-%d\n", myCoreID);
		io_printf(IO_BUF, "Fatal Error allocating DTCM for chunk by core-%d\n", myCoreID);
		rt_error(RTE_ABORT);
	}
	else {
		io_printf(IO_BUF, "@chrChunk = 0x%x\n", chrChunk);
	}

    uint szDTCMbuf = nChrChunk * sizeof(uint);
	// then prepare DTCM memory for objVal
	if(objVal != NULL)
		sark_free(objVal);
    io_printf(IO_BUF, "Will allocate DTCM %d-bytes for objVal\n", szDTCMbuf);
	objVal = sark_alloc(nChrChunk, sizeof(uint));
	if(objVal == NULL) {
//		io_printf(IO_STD, "Fatal Error allocating DTCM for objVal by core-%d\n", myCoreID);
//		io_printf(IO_BUF, "Fatal Error allocating DTCM for objVal by core-%d\n", myCoreID);
		rt_error(RTE_ABORT);
	}
	else {
		io_printf(IO_BUF, "@objVal = 0x%x\n", objVal);
	}

	// then prepare DTCM for fitVal
	if(fitVal != NULL)
		sark_free(fitVal);
    io_printf(IO_BUF, "Will allocate DTCM %d-bytes for fitVal\n", szDTCMbuf);
	fitVal = sark_alloc(nChrChunk, sizeof(uint));
	if(fitVal == NULL) {
//		io_printf(IO_STD, "Fatal Error allocating DTCM for fitVal by core-%d\n", myCoreID);
//		io_printf(IO_BUF, "Fatal Error allocating DTCM for fitVal by core-%d\n", myCoreID);
		rt_error(RTE_ABORT);
	}
	else {
		io_printf(IO_BUF, "@objVal = 0x%x\n", fitVal);
	}

    // then prepare DTCM for prob
    if(prob != NULL)
        sark_free(prob);
    io_printf(IO_BUF, "Will allocate DTCM %d-bytes for prob\n", szDTCMbuf);
    prob = sark_alloc(nChrChunk, sizeof(uint));
    if(prob == NULL) {
//		io_printf(IO_STD, "Fatal Error allocating DTCM for prob by core-%d\n", myCoreID);
//		io_printf(IO_BUF, "Fatal Error allocating DTCM for prob by core-%d\n", myCoreID);
        rt_error(RTE_ABORT);
    }
    else {
        io_printf(IO_BUF, "@prob = 0x%x\n", prob);
    }

	// then automatically initialize population
	initPopulation();
}

void poolWorkers(uint arg0, uint arg1)
{
    // send ping to all workers, including the leadAp :)
    spin1_send_mc_packet(MCPL_BCAST_PING, 0, WITH_PAYLOAD);
}

void computeProb(uint arg0, uint arg1)
{
    uint i;
    ushort chrNum = chrIdxStart;
    uint pv;
    for(i=0; i<nChrChunk; i++) {
        prob[i] = fitVal[i] / TFitness;
        spin1_memcpy((void *)&pv, (void *)&prob[i], sizeof(uint));
        io_printf(IO_BUF, "chr-%d -> prob = %k\n", chrIdxStart+i, prob[i]);
        io_printf(IO_BUF, "Sending prob-%d = %k\n", chrNum, fv);
        spin1_send_mc_packet(MCPL_2LEAD_PROBVAL + chrNum, pv, WITH_PAYLOAD);
        chrNum++;
    }
    // then upload to sdram
    io_printf(IO_BUF, "allprob = 0x%x\n", allProb);
    uint *dest = (uint *)allProb + chrIdxStart;
    uint szDMA = nChrChunk * sizeof(uint);
    uint tid = spin1_dma_transfer(DMA_TAG_PROBVAL_W, (void *)dest,
                       (void *)prob, DMA_WRITE, szDMA);
    if(tid==0)
        io_printf(IO_BUF, "DMA request for prob error!\n");
    else
        io_printf(IO_BUF, "DMA is requested for prob: tid = %d, tag = %d!\n",
                  tid, DMA_TAG_PROBVAL_W);

    // finally, tell leader that we're done objVal
    spin1_send_mc_packet(MCPL_2LEAD_PROB_RPT, 0, WITH_PAYLOAD);
}

/*------------------------------------------------------- EVENT HANDLERS ---*/


/*---------------------------- Main Program --------------------------------*/
void c_main()
{
    myCoreID = spin1_get_core_id();

    /* Initialize system */
    unsigned long seed = (sark_chip_id () << 8) + myCoreID * sv->time_ms;
    init_genrand(seed);

    /* Setup callbacks */
    spin1_callback_on(MCPL_PACKET_RECEIVED, hMCPL, 0);
    spin1_callback_on(DMA_TRANSFER_DONE, hDMADone, PRIORITY_DMA);

    if(leadAp) {
        initSDP();

        workers.tAvailable = 0;

        io_printf(IO_BUF, "Initializing router...\n");
        //io_printf(IO_STD, "The leader core is %u\n", myCoreID);
        initRouter();

        // timer is optional, in this case, we use it to trigger simulation
		spin1_set_timer_tick(TIMER_TICK_PERIOD_US);
        spin1_callback_on(TIMER_TICK, hTimer, PRIORITY_TIMER);

        spin1_delay_us(500000); // let workers to be settle
    }

    /* Run simulation */
    //spin1_start(SYNC_WAIT);
    spin1_start(SYNC_NOWAIT);
}


/*------------------------------ IMPLEMENTATION -------------------------------*/


/*----------------------------------- GA Stuffs ----------------------------------------*/

// in initPopulation(), each worker generates initial populations and store them in *chr
void initPopulation()
{
	io_printf(IO_BUF, "In initPopulation()\nchrIdxStart = %d, chrIdxEnd = %d\n", chrIdxStart, chrIdxEnd);
	uint c, g;	// chromosome and gene counter
	uint gg;	// encoded gen
	REAL G;
	uint **pChr = (uint **)chrChunk;
	uint *pChrChunk = chrChunk;
//	io_printf(IO_BUF, "chrChunk @ 0x%x, pChr @ 0x%x, pChrChunk @ 0x%x\n", chrChunk, pChr, pChrChunk);
	// step-1: generate chromosomes and store in chrChunk
//	io_printf(IO_BUF, "Generated genes:\n------------------\n");
	for(c=0; c<(chrIdxEnd-chrIdxStart+1); c++) {
		for(g=0; g<nGen; g++) {
            G = genrand_fixp(minGenVal, maxGenVal, 0);  // 0 = don't init rnd again
			//gg = bin2gray(encodeGen(G));	--> still wrong with bin2gray?
			gg = encodeGen(G);
//			io_printf(IO_BUF, "[ G = 0x%x = %k, gg = 0x%x ] ", G, G, gg);
			*pChrChunk = gg;
			//io_printf(IO_BUF, "@pChrChunk = 0x%x\n", pChrChunk);
			pChrChunk++;
			//io_printf(IO_BUF, "@pChrChunk = 0x%x\n", pChrChunk);
			//pChr[c][g] = gg;
		}
//		io_printf(IO_BUF, "\n");
	}
	// step-2: transfer to sdram via dma
	// NOTE: uint direction: 0 = transfer to TCM, 1 = transfer to system
	//       return 0 if the request queue is full, or DMA transfer ID otherwise
	uint *dest = chr + (chrIdxStart * nGen);
	io_printf(IO_BUF, "Will store in sdram @ 0x%x\n", dest);
	uint tid = spin1_dma_transfer(DMA_TAG_CHRCHUNK_W, (void *)dest,
					   (void *)chrChunk, DMA_WRITE, szChrChunk*sizeof(uint));
	if(tid==0)
		io_printf(IO_BUF, "DMA request error!\n");
	else
		io_printf(IO_BUF, "DMA is requested with tid = %d, tag = %d!\n", tid, DMA_TAG_CHRCHUNK_W);

	// TODO: check check!
	/*
	uint h,i, gg;
	REAL g, ig;
	// initialize own chromosomes, don't care with the others...
	for(h=0; h<DEF_N_CHR_PER_CORE; h++)
		for(i=0; i<DEF_RASTRIGIN_ORDER; i++) {
			g = genrand_fixp(MIN_PARAM, MAX_PARAM, sv->clock_ms);
			//Chromosomes[myCellID*DEF_N_CHR_PER_CORE + h][i] = encodeGen(g);
			//io_printf(IO_BUF, "Generating g = %k, converted into 0x%x\n", g, encodeGen(g));
			//gg = genrand_int32();
			//gg = spin1_rand();
			gg = encodeGen(g);
			ig = decodeGen(gg);
			Chromosomes[myCellID*DEF_N_CHR_PER_CORE + h][i] = gg;
			io_printf(IO_BUF, "Generating gen = %k -> 0x%x -> %k\n", g, gg, ig);
		}
	collectedGenes = DEF_N_CHR_PER_CORE * DEF_RASTRIGIN_ORDER;
	*/
}

// objEval() is a callback that will be called if worker receives MCPL_BCAST_OBJEVAL
void objEval(uint arg0, uint arg1)
{
	uint i, j;
	// assuming that worker still holds the last working chromosome chunk
	uint genes[DEF_MAX_GENE];
	uint *gSrc = chrChunk;
	REAL ov, fv;
	uint *pfv = (uint *)&fv;
//	uint *ptrObjVal, *ptrFitVal;
	ushort chrNum = chrIdxStart;
	for(i=0; i<nChrChunk; i++) {
		// copy from chrChunk
		spin1_memcpy((void *)genes, (void *)gSrc, nGen*sizeof(uint));
		gSrc += nGen;
		ov = objFunction(nGen, genes);
		fv = 1.0/ov;
		objVal[i] = ov;
		//fitVal[i] = REAL_CONST(1.0)/objVal[i];
		fitVal[i] = fv;
		io_printf(IO_BUF, "chr-%d -> ov = %k, fv = %k\n", chrIdxStart+i, objVal[i], fitVal[i]);
//		ptrObjVal = &objVal[i];
//		ptrFitVal = &fitVal[i];
//		spin1_send_mc_packet(MCPL_2LEAD_OBJVAL + chrNum, *ptrObjVal, WITH_PAYLOAD);
		// the following is useful for computing total fitness:
		io_printf(IO_BUF, "Sending fitval-%d = %k\n", chrNum, fv);
		spin1_send_mc_packet(MCPL_2LEAD_FITVAL + chrNum, *pfv, WITH_PAYLOAD);
		chrNum++;
	}
	// then upload to sdram
	io_printf(IO_BUF, "allobjval = 0x%x, allfitval = 0x%x\n", allObjVal, allFitVal);
	uint *dest = (uint *)allObjVal + chrIdxStart;
	uint szDMA = nChrChunk * sizeof(uint);
	uint tid = spin1_dma_transfer(DMA_TAG_OBJVAL_W, (void *)dest,
					   (void *)objVal, DMA_WRITE, szDMA);
	if(tid==0)
		io_printf(IO_BUF, "DMA request for objVal error!\n");
	else
		io_printf(IO_BUF, "DMA is requested for objVal: tid = %d, tag = %d!\n",
				  tid, DMA_TAG_OBJVAL_W);
	dest = (uint *)allFitVal + chrIdxStart;
	tid = spin1_dma_transfer(DMA_TAG_FITVAL_W, (void *)dest,
					   (void *)fitVal, DMA_WRITE, szDMA);
	if(tid==0)
		io_printf(IO_BUF, "DMA request for fitVal error!\n");
	else
		io_printf(IO_BUF, "DMA is requested for fitVal: tid = %d, tag = %d!\n",
				  tid, DMA_TAG_FITVAL_W);

	// finally, tell leader that we're done objVal
	spin1_send_mc_packet(MCPL_2LEAD_OBJEVAL_RPT, 0, WITH_PAYLOAD);

/*
	// find the best objVal and record it
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


/* This shows how roundr() function works
int main()
{
  float conver = 45.592346543;
  printf("conver is %0.1f\n",conver);

  conver = conver*10.0f;
  conver = (conver > (floor(conver)+0.5f)) ? ceil(conver) : floor(conver);
  conver = conver/10.0f;

  //If you're using C99 or better, rather than ANSI C/C89/C90, the following will also work.
  //conver = roundf(conver*10.0f)/10.0f;

  printf("conver is now %f\n",conver);
  return 0;
}
*/


/*_______________________ Helper functions _________________________*/
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
/*_______________________________________________ Helper functions _*/
