#include <spin1_api.h>
#include "spinGA.h"

/*---------------------------- Initialization ------------------------------*/
void initSDP()
{
	spin1_callback_on(SDP_PACKET_RX, hSDP, PRIORITY_SDP);
    reportMsg->flags = 0x07;	//no reply
    reportMsg->tag = SDP_TAG_REPLY;
    reportMsg->srce_port = SDP_PORT_CONFIG;
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
    mask = 0xFFFFFFFF;
    // individual info/cmd, assuming 17 working core
    e = rtr_alloc(17);
    if (e == 0)
        rt_error(RTE_ABORT);
    else {
        // each destination core might have its own key association
        // so that leadAp can tell each worker, which region is their part
        for(i=0; i<17; i++)
            // starting from core-1 up to core-17
            rtr_mc_set(e+i, i+1, 0xFFFFFFFF, (MC_CORE_ROUTE(i+1)));
    }
    // broadcast and toward_leader MCPL
	e = rtr_alloc(3);
    if ( e== 0)
        rt_error(RTE_ABORT);
    else {
        // allRoute &= ~leader;    // remove leadAp
        rtr_mc_set(e, MCPL_BCAST_PING, 0xFFFFFFFF, allRoute);
		rtr_mc_set(e+1, MCPL_2LEAD_PING_RPT, 0xFFFFFFFF, leader);
		rtr_mc_set(e+2, MCPL_2LEAD_INITCHR_RPT, 0xFFFFFFFF, leader);
    }
}

// initGA() will allocate memory in SDRAM to hold chromosomes
void initMemGA()
{
    uint szChr = nChr * nGen * sizeof(uint);
    if(chr != NULL)
        sark_xfree(sv->sdram_heap, chr, ALLOC_LOCK);
    chr = sark_xalloc(sv->sdram_heap, szChr, appID, ALLOC_LOCK);
    if(chr==NULL) {
        io_printf(IO_STD, "SDRAM allocation fail!\n");
        rt_error(RTE_ABORT);
    }
}

// selfSimulation() is just for providing an alternative for GA configuration
// in final version, the GA configuration (all variables) will be sent via SDP from a host
void selfSimulation()
{
    nChr = DEF_N_CHR;
    nGen = DEF_RASTRIGIN_ORDER;
	minGenVal = DEF_RASTRIGIN_MINVAL;
	maxGenVal = DEF_RASTRIGIN_MAXVAL;
	initMemGA();
}

void reportInitChr(uint arg0, uint arg1)
{

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
		uint payload;
		for(uint i=0; i<workers.tAvailable; i++) {
			payload = (workers.tAvailable << 16) + i;
			spin1_send_mc_packet(workers.wID[i], payload, WITH_PAYLOAD);
        }
    }
	else if(tick==4) {
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
	if(tag==DMA_TAG_CHRCHUNK_W) {
		spin1_send_mc_packet(MCPL_2LEAD_INITCHR_RPT, myCoreID, WITH_PAYLOAD);
	}
}


void hMCPL(uint key, uint payload)
{
    if(key==MCPL_BCAST_PING) {
        // send core-ID to leadAp, including leadAp itself
		spin1_send_mc_packet(MCPL_2LEAD_PING_RPT, myCoreID, WITH_PAYLOAD);
    }
    else if(key==MCPL_2LEAD_RPT) {
        workers.wID[workers.tAvailable] = payload;
        workers.tAvailable++;
    }
    // all cores will receives the following wID
    else if(key==myCoreID) {
		spin1_schedule_callback(computeWload, payload, 0, PRIORITY_NORMAL);
    }
	else if(key==MCPL_2LEAD_INITCHR_RPT) {
		initPopCntr++;
		if(initPopCntr==workers.tAvailable)
			spin1_schedule_callback(reportInitChr, 0, 0, PRIORITY_NORMAL);
	}
}

// at this point, the basic parameters such as nChr and nGen have been defined
// here we compute chrIdxStart and chrIdxEnd
void computeWload(uint payload, uint arg1)
{
	uint n, r;
	ushort tAvailable, wID;
	tAvailable = payload >> 16;
	wID = payload & 0xFFFF;
	workers.tAvailable = tAvailable;
	workers.wID[myCoreID] = wID;	// working load (wID) might be different to coreID
	n = nChr / tAvailable;
	r = nChr % tAvailable;
	chrIdxStart = wID * n;
	chrIdxEnd = chrIdxStart + n-1;
	if(wID==tAvailable-1)
		chrIdxEnd += r;
	// then prepare DTCM memory
	if(chrChunk != NULL)
		sark_free(chrChunk);
	szChrChunk = (chrIdxEnd - chrIdxStart + 1)*nGen;
	chrChunk = sark_alloc(szChrChunk, sizeof(uint));
	if(chrChunk == NULL) {
		io_printf(IO_STD, "Error allocating DTCM for chunk by core-%d\n", myCoreID);
		rt_error(RTE_ABORT);
	}
	// then automatically initialize population
	initPopulation();
}

void poolWorkers(uint arg0, uint arg1)
{
    // send ping to all workers, including the leadAp :)
    spin1_send_mc_packet(MCPL_BCAST_PING, 0, WITH_PAYLOAD);
}

/*------------------------------------------------------- EVENT HANDLERS ---*/


/*---------------------------- Main Program --------------------------------*/
void c_main()
{
    myCoreID = spin1_get_core_id();
    myCellID = myCoreID - 1;    // for c-style indexing

    /* Initialize system */
    unsigned long seed = (sark_chip_id () << 8) + myCoreID * sv->time_ms;
    init_genrand(seed);
    getRegParam(&m, &b);    // if using regression

    /* Generate initial population */
    initMyChromosomes();

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
        spin1_set_timer_tick(TIMER_TICK_PERIOD);
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
	uint c, g;	// chromosome and gene counter
	uint gg;	// encoded gen
	REAL G;
	uint **pChr = (uint **)chrChunk;
	// step-1: generate chromosomes and store in chrChunk
	for(c=0; c<(chrIdxEnd-chrIdxStart+1); c++)
		for(g=0; g<nGen; g++) {
			G = genrand_fixp(minGenVal, maxGenVal, 0);
			gg = bin2gray(encodeGen(G));
			pChr[c][g] = gg;
		}
	// step-2: transfer to sdram via dma
	// NOTE: uint direction: 0 = transfer to TCM, 1 = transfer to system
	spin1_dma_transfer(DMA_TAG_CHRCHUNK_W, (void *)chr,
					   (void *)chrChunk, DMA_WRITE, szChrChunk*sizeof(uint));

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

void objEval()
{
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
    float objVal[TOTAL_CHROMOSOMES];
    float fitVal[TOTAL_CHROMOSOMES];
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
