#include <spin1_api.h>
#include "spinGA.h"

/*------------------------------ Initialization ------------------------------*/
void initSDP()
{
	spin1_callback_on(SDP_PACKET_RX, hSDP, PRIORITY_SDP);
    reportMsg->flags = 0x07;	//no reply
    reportMsg->tag = SDP_TAG_REPLY;
    reportMsg->srce_port = SDP_PORT_CONFIG;
    reportMsg->srce_addr = sv->p2p_addr;
    reportMsg->dest_port = PORT_ETH;
    reportMsg->dest_addr = sv->eth_addr;
    reportMsg->length = sizeof(sdp_hdr_t) + sizeof(cmd_hdr_t);
}

/* Routing strategy for sending a chromosome (broadcast):
 * - broadcast ping
 *   key     : 0xbca50001
 *   payload : 0
 *   dest    : 0xFFFFFF80 -> intra chip only, excluding core-0
 * - worker reports ID
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
    e = rtr_alloc(2);
    if ( e== 0)
        rt_error(RTE_ABORT);
    else {
        // allRoute &= ~leader;    // remove leadAp
        rtr_mc_set(e, MCPL_BCAST_PING, 0xFFFFFFFF, allRoute);
        rtr_mc_set(e+1, MCPL_2LEAD_RPT, 0xFFFFFFFF, leader);
    }
}

// initGA() will allocate memory in SDRAM to hold chromosomes
void initGA()
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
// in final version, the GA configuration can be sent via SDP from a host
void selfSimulation()
{
    nChr = DEF_N_CHR;
    nGen = DEF_RASTRIGIN_ORDER;
}
/*--------------------------------------------------------- Initialization ---*/

/*---------------------------- EVENT HANDLERS ----------------------------------*/
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
    // assumming that 1s is enough for collecting data
    else if(tick==3) {
        for(uint i=0; i<workers.tAvailable; i++) {
            spin1_send_mc_packet(workers.wID[i], i, WITH_PAYLOAD);
        }
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

}


void hMCPL(uint key, uint payload)
{
    if(key==MCPL_BCAST_PING) {
        // send core-ID to leadAp, including leadAp itself
        spin1_send_mc_packet(MCPL_2LEAD_RPT, myCoreID, WITH_PAYLOAD);
    }
    else if(key==MCPL_2LEAD_RPT) {
        workers.wID[workers.tAvailable] = payload;
        workers.tAvailable++;
    }
    // all cores will receives the following wID
    else if(key==myCoreID) {
        spin1_schedule_callback(computeWload, 0, 0, PRIORITY_NORMAL);
    }
}

void computeWload(uint arg0, uint arg1)
{

}

void poolWorkers(uint arg0, uint arg1)
{
    // send ping to all workers, including the leadAp :)
    spin1_send_mc_packet(MCPL_BCAST_PING, 0, WITH_PAYLOAD);
}

/*--------------------------------------------------------- EVENT HANDLERS ---*/


/*----------------------------- Main Program ---------------------------------*/
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
        // timer is optional, just for debugging
        spin1_set_timer_tick(TIMER_TICK_PERIOD);
        spin1_callback_on(TIMER_TICK, hTimer, PRIORITY_TIMER);

        initSDP();

        workers.tAvailable = 0;

        io_printf(IO_BUF, "Initializing router...\n");
        //io_printf(IO_STD, "The leader core is %u\n", myCoreID);
        initRouter();

        // instead of GA config from host, let's just do selfSimulation()
        selfSimulation();
        initGA();

        spin1_delay_us(500000); // let workers to be settle
    }

    /* Run simulation */
    //spin1_start(SYNC_WAIT);
    spin1_start(SYNC_NOWAIT);
}


/*-------------------------------- IMPLEMENTATION ----------------------------------*/


/* initMyChromosomes() will generate initial N-chromosomes that will be stored in the Chromosomes buffer
 * (where N is DEF_RASTRIGIN_ORDER).
 * At this point, the collectedChromosomes will be equal to DEF_RASTRIGIN_ORDER
 * Afterwards, each cell will wait for chromosomes from another cells.
 * */
void initMyChromosomes()
{
    /* generate random value between MIN_PARAM and MAX_PARAM */
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
}

void showMyChromosomes()
{
    uint i,j;
    for(i=0; i<DEF_N_CHR_PER_CORE*NUM_CORES_USED; i++) {
        io_printf(IO_BUF, "Chromosome-%2d = 0x", i);
        for(j=0; j<DEF_RASTRIGIN_ORDER; j++)
            io_printf(IO_BUF, "%x", Chromosomes[i][j]);
        io_printf(IO_BUF, "\n");
        spin1_delay_us(1000);
    }
}

void bcastMyChromosomes(uint arg0, uint arg1)
{
    // Debugging
    //io_printf(IO_STD, "Cell-%d broadcasts chromosomes...\n", myCellID);
    //showMyChromosomes();
    uint i, j, key, data, idx;
    uint ii, ss, xx;	// key: 0xBCiissxx, "BC" = broadcast id, ii = chromosome ID, ss = cell-ID, xx = gen-ID
    for(i=0; i<DEF_N_CHR_PER_CORE; i++)
        for(j=0; j<DEF_RASTRIGIN_ORDER; j++){
            idx = myCellID*DEF_N_CHR_PER_CORE + i;
            xx = j;
            ss = myCellID << 8;
            ii = idx << 16; // chromosome ID is computed by the sender cell
            key = 0xBC000000;
            key |= (ii | ss | xx);
            data = Chromosomes[idx][j];
            spin1_send_mc_packet(key, data, WITH_PAYLOAD);
            //io_printf(IO_BUF, "Sending 0x%x:0x%x\n", key, data);
            //io_printf(IO_STD, "Broadcast my chromosomes:\n");
            //TODO: masih ada packet drop!!!
            //spin1_delay_us(100);
        }
}

/*----------------------------------- GA Stuffs ----------------------------------------*/
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

unsigned int bin2gray(unsigned int num)
{
    return num ^ (num >> 1);
}

unsigned int gray2bin(unsigned int num)
{
    num = num ^ (num >> 16);
    num = num ^ (num >> 8);
    num = num ^ (num >> 4);
    num = num ^ (num >> 2);
    num = num ^ (num >> 1);
    return num;
}
/*_______________________________________________ Helper functions _*/
