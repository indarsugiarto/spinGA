#include "spinGA.h"

/*-------------------------- EVENT HANDLERS --------------------------------*/
// only leadAp has access to hTimer
void hTimer(uint tick, uint Unused)
{
	// NOTE: as usual, don't start at tick-0 !!!
	if(tick==1) {
		io_printf(IO_STD, "[SpinGA-v%d.%d] running leadAp @core-%d!\n",
				  MAJOR_VERSION, MINOR_VERSION, myCoreID);
		// then trigger the population
		io_printf(IO_BUF, "[leadAp] Populating workers...\n");
		poolWorkers(0,0);
	}
	// assumming that 1sec above is enough for collecting data
	// the payload will be split into:
	// payload.high = total number of worker
	// payload.low = wID
	else if(tick==2) {
		io_printf(IO_BUF, "[leadAp] Broadcasting wIDs\n");
		uint key, payload;
		for(uint i=0; i<workers.tAvailable; i++) {
			io_printf(IO_BUF, "wID core-%d = %d\n", workers.wID[i], i);
			// key.high = coreID, key.low = 0
			key = workers.wID[i] << 24;
			// payload.high = tAvailable, payload.low = wID
			payload = (workers.tAvailable << 16) + i;
			spin1_send_mc_packet(key, payload, WITH_PAYLOAD);
		}
	}
	else if(tick==3) {
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
	else if(tag==DMA_TAG_GETPROB_R) {
		dmaGetProbDone = TRUE;
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
	else if(key==MCPL_2LEAD_FITVAL) {
		REAL *f = (REAL *)&payload;
		TFitness += *f;
	}
	else if(key==MCPL_2LEAD_PROB_RPT) {
		probValCntr++;
		if(probValCntr==workers.tAvailable) {
			probEvalDone = TRUE;
		}
	}
	else if(key==MCPL_2LEAD_PROBVAL) {
		// TODO: hitung cdf yang bener!!!
		// masalahnya, apa yang akan terjadi jika ada core yang mati?
		// makanya, sementara ini tidak pakai cara broadcasting ini
		// untuk mengumpulkan cdf
	}
	/*______________________ workers part ___________________________*/
	else if(key==MCPL_BCAST_PING) {
		// send core-ID to leadAp, including leadAp itself
		spin1_send_mc_packet(MCPL_2LEAD_PING_RPT, myCoreID, WITH_PAYLOAD);
	}
	// if a core is addressed directly
	else if((key>>24)==myCoreID) {
		if((key&0xFFFF)==0) {
			// all cores will receives the following wID
			workers.tAvailable = payload >> 16;
			//workers.wID[myCoreID] = payload & 0xFFFF;
			workers.subBlockID = payload & 0xFFFF;
		}
		else if((key&0xFFFF)==CP_MODE_SINGLE)
			spin1_schedule_callback(execCross, key, payload, PRIORITY_NORMAL);
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
	else if(key==MCPL_BCAST_NEWCHR_ADDR)
		newChr = (uint *)payload;
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
	else if((key&0xFFFF0000)==MCPL_BCAST_CROSS_PAR) {
		//TODO: habis makan!
	}
}

