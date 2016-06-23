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

void initRouter()
{
	uint allRoute = 0xFFFF80;	// excluding core-0 and external links
	uint leader = (1 << (myCoreID+6));

	// do we ask leadAp to work as worker?
#if (LEADAP_AS_WORKER==FALSE)
	allRoute &= ~leader;
#endif

	uint e, i, key;
	// individual info/cmd, assuming 17 working core
	// hence, we might have a packet drop if there's a dead core !!!
	e = rtr_alloc(17);
	if (e == 0)
		rt_error(RTE_ABORT);
	else {
		// each destination core might have its own key association
		// so that leadAp can address each worker directly
		for(i=0; i<17; i++) {
			key = (i+1) << 24;
			// starting from core-1 up to core-17
			rtr_mc_set(e+i, key, MCPL_DIRECT_BASE_MASK, (MC_CORE_ROUTE(i+1)));
		}
	}
	// broadcast and toward_leader MCPL
	e = rtr_alloc(20);
	if ( e== 0)
		rt_error(RTE_ABORT);
	else {
		// broadcasted keys: 12
		rtr_mc_set(e, MCPL_BCAST_PING,				0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_CHR_ADDR,			0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_NEWCHR_ADDR,		0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_OBJVAL_ADDR,		0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_FITVAL_ADDR,		0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_NCHRGEN,			0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_MINVAL,			0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_MAXVAL,			0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_EOC,				0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_OBJEVAL,			0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_TFITNESS,			0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_UPDATE_CHR_CHUNK,	0xFFFFFFFF, allRoute); e++;
		rtr_mc_set(e, MCPL_BCAST_CROSS_PAR,			0xFFFF0000, allRoute); e++;
		// to leadAp keys: 8
		rtr_mc_set(e, MCPL_2LEAD_PING_RPT,			0xFFFFFFFF, leader); e++;
		rtr_mc_set(e, MCPL_2LEAD_INITCHR_RPT,		0xFFFFFFFF, leader); e++;
		rtr_mc_set(e, MCPL_2LEAD_OBJEVAL_RPT,		0xFFFFFFFF, leader); e++;
		rtr_mc_set(e, MCPL_2LEAD_OBJVAL,			0xFFFFFFFF, leader); e++;
		rtr_mc_set(e, MCPL_2LEAD_FITVAL,			0xFFFFFFFF, leader); e++;
		rtr_mc_set(e, MCPL_2LEAD_PROB_RPT,			0xFFFFFFFF, leader); e++;
		rtr_mc_set(e, MCPL_2LEAD_BEST_CHR,			0xFFFFFFFF, leader); e++;
	}

	// TODO: broadcast emigrants out of the chip

}

// initGA() will allocate memory in SDRAM to hold chromosomes
void initMemGA()
{
	if(leadAp) {
		// prepare container for current chromosomes
		uint szMem = gaParams.nChr * gaParams.nGen * sizeof(uint);
		if(chr != NULL)
			sark_xfree(sv->sdram_heap, chr, ALLOC_LOCK);
		chr = sark_xalloc(sv->sdram_heap, szMem, SDRAM_TAG_CHR, ALLOC_LOCK);
		if(chr==NULL) {
			io_printf(IO_STD, "Fatal SDRAM allocation fail for SDRAM_TAG_CHR!\n");
			rt_error(RTE_ABORT);
		}

		// prepare container for new population/new generation
		if(newChr != NULL)
			sark_xfree(sv->sdram_heap, newChr, ALLOC_LOCK);
		newChr = sark_xalloc(sv->sdram_heap, szMem, SDRAM_TAG_NEWCHR, ALLOC_LOCK);
		if(newChr==NULL) {
			io_printf(IO_STD, "Fatal SDRAM allocation fail for SDRAM_TAG_NEWCHR!\n");
			rt_error(RTE_ABORT);
		}
		tNewGen = 0;

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

		// prepare container for cdf in DTCM of leadAp
		if(cdf != NULL)
			sark_free(cdf);
		cdf = sark_alloc(nChr, sizeof(uint));

		// prepare random values and roulette wheel result in DTCM of leadAp
		if(rv != NULL)
			sark_free(rv);
		rv = sark_alloc(gaParams.nChr, sizeof(uint)); // random values for roulette wheel
		if(selectedChr != NULL)
			sark_free(selectedChr);
		selectedChr = sark_alloc(nChr, sizeof(ushort));
		if(selectedChr==NULL) {
			io_printf(IO_STD, "Fatal DTCM for allocating selectedChr!\n");
			rt_error(RTE_ABORT);
		}

		// distribute this information to workers
		io_printf(IO_BUF, "@chr = 0x%x, @allobjVal = 0x%x, @allfitVal = 0x%x, @allProb = 0x%x\n",
				  chr, allObjVal, allFitVal, allProb);
		spin1_send_mc_packet(MCPL_BCAST_CHR_ADDR, (uint)chr, WITH_PAYLOAD);
		spin1_send_mc_packet(MCPL_BCAST_NEWCHR_ADDR, (uint)newChr, WITH_PAYLOAD);
		spin1_send_mc_packet(MCPL_BCAST_OBJVAL_ADDR, (uint)allObjVal, WITH_PAYLOAD);
		spin1_send_mc_packet(MCPL_BCAST_FITVAL_ADDR, (uint)allFitVal, WITH_PAYLOAD);
		spin1_send_mc_packet(MCPL_BCAST_PROB_ADDR, (uint)allProb, WITH_PAYLOAD);
	}
}

// this is the opposite of initMemGA()
void releaseMemGA()
{
	if(leadAp) {
		if(chr != NULL)
			sark_xfree(sv->sdram_heap, chr, ALLOC_LOCK);
		if(newChr != NULL)
			sark_xfree(sv->sdram_heap, newChr, ALLOC_LOCK);
		if(allObjVal != NULL)
			sark_xfree(sv->sdram_heap, allObjVal, ALLOC_LOCK);
		if(allFitVal != NULL)
			sark_xfree(sv->sdram_heap, allFitVal, ALLOC_LOCK);
		if(allProb != NULL)
			sark_xfree(sv->sdram_heap, allProb, ALLOC_LOCK);
		if(cdf != NULL)
			sark_free(cdf);
		if(selectedChr != NULL)
			sark_free(selectedChr);
		if(rv != NULL)
			sark_free(rv);
	}
}


void initPopulation()
{
	uint c, g;	// chromosome and gene counter
	REAL G;		// the randomly generated gene value
	uint eG;	// encoded gen
	uint *pChrChunk = chrChunk;

	// step-1: generate chromosomes and store in worker's chrChunk

	for(c=0; c<workers.nChrChunk; c++) {
		for(g=0; g<gaParams.nGen; g++) {
			G = genrand_fixp(gaParams.minGenVal, gaParams.maxGenVal, 0);  // 0 = don't init rnd again
			//gg = bin2gray(encodeGen(G));	--> still wrong with bin2gray?
			eG = encodeGen(G);
			*pChrChunk = eG;
			pChrChunk++;
		}
	}

	// step-2: transfer to sdram via dma
	// NOTE: uint direction: 0 = transfer to TCM, 1 = transfer to system
	//       return 0 if the request queue is full, or DMA transfer ID otherwise
	uint *dest = chr + (chrIdxStart * nGen);
	io_printf(IO_BUF, "Will store in sdram @ 0x%x\n", dest);
	uint tid = 0;
	while(tid == 0) {
		tid = spin1_dma_transfer(DMA_TAG_CHRCHUNK_W, (void *)dest,
						   (void *)chrChunk, DMA_WRITE, workers.szChrChunk);
		if(tid==0)
			io_printf(IO_BUF, "DMA full. Retry!\n");
	}

	// for debugging only:
	showChromosomes(0,0);

	// step-3: report to leadAp
	spin1_send_mc_packet(MCPL_2LEAD_INITCHR_RPT, 0, WITH_PAYLOAD);
}
