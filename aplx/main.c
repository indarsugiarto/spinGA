#include "spinGA.h"

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
		workers.lp = 0;

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
