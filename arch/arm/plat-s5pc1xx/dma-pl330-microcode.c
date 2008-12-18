

/*---------------------- Primitive functions -------------*/
/* DMAMOV CCR, ...  */
static void encodeDmaMoveChCtrl(void)
{
	
}

/* DMAMOV SAR, uStAddr
 * DMAMOV DAR, uStAddr   */
static void encodeDmaMove(bool bSrcDir, u32 uStAddr)
{
	
}

/* DMAMOV SAR, uStAddr */
static void encodeDmaMoveSRC(u32 uStAddr)
{
	
}

/* DMAMOV DAR, uStAddr */
static void encodeDmaMoveDST(u32 uStAddr)
{
	
}

/* DMAMOV CCR, ...  */
static void encodeDmaMoveCCR(u32 uStAddr)
{
	
}

/* DMALD, DMALDS, DMALDB  */
static void encodeDmaLoad(void)
{
	
}

/* DMALDPS, DMALDPB (Load Peripheral)  */
static void encodeDmaLoadPeri(void)
{
	
}

/* DMAST, DMASTS, DMASTB  */
static void encodeDmaStore(void)
{
	
}

/* DMASTPS, DMASTPB (Store and notify Peripheral)  */
static void encodeDmaStorePeri(void)
{
	
}

/* DMASTZ  */
static void encodeDmaStoreZero(void)
{
	
}

/* DMALP  */
static void encodeDmaLoop(u8 uLoopCnt, u8 uIteration)
{
	
}

/* DMALPFE  */
static void encodeDmaLoopForever(u8 uLoopCnt)
{
	
}

/* DMALPEND, DMALPENDS, DMALPENDB  */
static void encodeDmaLoopEnd(u8 uLoopCnt)
{
	
}

/*  DMAWFP, DMAWFPS, DMAWFPB (Wait For Peripheral) */
static void encodeDmaWaitForPeri(void)
{	
	
}

/* DMAWFE (Wait For Event) */
static void encodeDmaWaitForEvent(u8 uEventNum) // 0~31
{
	
}

/*  DMAFLUSHP (Flush and notify Peripheral) */
static void encodeDmaFlushPeri(void)
{
	
}

/* DMAEND */
static void encodeDmaEnd(void)
{
	
}

/* DMAADDH (Add Halfword) */
static void encodeDmaAddHalfword(bool bSrcDir, u16 uStAddr)
{
	
}

/* DMAKILL (Kill) */
static void encodeDmaKill(void)
{
	
}

/* DMANOP (No operation) */
static void encodeDmaNop(void)
{
	
}


/* DMARMB (Read Memory Barrier) */
static void encodeDmaReadMemBarrier(void)
{
	
}

/* DMASEV (Send Event) */
static void encodeDmaSendEvent(u8 uEventNum) // 0~31
{
	
}



/* DMAWMB (Write Memory Barrier) */
static void encodeDmaWriteMemBarrier(void)
{
	
}

/* DMAGO (Go) */
static void encodeDmaGo(void)
{
	
}

/*---------------------- Feature functions -------------*/

/* start_DMA_controller
 * - start the DMA controller
 *
 */
void start_DMA_controller() 
{
	
}


/* stop_DMA_controller
 * - stop the DMA controller
 *	
 */
void stop_DMA_controller() 
{
	
}


/* start_DMA_channel
 * - get the DMA channel started
 *
 *	chanNum		the number to be started
 *	sStAddr		the address to be set to PC of the DMAC
 */
u32 start_DMA_channel(u8 chanNum, u32 uStAddr) 
{
	return encodeDmaGo(chanNum, uStAddr);
}


/* stop_DMA_channel
 * - get the DMA channel stopped
 *
 *	chanNum		the number to be stopped
 */
void stop_DMA_channel(u8 chanNum) 
{
	
}


/* setup_DMA_start_address
 * - set the DMA start address
 *
 *	uStAddr		the DMA start address
 */
void setup_DMA_start_address(u32 uStAddr) 
{
	encodeDmaMoveSRC(uStAddr);
}


/* setup_DMA_destination_address
 * - set the DMA destination address
 *
 *	uStAddr		the DMA destination address
 */
void setup_DMA_destination_address(u32 uStAddr) 
{
	encodeDmaMoveDST(uStAddr);
}


/* setup_DMA_transfer_size
 * - set the transfer size
 *
 *	dmaDir		the DMA direction
 *	transferSize	the size to be transfered
 */
void setup_DMA_transfer_size(u8 dmaDir, u32 transferSize)
{	

	encodeDmaLoop(uLoopCnt, uIteration);
	
	switch(dmaDir) {
		case DMA_M2M:]
			encodeDmaLoad();
			encodeDmaStore();
			break;
		
		case DMA_M2P:
			encodeDmaWaitForPeri();
			encodeDmaLoad();
			encodeDmaStorePeri();
			break;
			
		case DMA_P2M:
			encodeDmaWaitForPeri();
			encodeDmaLoadPeri();
			encodeDmaStore();
			break;
			
		case DMA_P2P:
			break;
			
		default:
			break;
	}
			
	encodeDmaLoopEnd(uLoopCnt);
}


/* setup_DMA_control
 * - set the burst length, burst size, source and destination increment/fixed field
 *
 *	dcon	the value for channel control register
 */
void setup_DMA_control(u32 dcon)
{
	encodeDmaMoveCCR(dcon);
}


/* register_irq_to_DMA_channel
 * - register an event with the DMA channel
 *
 *	uEventNum	the event number
 */
void register_irq_to_DMA_channel(u8 uEventNum)
{
	encodeDmaSendEvent(uEventNum);
}
