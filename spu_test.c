#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <limits.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <time.h>
#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <linux/types.h>


char psxBuffer[2 * 1024 * 1024];

//#include "../Main_MiSTerMSU/file_io.h"
//#include "../Main_MiSTerMSU/user_io.h"
//#include "../Main_MiSTerMSU/menu.h"

// DMA testing stuff. ElectronAsh...
#define DEBUG
#include <sys/mman.h>
//#include "hwlib.h"
//#include "sgdma.h"
//#include "hps_0.h"
//#include "dma.h"

//setting for the HPS2FPGA AXI Bridge
#define ALT_AXI_FPGASLVS_OFST (0xC0000000)	// axi_master
#define HW_FPGA_AXI_SPAN (0x00400000) 		// Bridge span 4MB
#define HW_FPGA_AXI_MASK ( HW_FPGA_AXI_SPAN - 1 )

#define BRIDGE_0_BASE 0x00000
#define BRIDGE_0_SPAN 0xFFFFF
#define BRIDGE_0_END  0xFFFFF

#define HW_FPGA_FB_OFST (0x10000000)
#define HW_FPGA_FB_SPAN (0x00400000) 		// Bridge span 4MB
#define HW_FPGA_FB_MASK ( HW_FPGA_FB_SPAN - 1 )

void *axi_addr;
void *fb_addr;

/*
ADR +0 = Write / Read to GP0
ADR +4 = Write / Read to GP1

ADR +8 = Bit 31 DMA_ACK (Write)
		 Bit 30 DMA_REQ Read / gpu_nrst (Write). Must keep gpu_nrst bit HIGH when writing to DMA_ACK!
		 Bit 29 IRQRequest Read 
         Bit 28 dbg_canWrite
		 Bit 27:0 myDebugCnt (GPU counter cycle)

ADR +12= Read Data bus (cpuDataOut), without any other CPU signal.
         Write Data bus (cpuDataIn) + DMA_ACK = true (pulse).
*/

void writeRaw(uint32_t data) {
	uint32_t* BASE_GPU = (uint32_t *)axi_addr;
	
	while ( !(BASE_GPU[2] & 1<<28) );	// Wait for dbg_canWrite flag (High) before sending data. If LOW, then stay in while loop. TODO: Add timeout?
	BASE_GPU[0] = data;
}

/*
typedef unsigned short u16;
typedef unsigned int   u32;

class GPUManager {
public:
	GPUManager(u32* gpuBase):BASE_GPU(gpuBase),diff(0),writeIdx(0),readIdx(0) {}
	
	bool canPush      () { return (BASE_GPU[2] & 1<<28);    }
	u32  getGPUCycle  () { return BASE_GPU[2] & 0x0FFFFFFF; }

	void StartGPUReset() { BASE_GPU[2] = 0<<30;                 }
	void EndGPUReset  () { BASE_GPU[2] = 1<<30;                 }
	
	bool canWriteCommand() {
		return (diff < MAX_SIZE-2);
	}
	
	void writeCommand (u32 v) {
		// User must check to know, but I check here to avoid memory overwrite.
		if (canWriteCommand()) {
			diff++;
			buffer[writeIdx++] = v;
			if (writeIdx >= MAX_SIZE) { writeIdx = 0; }
		}
	}
	
	void executeInLoop() {
		if (canPush() && diff) {
			writeGP0(buffer[readIdx++]);
			if (readIdx >= MAX_SIZE) { readIdx = 0; }
			diff--;
		}
	}

	void waitUntilWrite(u32 data) {
		while (!canWriteCommand()) { 
			executeInLoop();
		}
		writeCommand(data);
		executeInLoop();
	}
	
private:
	static const u32 MAX_SIZE = 4096;
	
	void writeGP0(u32 value) { BASE_GPU[0] = value; }
	void writeGP1(u32 value) { BASE_GPU[1] = value; }

	u32* BASE_GPU;
	u32 diff;
	u32 readIdx;
	u32 writeIdx;
	u32 buffer[MAX_SIZE];
};
*/

uint8_t mmap_setup_done = 0;


int mmap_setup() {
	printf("HPS AXI Bridge Setup. ElectronAsh / dentnz\n\n");

	void *axi_virtual_base;
	void *fb_virtual_base;

	int fd;

	printf("Opening /dev/mem, for mmap...\n");
	if ( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n\n" );
		return( 1 );
	}
	else {
		printf("/dev/mem opened OK...\n\n");
	}

	axi_virtual_base = mmap( NULL, HW_FPGA_AXI_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, ALT_AXI_FPGASLVS_OFST );
	printf("Mapping the HPS-to_FPGA bridge for AXI...\n");
	if ( axi_virtual_base == MAP_FAILED ) {
		printf( "ERROR: axi mmap() failed...\n\n" );
		close( fd );
		return( 1 );
	}
	else {
		printf("HPS-to_FPGA bridge (for AXI) mapped OK.\n");
	}
	axi_addr = axi_virtual_base + ( ( unsigned long  )( BRIDGE_0_BASE ) & ( unsigned long)( HW_FPGA_AXI_MASK ) );
	printf("axi_addr: 0x%08X\n\n", (unsigned int)axi_addr );
	
	
	fb_virtual_base = mmap( NULL, HW_FPGA_FB_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_FPGA_FB_OFST );
	printf("Mapping the HPS-to_FPGA bridge for Framebuffer...\n");
	if ( fb_virtual_base == MAP_FAILED ) {
		printf( "ERROR: fb mmap() failed...\n\n" );
		close( fd );
		return( 1 );
	}
	else {
		printf("HPS-to_FPGA bridge (for FB) mapped OK.\n");
	}
	fb_addr = fb_virtual_base + ( ( unsigned long  )( 0x00000000 ) & ( unsigned long)( HW_FPGA_FB_MASK ) );
	printf("fb_addr: 0x%08X\n\n", (unsigned int)fb_addr );

	printf("Closing /dev/mem...\n\n");
	close( fd );

	return 0;
}

/*
void parser(const char* fileName, u16* psxBuffer, GPUManager& mgr, uint32_t delay) {
	FILE* binSrc = fopen(fileName,"rb");

	//printf("feof flag: %d\n", feof(binSrc) );
	
	// ---------------------------------------------------------------------------------
	// Method 1 ----- Read file directly into DDR VRAM of the PSX ?
	// Load dump from VRAM in memory.
	// With this method, stencil cache is not updated correctly on real HW.
	
    u16* buff16 = (u16*)psxBuffer;
    for (int y=0; y < 524288; y++) {
        fread(&buff16[y],sizeof(u16),1,binSrc);
        //if ((y & 0x3FF) == 0) { printf("Y:%i\n", y >> 10); }
        //usleep(1);
    }
	
	//u16* buff16 = (u16*)psxBuffer;
	//fread(psxBuffer,sizeof(u16),1024*512,binSrc);

	 In the SIM I update directly the chip stencil with this code
	// ----- Sync Stencil cache and VRAM state.
	//for (int y=0; y < 512; y++) {
		//for (int x=0; x < 1024; x++) {
			//bool bBit    = (buff16[x + (y*1024)] & 0x8000) ? true : false;
			//setStencil(mod,x,y,bBit);
		//}
	//}
	
	//printf("cmdSetup:");

	// ---------------------------------------------------------------------------------
	// Method 2 ---- Create a REAL UPLOAD COMMAND
	// ---- Setup
	u32 setupCommandCount;
	fread(&setupCommandCount, sizeof(u32), 1, binSrc);
	for (int n=0; n < setupCommandCount; n++) {
		u32 cmdSetup;
		fread(&cmdSetup, sizeof(u32),1, binSrc);
		//printf("cmdSetup: 0x%08X  ", cmdSetup);
		writeRaw(cmdSetup);
		if (delay>0) usleep(delay);
	}

	u32 logCommandCount;
	fread(&logCommandCount, sizeof(u32), 1, binSrc);
	for (int n=0; n < logCommandCount; n++) {
		u32 cmdLength;
		fread(&cmdLength, sizeof(u32),1, binSrc);
		for (int m=0; m < cmdLength; m++) {
			u32 operand;
			fread(&operand, sizeof(u32),1, binSrc);
			//printf("operand: 0x%08X\n", operand);
			writeRaw(operand);
			if (delay>0) usleep(delay);
		}
	}
	
	fclose(binSrc);
}
*/


int main()
{
	uint32_t reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7;

	mmap_setup();

	//GPUManager mgr( ((uint32_t *)axi_addr) );
	
/*	
ADR +0 = Write / Read to GP0
ADR +4 = Write / Read to GP1
ADR +8 = Bit 0:27 myDebugCnt (GPU counter cycle)
         Bit 28 dbg_canWrite
         Bit 29 IRQ Read 
         Bit 30 DMA_Req Read / gpu_nrst (Write)
         Bit 31 DMA_Ack (Write)
ADR +12= Read Data bus (cpuDataOut), without any other CPU signal.
         Write Data bus (cpuDataIn) + DMA_ACK = true.
*/


	//printf("Resetting the GPU core...\n\n");

	//mgr.StartGPUReset(); // First thing to do in the morning, brush your teeth.
	//mgr.EndGPUReset();   // Toilet, then breakfast.
	//usleep(10000);		// 10ms delay, just to be sure. Probably don't need this?
	
	/*
	FILE *fd = fopen("/media/fat/FF7Fight", "rb");
	fread(fb_addr, 1024*512*2, 1, fd);
	
	//uint8_t my_buf [1101824];
	//fread(my_buf,1024*512,2,fd);
	//memcpy(fb_addr, my_buf, 1101824);
	
	//for (int y=0; y < 512; y++) {
	   //fread(&((unsigned int * )fb_addr)[y*512 + 128], 512 * 2, 1, fd); // y*512 because fb_addr is a u32 type, isnt it ?
	//}
	fclose(fd);
	*/
	
	/*
	writeRaw(0x02FFFFFF);	// GP0(02h) FillVram / Colour.
	writeRaw(0x00000000);
	writeRaw(0x00040010);	// xpos.bit0-3=0Fh=bugged  xpos.bit0-3=ignored.
	*/

	
	//parser( "/media/fat/DumpSet/FF7Station2_export", (u16*)fb_addr, mgr, 0);

	usleep(5000);
	

	/*
	uint32_t offset = 0x0;
	printf("Displaying FB offset: 0x%08X...\n", offset);
	for (int i=0; i<4096; i++) {
		if ( (i&7)==0 ) printf("\n");
		reg0 = *((uint32_t *)fb_addr+offset+i); printf("%08X ", reg0);
	}
	printf("\n");
	*/

	/*
	printf("printing lines 0-31...\n");
	for (int line=0; line<32; line++) {
		for (int word=0; word<16; word++) { reg0 = *((uint32_t *)fb_addr+word+(line*512) ); printf("%08X ", reg0); }	// 32-bit word address for fb_addr, 2 pixels per word, so 1024/2.
		printf("\n");
	}
	*/
	
	// Read flags / myDebugCnt.
	/*
	reg0 = *((uint32_t *)axi_addr+0x000000002);
	printf("flag bits:  0x%01X\n", (reg0&0xF0000000)>>28 );
	printf("myDebugCnt: 0x%08X\n", reg0&0x0FFFFFFF);
	*/
	
	//*((uint32_t *)axi_addr+0x1) = 0xDEADBEEF; // Write to GP1.
	//*((uint32_t *)axi_addr+0x2) = 0x40000000; // Write to DMA_ACK / gpu_nrst. (keep gpu_nrst HIGH!)
	
	//reg0 = *((uint32_t *)axi_addr+0x3);		 // Read DMA (no gpuSel nor read/write pulse).
	//*((uint32_t *)axi_addr+0x3) = 0xCAFEBABE; // Write DMA (no gpuSel nor read/write pulse).
	
	return 0;
}

