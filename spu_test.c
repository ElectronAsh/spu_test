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

typedef unsigned short u16;
typedef unsigned int   u32;

/*
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


void writeRaw(uint16_t addr, uint16_t data) {
	volatile uint32_t* BASE_SPU = (uint32_t *)axi_addr;	// Address is shifted right by 2 bits on the SPU test core now.
												// (because AXI is set up for 32-bit aligned access, and seems to ignore the two LSB bits of the address.)
	
	//while ( !(BASE_SPU[2] & 1<<28) );	// Wait for dbg_canWrite flag (High) before sending data. If LOW, then stay in while loop. TODO: Add timeout?
	BASE_SPU[addr&0x3ff] = data;
}

void waitForDMAready() {
	volatile uint32_t* SPU_STAT = (uint32_t *)(axi_addr+0x1ae);
	while ( (*SPU_STAT) & 1<<10 );	// Stay in the while loop if the Data Transfer Busy Flag (bit 10) is high.
}

void waitForFIFOnotFull() {
	volatile uint32_t* FLAGS = (uint32_t *)(axi_addr+0x8);
	while ( (*FLAGS) & 1<<31 );		// Stay in the while loop if the isFIFOFull Flag (bit 31) is high.
}

void SPUreset() {
	volatile uint32_t* BASE_SPU = (uint32_t *)axi_addr;	// Address is shifted right by 2 bits on the SPU test core now.
	BASE_SPU[0x0800] = 0x0000;
}

void SPUrun() {
	volatile uint32_t* BASE_SPU = (uint32_t *)axi_addr;	// Address is shifted right by 2 bits on the SPU test core now.
	BASE_SPU[0x0800] = 0x0001;
}



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

void spu_interp(const char* fileName) {
	uint32_t bytesWritten = 0;
	uint32_t lastConfig = 0x8000;
	
	FILE* binSrc = fopen(fileName,"rb");
	
	fseek(binSrc, 0L, SEEK_END);
	u32 file_size = ftell(binSrc);
	fseek(binSrc, 0L, SEEK_SET);
	printf("file_size: %d bytes.\n", file_size);
	
    for (uint32_t ptr=0; ptr<file_size; ) {
        uint8_t opcode/* = spu_dump_bin[ptr++]*/;
		fread(&opcode, sizeof(uint8_t),1, binSrc);
		ptr++;
		
        if (opcode == 'X') { 
            if (ptr>1) return;
        }else if (opcode == 'V') {
            //VSync(0);
			usleep(16666);
        }else if (opcode == 'W') {
            uint32_t addr = 0;
			fread(&addr, sizeof(uint32_t),1, binSrc);
			ptr+=4;

            uint16_t data = 0;
			fread(&data, sizeof(uint16_t),1, binSrc);
			ptr+=2;
			
			if (addr==0x1f801daa /* && (data&0x20)*/) {
				/*
				data = data&0xffcf;
				data = data|0x10;
				writeRaw(addr&0xffff, data);
				*/
				lastConfig = data & (~(3<<4));
				writeRaw(addr, lastConfig); // Clear bit 4-5
			} else {
				//printf("ptr: %08d  W (Write reg) addr: 0x%08X  data: 0x%04X\n", ptr, addr, data);
				writeRaw(addr, data);	// writeRaw only needs the lower 16 bits of the address.
			}
        }else if (opcode == 'F') {
            uint16_t size = 0;
			fread(&size, sizeof(uint16_t),1, binSrc);
			ptr+=2;
			//printf("ptr: %08d  F (write FIFO) size: 0x%04X...\n", ptr, size);
			
            for (uint32_t i = 0; i<size; i++) {
                uint16_t data = 0;
				fread(&data, sizeof(uint16_t),1, binSrc);
				ptr+=2;
				
				//printf("ptr: %08d  (write FIFO) data: 0x%04X\n", ptr, data);
				writeRaw(0x1DA8, data);	//W(SPU_FIFO, data);

                bytesWritten++;
                if ((bytesWritten % 32 == 0) || ((i+1)==size)) {
					writeRaw(0x1daa, lastConfig | (1<<4)); // MANUAL WRITE
					waitForDMAready();
                }
            }
        } else {
            printf("Unknown opcode %c (%d) @ 0x%x, breaking\n", opcode, opcode, ptr-1);
            return;
        }
    }
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

/*
SPU Voice 0..23 Registers
  1F801C00h+N*10h 4   Voice 0..23 Volume Left/Right
  1F801C04h+N*10h 2   Voice 0..23 ADPCM Sample Rate
  1F801C06h+N*10h 2   Voice 0..23 ADPCM Start Address
  1F801C08h+N*10h 4   Voice 0..23 ADSR Attack/Decay/Sustain/Release
  1F801C0Ch+N*10h 2   Voice 0..23 ADSR Current Volume
  1F801C0Eh+N*10h 2   Voice 0..23 ADPCM Repeat Address
SPU Control Registers
  1F801D80h 4  Main Volume Left/Right
  1F801D84h 4  Reverb Output Volume Left/Right
  1F801D88h 4  Voice 0..23 Key ON (Start Attack/Decay/Sustain) (W)
  1F801D8Ch 4  Voice 0..23 Key OFF (Start Release) (W)
  1F801D90h 4  Voice 0..23 Channel FM (pitch lfo) mode (R/W)
  1F801D94h 4  Voice 0..23 Channel Noise mode (R/W)
  1F801D98h 4  Voice 0..23 Channel Reverb mode (R/W)
  1F801D9Ch 4  Voice 0..23 Channel ON/OFF (status) (R)
  1F801DA0h 2  Unknown? (R) or (W)
  1F801DA2h 2  Sound RAM Reverb Work Area Start Address
  1F801DA4h 2  Sound RAM IRQ Address
  1F801DA6h 2  Sound RAM Data Transfer Address
  1F801DA8h 2  Sound RAM Data Transfer Fifo
  1F801DAAh 2  SPU Control Register (SPUCNT)
  1F801DACh 2  Sound RAM Data Transfer Control
  1F801DAEh 2  SPU Status Register (SPUSTAT) (R)
  1F801DB0h 4  CD Volume Left/Right
  1F801DB4h 4  Extern Volume Left/Right
  1F801DB8h 4  Current Main Volume Left/Right
  1F801DBCh 4  Unknown? (R/W)
SPU Reverb Configuration Area
  1F801DC0h 2  dAPF1  Reverb APF Offset 1
  1F801DC2h 2  dAPF2  Reverb APF Offset 2
  1F801DC4h 2  vIIR   Reverb Reflection Volume 1
  1F801DC6h 2  vCOMB1 Reverb Comb Volume 1
  1F801DC8h 2  vCOMB2 Reverb Comb Volume 2
  1F801DCAh 2  vCOMB3 Reverb Comb Volume 3
  1F801DCCh 2  vCOMB4 Reverb Comb Volume 4
  1F801DCEh 2  vWALL  Reverb Reflection Volume 2
  1F801DD0h 2  vAPF1  Reverb APF Volume 1
  1F801DD2h 2  vAPF2  Reverb APF Volume 2
  1F801DD4h 4  mSAME  Reverb Same Side Reflection Address 1 Left/Right
  1F801DD8h 4  mCOMB1 Reverb Comb Address 1 Left/Right
  1F801DDCh 4  mCOMB2 Reverb Comb Address 2 Left/Right
  1F801DE0h 4  dSAME  Reverb Same Side Reflection Address 2 Left/Right
  1F801DE4h 4  mDIFF  Reverb Different Side Reflection Address 1 Left/Right
  1F801DE8h 4  mCOMB3 Reverb Comb Address 3 Left/Right
  1F801DECh 4  mCOMB4 Reverb Comb Address 4 Left/Right
  1F801DF0h 4  dDIFF  Reverb Different Side Reflection Address 2 Left/Right
  1F801DF4h 4  mAPF1  Reverb APF Address 1 Left/Right
  1F801DF8h 4  mAPF2  Reverb APF Address 2 Left/Right
  1F801DFCh 4  vIN    Reverb Input Volume Left/Right
SPU Internal Registers
  1F801E00h+N*04h  4 Voice 0..23 Current Volume Left/Right
  1F801E60h      20h Unknown? (R/W)
  1F801E80h     180h Unknown? (Read: FFh-filled) (Unused or Write only?)
*/


int main()
{
	uint32_t reg0, reg1, reg2, reg3, reg4, reg5, reg6, reg7;

	mmap_setup();

	//GPUManager mgr( ((uint32_t *)axi_addr) );
	
/*	
ADR +0 = Write / Read SPU
ADR +4 = Write / Read SPU
ADR +8 = Bit 31 SPUDACK (write).
         Bit 30 SPU_NRST (Write).
         Bit 29 SPUINT (Read).
         Bit 28 SPUDREQ (Read).
         Bit 27:0 (not used).  
		 
ADR +12= Read Data bus (cpuDataOut), without any other CPU signal.
         Write Data bus (cpuDataIn) + SPUDACK = true.
*/

// wire [31:0] flag_data = {2'b00, SPUINT, SPUDREQ, 28'h0000000 };


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
1F801DAAh - SPU Control Register (SPUCNT)
  15    SPU Enable              (0=Off, 1=On)       (Don't care for CD Audio)
  14    Mute SPU                (0=Mute, 1=Unmute)  (Don't care for CD Audio)
  13-10 Noise Frequency Shift   (0..0Fh = Low .. High Frequency)
  9-8   Noise Frequency Step    (0..03h = Step "4,5,6,7")
  7     Reverb Master Enable    (0=Disabled, 1=Enabled)
  6     IRQ9 Enable (0=Disabled/Acknowledge, 1=Enabled; only when Bit15=1)
  5-4   Sound RAM Transfer Mode (0=Stop, 1=ManualWrite, 2=DMAwrite, 3=DMAread)
  3     External Audio Reverb   (0=Off, 1=On)
  2     CD Audio Reverb         (0=Off, 1=On) (for CD-DA and XA-ADPCM)
  1     External Audio Enable   (0=Off, 1=On)
  0     CD Audio Enable         (0=Off, 1=On) (for CD-DA and XA-ADPCM)
*/

	SPUreset();		// Reset the SPU, by clearing SPU_NRST on bit [0].
	usleep(10);
	SPUrun();		// Bring the SPU out of reset, by setting SPU_NRST on bit [0].
	usleep(10);

	//writeRaw(0x1DAA, 0x0010);	// SPU Control Register (SPUCNT). [15]=DISABLE SPU! [14]=MUTE! [5:4]=b01 (Manual Write).
	//writeRaw(0x1DAC, 0x0004);	// Sound RAM Data Transfer Control (should be 0004h).
	//writeRaw(0x1DA6, 0x0200);	// 0x1000/8. Sound RAM Data Transfer Start Address.

	//spu_interp("/media/fat/bios-sound.bin");
	spu_interp("/media/fat/bios-no-reverb.spudump");
	//spu_interp("/media/fat/crash1.spudump");
	//spu_interp("/media/fat/ff7-101-the-prelude.spudump");
	//spu_interp("/media/fat/metal-slug-x-03-Judgement.spudump");
	
	//writeRaw(0x1DAA, 0xC010);	// SPU Control Register (SPUCNT). [15]=ENABLE SPU. [14]=UNMUTE. [5:4]=b01 (Manual Write).

	/*
	writeRaw(0x1DAA, 0xC000);	// SPU Control Register (SPUCNT). [15]=ENABLE SPU. [14]=UNMUTE. [5:4]=b00.
	
	writeRaw(0x1D80, 0x3FFF);	// Main Volume LEFT.
	writeRaw(0x1D82, 0x3FFF);	// Main Volume RIGHT.

	writeRaw(0x1D90, 0x0000);	// Voice 0. Pitch Modulation Enable Flags (PMON).
	
	writeRaw(0x1DB0, 0x0000);	// CD Audio Input Volume (for normal CD-DA, and compressed XA-ADPCM).
	writeRaw(0x1DB4, 0x0000);	// External Audio Input Volume.
	writeRaw(0x1DB8, 0x3FFF);	// Current Main Volume Left/Right.

	writeRaw(0x1C00, 0x3FFF);	// Voice 0 Volume Left.  (-4000h..+3FFFh = Volume -8000h..+7FFEh)
	writeRaw(0x1C02, 0x3FFF);	// Voice 0 Volume Right. (-4000h..+3FFFh = Volume -8000h..+7FFEh)
	writeRaw(0x1C04, 0x1000);	// Voice 0. ADPCM Sample Rate [15:0]. 1000h = 44,100Hz.
	
	writeRaw(0x1C06, 0x0000);	// Voice 0. Sample Start Address [15:0].	
	writeRaw(0x1C0E, 0x8000);	// Voice 0. Sample Repeat Address [15:0]
	
	writeRaw(0x1C08, 0x000f);	// Voice 0. ADSR Lower. [15]=Attack Exp.  [14:0]=Attack shift. [9:8]=Attack Step. [7:4]=Decay shift. [3:0]=Sustain Level.
	writeRaw(0x1C0A, 0x0000);	// Voice 0. ADSR Upper. [31]/15?=Sustain Exp. [30]/14?=Sustain Dir. [29]/13?=0. [28:24]/12:8?=Sustain shift. [23:22]/7:6?=Sustain step. [21]/5?=Release Mode. [20:16]/4:0?=Release shift.
	
	writeRaw(0x1C0C, 0x0000);	// Voice 0. Current ADSR Volume (R/W).
	
	writeRaw(0x1D88, 0x0001);	// Voice 0. KON (Key ON). [23:0]=Voice 23 to 0.
	
	//writeRaw(0x1E00, 0x3FFF);	// Voice 0. Current Volume Left/Right. 31:16=Right. 15:0=Left. (-8000h to +7FFFh).
	*/
	
	//usleep(40000);

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


