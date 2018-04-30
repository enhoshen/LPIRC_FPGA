/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"

#include "hps_0.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define D2S_BASE 0x03020000
#define S2D_BASE 0x03040000
#define SRAM_SIZE  ONCHIP_MEMORY2_0_SIZE_VALUE
#define SRAM_BW ONCHIP_MEMORY2_0_MEMORY_INFO_MEM_INIT_DATA_WIDTH
typedef enum { D2S, S2D } memdir;
void* virtual_map( void* ,int);
void* dma_send(void* , memdir,int,int,int );
int bit_ext( uint32_t, int);
enum { STATUS=0 , RADDR=1 , WADDR=2, LEN =3,CTL=6} DMAREG;
enum { DONE=0,BUSY=1,REOP=2,WEOP=3,LEN=4 }STABIT;
#define DMAGO 0x0000009c
// 0 BYTE
// 0 HW 
// 1 WORD
// 1 GO
// 1 I_EN
// 0 REEN
// 0 WEEN
// 1 LEEN
// 0 RCON
// 0 WCON
// 0 DOUBLEWORD
// 0 QUADWORD
// 0 SOFTWARERESET

uint32_t set_bit( uint32_t x , int pos , ){
    x= x>>pos;
    return x & 0x00000001;	
}
void* virtual_map ( void* VT_BASE,int IP_BASE){
    return (void*)(VT_BASE + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + IP_BASE ) & ( unsigned long)( HW_REGS_MASK ) ) );
}
void* dma_send(void* VT_BASE, memdir dir,int from, int to,int range){
	int DMA_BASE = (dir == D2S)? D2S_BASE : S2D_BASE;
	dma_addr = virtual_map ( VT_BASE, DMA_BASE);
	while( (*(uint32_t*)(dma_addr+STATUS), 1) == 1) ; // pending
	*(uint32_t *)(dma_addr+RADDR  )= from;
	*(uint32_t *)(dma_addr+WADDR  )=  to;
	*(uint32_t *)(dma_addr+LEN    )= range;
	*(uint32_t *)(dma_addr+CTL    )= DMAGO;
    return dma_addr;
}


int main() {

	void *virtual_base;
	int fd;
	int loop_count;
	int led_direction;
	int led_mask;
	void *h2p_lw_led_addr;

	// map the address space for the LED registers into user space so we can interact with them.
	// we'll actually map in the entire CSR span of the HPS since we want to access various registers within that span

	if( ( fd = open( "/dev/mem", ( O_RDWR | O_SYNC ) ) ) == -1 ) {
		printf( "ERROR: could not open \"/dev/mem\"...\n" );
		return( 1 );
	}

	virtual_base = mmap( NULL, HW_REGS_SPAN, ( PROT_READ | PROT_WRITE ), MAP_SHARED, fd, HW_REGS_BASE );

	if( virtual_base == MAP_FAILED ) {
		printf( "ERROR: mmap() failed...\n" );
		close( fd );
		return( 1 );
	}
	
	h2p_lw_led_addr=virtual_base + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + LED_PIO_BASE ) & ( unsigned long)( HW_REGS_MASK ) );
	

	// toggle the LEDs a bit

	loop_count = 0;
	led_mask = 0x01;
	led_direction = 0; // 0: left to right direction
	while( loop_count < 5 ) {
		
		// control led
		*(uint32_t *)h2p_lw_led_addr = ~led_mask; 

		// wait 100ms
		usleep( 100*1000 );
		
		// update led mask
		if (led_direction == 0){
			led_mask <<= 1;
			if (led_mask == (0x01 << (LED_PIO_DATA_WIDTH-1)))
				 led_direction = 1;
		}else{
			led_mask >>= 1;
			if (led_mask == 0x01){ 
				led_direction = 0;
				loop_count++;
			}
		}
		
	} // while
	enum  { stop , work }state ;
	enum state s;
	int s = 0;
    int* test_arr = int[SRAM_SIZE];	
	for (int i=0 ; i<SRAM_SIZE ; ++i){
        test_arr[i]=i;
	}
	
	while( s!=stop ){
		printf("enter range")
		int start,end;
		scanf("%d",&start);
		scanf("%d",&end);
		int dma_send=alt_avalon_dma_send( 
		
	}
	

	// clean up our memory mapping and exit
	
	if( munmap( virtual_base, HW_REGS_SPAN ) != 0 ) {
		printf( "ERROR: munmap() failed...\n" );
		close( fd );
		return( 1 );
	}

	close( fd );

	return( 0 );
}
