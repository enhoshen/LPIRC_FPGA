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
#define D2S_BASE DMA_0_BASE
#define S2D_BASE DMA_1_BASE
#define SRAM_SIZE  ONCHIP_MEMORY2_0_SIZE_VALUE
#define SRAM_BW ONCHIP_MEMORY2_0_MEMORY_INFO_MEM_INIT_DATA_WIDTH
#define SRAM_BASE ONCHIP_MEMORY2_0_BASE

typedef uint32_t u32;
typedef enum memdir { D2S, S2D } memdir;

void* virtual_map( void* ,int);
void* dma_send(void* , memdir,u32,u32,int );
u32 bit_ext( u32, int , bool);
bool get_bit ( u32 ,int );
u32 set_bit ( u32 * , int);
enum DMAREG{ STATUS=0 , RADDR=1 , WADDR=2, LENGTH =3,CTL=6} ;
enum STABIT{ DONE=0,BUSY=1,REOP=2,WEOP=3,LEN=4 };

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

inline u32 set_bit( u32 *x , int pos ){
    
    return *x |= 1UL << pos ;  
}
inline bool get_bit( u32 x, int pos ){
    x = x>>pos;
    return x%1;    
}
void* virtual_map ( void* VT_BASE,int IP_BASE){
    return (void*)(VT_BASE + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + IP_BASE ) & ( unsigned long)( HW_REGS_MASK ) ) );
}
void* dma_send(void* VT_BASE, memdir dir,u32 from, u32 to,int range){
    int DMA_BASE = (dir == D2S)? D2S_BASE : S2D_BASE;
    void* dma_addr = virtual_map ( VT_BASE, DMA_BASE);
    while( get_bit(*(u32*)(dma_addr+STATUS), 1) ) ; // pending
    *(u32 *)(dma_addr+RADDR  )= from;
    *(u32 *)(dma_addr+WADDR  )=  to;
    *(u32 *)(dma_addr+LEN    )= range;
    *(u32 *)(dma_addr+CTL    )= DMAGO;
    return dma_addr;
}


int main() {
    alt_int_dist_targe_set(
    void *virtual_base;
    int fd;
    int loop_count;
    int led_direction;
    int led_mask;
    void *h2p_lw_led_addr;

    printf("%d get_bit test" , get_bit( 31,  5) );
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
    typedef enum  { STOP , IDLE, TRANSFER }state ;
    state s;
    s = IDLE;
    int test_arr[SRAM_SIZE];    
    for (int i=0 ; i<SRAM_SIZE ; ++i){
        test_arr[i]=i;
    }
    
    void* sram_base = virtual_map( virtual_base , SRAM_BASE);
    while( s!= STOP ){
        int start, range;
        switch ( s ){
            case IDLE:
                printf("enter start,range");
                scanf("%d",&range);
                scanf("%d",&start);
                dma_send(virtual_base,D2S, (u32)(test_arr+start), (u32)(sram_base+start), range);
            case TRANSFER:
                printf("Transferring");
                if()// interrupt
                    dma_send(virtual_base,S2D, (u32)(sram_base+start)m(u32)(test_arr+start), range);
            case STOP:
                break;
        }
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
