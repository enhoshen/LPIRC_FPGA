/*
This program demonstrate how to use hps communicate with FPGA through light AXI Bridge.
uses should program the FPGA by GHRD project before executing the program
refer to user manual chapter 7 for details about the demo
*/


#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <string.h>
#include "hwlib.h"
#include "socal/socal.h"
#include "socal/hps.h"
#include "socal/alt_gpio.h"

#include "alt_dma.h"
#include "alt_globaltmr.h"

#include "hps_0.h"

#define HW_REGS_BASE ( ALT_STM_OFST )
#define HW_REGS_SPAN ( 0x04000000 )
#define HW_REGS_MASK ( HW_REGS_SPAN - 1 )
#define D2S_BASE DMA_0_BASE
#define S2D_BASE DMA_1_BASE
#define SRAM_SIZE  ONCHIP_MEMORY2_0_SIZE_VALUE
#define SRAM_BW ONCHIP_MEMORY2_0_MEMORY_INFO_MEM_INIT_DATA_WIDTH
#define SRAM_BASE ONCHIP_MEMORY2_0_BASE
#define u32_s 4
typedef uint32_t u32;
typedef enum memdir { D2S, S2D } memdir;
typedef enum DMAREG{ STATUS=0 , RADDR=1 , WADDR=2, LENGTH =3,CTL=6} DMAREG;
typedef enum STABIT{ DONE=0,BUSY=1,REOP=2,WEOP=3,LEN=4 } STABIT;


u32* virtual_map( void* ,void*);
void* dma_send(void* , memdir,u32,u32,u32 );

bool get_bit ( u32 ,int );
u32 set_bit ( u32 * , int);
u32 get_dmareg( u32* base , DMAREG reg);
u32 set_dmareg( u32* base , DMAREG reg, u32 patt );


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

    return (x>>pos)%2;    
}
inline u32 get_dmareg( u32* base , DMAREG reg){
    return *(base+reg);
}
inline u32 set_dmareg( u32* base , DMAREG reg, u32 patt ){
    *(u32*)(base+reg) = patt;
    return patt;
}
u32* virtual_map ( void* VT_BASE,void* IP_BASE){
    return (u32*)(VT_BASE + ( ( unsigned long  )( ALT_LWFPGASLVS_OFST + IP_BASE ) & ( unsigned long)( HW_REGS_MASK ) ) );
}
void* dma_send(void* VT_BASE, memdir dir,u32 from, u32 to,u32 range){
    void* DMA_BASE = (dir == D2S)? (void*)D2S_BASE : (void*)S2D_BASE;
    u32* dma_addr = virtual_map ( VT_BASE, DMA_BASE);
    *(u32 *)(dma_addr+RADDR  )= from;
    *(u32 *)(dma_addr+WADDR  )=  to;
    *(u32 *)(dma_addr+LENGTH )= u32_s*range;
    *(u32 *)(dma_addr+CTL    )= DMAGO;
    return dma_addr;
}
bool memprint( int* addr, u32 start, u32 range){
    for ( int i = 0 ; i<range; ++i)
        printf("addr:%d , val:%d \n", start+i,*(addr+start+i)) ;
    return true;
}
/*
bool memprintvoid( void* addr, u32 start, u32 range){
    for ( int i = 0 ; i<range; ++i)
        printf("addr:%d , val:%d \n", start+u32_s*i,(addr+u32_s*(start+i)));
    return true;
}
*/
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
    
    while( loop_count < 1 ) {
        
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
    typedef enum  { STOP , IDLE, TRAND2S,TRANS2D }state ;
    state s;
    s = IDLE;

    int test_arr[SRAM_SIZE];    
  
    for (int i=0 ; i<SRAM_SIZE ; ++i){
        test_arr[i]=i;
        
    }
    
    u32* sram_base = virtual_map( virtual_base , (void*)SRAM_BASE);
    u32* dma0_base = virtual_map( virtual_base  ,(void*)DMA_0_BASE);
    u32* dma1_base = virtual_map( virtual_base  ,(void*)DMA_1_BASE);

    
    set_dmareg( dma0_base , STATUS, 0);
    set_dmareg( dma1_base , STATUS, 0);
  

    while( s!= STOP ){
        int start, range;
        void* dma_addr ;
        switch ( s ){
            case IDLE:;
                char test ='y';
                printf("go on? (y/n)");
                scanf("%c", &test);
                printf("enter start,range\n");
                scanf("%d",&start);
                printf("\n");
                scanf("%d",&range);
                if( test == 'y')
                    dma_send(virtual_base,D2S, (u32)(test_arr+u32_s*start), (u32)(DMA_0_WRITE_MASTER_ONCHIP_MEMORY2_0_BASE+start), range);
                s= test=='y'? TRAND2S : STOP;
            case TRAND2S:
                dma_addr = virtual_map ( virtual_base, (void*)D2S_BASE);
                //printf("%0x \n", get_bit(get_dmareg(dma_addr,STATUS), BUSY));
                if(get_bit(get_dmareg(dma_addr,STATUS), BUSY) )printf(" dram to sram transferring %0x\n", get_dmareg(dma_addr,LENGTH));
                if(get_bit(get_dmareg(dma_addr,STATUS),DONE)  ){
                    printf("check\n");
                    s=TRANS2D;
                    set_dmareg(dma_addr, STATUS,0);
                    dma_send(virtual_base,S2D, (u32)(DMA_1_READ_MASTER_ONCHIP_MEMORY2_0_BASE+start), (u32)(test_arr+u32_s*start), range);
                }
                break;
            case TRANS2D:
                printf("get data back\n");
                dma_addr = virtual_map ( virtual_base , (void*)S2D_BASE);
                if(get_bit(get_dmareg(dma_addr,STATUS),DONE) ){
                    s=IDLE;
                    set_dmareg(dma_addr, STATUS,0);
                    memprint(test_arr, start, range);
                }
                break;
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
