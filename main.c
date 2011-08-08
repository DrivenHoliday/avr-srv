#include <avr/interrupt.h>
#include <avr/io.h>
#include <avr/sleep.h>

#include <util/delay.h>

#include <stdint.h>

#define BAUD 19200
#include <util/setbaud.h>

#include "uart.h"

#define FOREVER for(;;)

// uint8_t spi_send_recv(uint8_t byte)
// {
//     SPDR = byte;
//     while(!(SPSR & (1<<SPIF)));
//     return SPDR;
// }
// 
// uint8_t sd_send_cmd(uint8_t *cmd)
// {
//     uint8_t n = 0, res = 0;
//     
//     spi_send_recv(0xff);
//     
//     PORTA &= ~(1<<PA7);
//     
//     asm volatile ("nop");
//     asm volatile ("nop");
//     
//     spi_send_recv(0xff);
//     
//     for(n=0;n<6;++n)
//     {
//         res = spi_send_recv(cmd[n]);
//     }
//     
//     while(res == 0xff)
//     {
//         res = spi_send_recv(0xff);
//     }
//         
//     return res;
// }
// 
// void sd_end_trans()
// {
//     spi_send_recv(0xff);
//     
//     PORTA |= (1<<PA7);
//     
//     asm volatile ("nop");
//     asm volatile ("nop");
//     
//     spi_send_recv(0xff);
//     spi_send_recv(0xff);
// }
// 
// int main(void)
// {
//     uint8_t n = 0, res = 0;
//     uint16_t bn = 0;
//     const uint8_t cmd0[]    = {0x40,0x00,0x00,0x00,0x00,0x95};
//     const uint8_t cmd1[]    = {0x41,0x00,0x00,0x00,0x00,0x01};
//     const uint8_t cmd55[]   = {0x77,0x00,0x00,0x00,0x00,0x01};
//     const uint8_t acmd41[]  = {0x69,0x00,0x00,0x00,0x00,0x01};
//     const uint8_t cmd16[]   = {0x50,0x00,0x00,0x02,0x00,0x01};
//     const uint8_t cmd17[]   = {0x51,0x00,0x00,0xff,0x00,0x01};
//     
//     DDRC = 0xff;
//     PORTC = 0xff;
// 
//     DDRA = 0xff;
//     PORTA = 0xff;
//     
// 	/* Init */
//     uart_init(UBRR_VALUE);
//     
//     DDRB  = 0b11111101;
//     PORTB = 0b00000100;
// 
// 	SPCR = (0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (1<<SPR0) | (1<<SPR1);
//     SPSR &= ~(1<<SPI2X); 
//     
//     _delay_ms(100);
//     
//     for(n=0;n<0x0f;++n)
//     {
//         spi_send_recv(0xff);
//     }
//     
//     res = sd_send_cmd(cmd0);
//     sd_end_trans();
//     
//     if(res == 0x01)
//     {
//         PORTC &= ~(1<<PC0);
//     }
//     else
//     {
//         PORTC &= ~(1<<PC1);
//     }
//     
//     uart_putc(res);
//     
//     res = sd_send_cmd(cmd55);
//     uart_putc(res);
//     
//     while(res == 0x01)
//     {
//         res = sd_send_cmd(acmd41);
//         uart_putc(res);
//         _delay_ms(100);
//     }
//     
//     if(res == 0x05)
//     {
//         do
//         {
//             res = sd_send_cmd(cmd1);
//             uart_putc(res);
//             _delay_ms(100);
//         }
//         while(res == 0x01);
//     }
//     
//     uart_puts("\ninit done\n");
//     
//     res = sd_send_cmd(cmd16);
//     uart_putc(res);
//     
//     res = sd_send_cmd(cmd17);
//     uart_putc(res);
//     
//     uart_puts("\naccessing ..\n");
//     
//     while(res != 0xfe)
//     {
//         res = sd_send_cmd(cmd17);
//         uart_putc(res);
//         _delay_ms(100);
//     }
//     
//     uart_puts("\ndata ..\n");
//     
//     for(bn=0;bn<514;++n)
//     {
//         res = sd_send_cmd(0xff);
//         uart_putc(res);
//         _delay_ms(100);
//     }
// 
//     /* wird nie erreicht */
//     return 0;
// }

// #include "diskio.h"
// 
// int main(void)
// {
//     uint8_t buf[1024];
//     uint16_t n = 0;
//     
//     DDRC = 0xff;
//     PORTC = 0xff;
// 
//     DDRA = 0xff;
//     PORTA = 0xff;
//     
//     /* Init */
//     uart_init(UBRR_VALUE);
//     
//     DDRB  = 0b11111101;
//     PORTB = 0b00000100;
//     
//     uart_putc(disk_initialize(0x00));
//     
//     uart_putc(disk_read(0x00, buf, 0, 2));
//     
//     for(n = 0; n < sizeof(buf); ++n)
//     {
//         uart_putc(buf[n]);
//         _delay_ms(10);
//     }
//     
//     return 0;
// }

// #include <avr/pgmspace.h>
// #include <string.h>
// #include "ff.h"
// #include "diskio.h"
// 
// WCHAR ff_convert (WCHAR ch, UINT ui)
// { return ch; }
// 
// WCHAR ff_wtoupper (WCHAR ch)
// { return ch; }
// 
// int main(void)
// {
// //     DDRD  = 0xff;
// //     PORTD = 0xff;
// //     
// //     DDRC = 0xff;
// //     PORTC = 0xff;
// //     
// //     volatile char bla[10000];
// //     
// //     PORTC &= ~(1<<PC0);
// //     
// //     /* Init */
// //     uart_init(UBRR_VALUE);
// //     
// //     PORTC &= ~(1<<PC1);
// //     
// //     uart_puts("start\n");
//     FRESULT res;
//     FILINFO fno;
//     DIR dir;
//     int i;
//     char *fn;   /* This function is assuming non-Unicode cfg. */
//     char path[] = "0:"; 
//     FATFS fs;   
//     
//     PORTC &= ~(1<<PC0);
//     
// #if _USE_LFN
//     static char lfn[_MAX_LFN *  2];
//     fno.lfname = lfn;
//     fno.lfsize = sizeof(lfn);
// #endif
//     
//     DDRC = 0xff;
//     PORTC = 0xff;
// 
//     DDRA = 0xff;
//     PORTA = 0xff;
//     
//     DDRB  = 0b11111101;
//     PORTB = 0b00000100;
//     
//     /* Init */
//     uart_init(UBRR_VALUE);
//     
//     uart_puts("start\n");
//     
//     disk_initialize(0x00);
//     
//     memset(&fs,0,sizeof(fs));
//     res = f_mount(0x00, &fs);
//     //uart_putc(res);
//     
//     res = f_opendir(&dir, path);                       /* Open the directory */
//     //uart_putc(res);
//     
//     if (res == FR_OK)
//     {
//         i = strlen(path);
//         for (;;)
//         {
//             res = f_readdir(&dir, &fno);                   /* Read a directory item */
//             //uart_putc(res);
//             if (res != FR_OK || fno.fname[0] == 0) break;  /* Break on error or end of dir */
//             if (fno.fname[0] == '.') continue;             /* Ignore dot entry */
// #if _USE_LFN
//             fn = *fno.lfname ? fno.lfname : fno.fname;
// #else
//             fn = fno.fname;
// #endif
// //             if (fno.fattrib & AM_DIR)
// //             {                    /* It is a directory */
// //                 sprintf(&path[i], "/%s", fn);
// //                 res = scan_files(path);
// //                 if (res != FR_OK) break;
// //                 path[i] = 0;
//             /*}
//             else
//             {*/                                       /* It is a file. */
//                 uart_puts(fn);
//                 uart_puts("\n");
//                 //printf("%s/%s\n", path, fn);
// //             }
//             
//             _delay_ms(100);
//         }
//     }
//     
//     uart_puts("error\n");
// 
//     
//     return 0;
// }

unsigned char SPI_ReadWrite( unsigned char Data )
{
    // daten senden
    SPDR = Data;
    // auf fertig warten
    while(!(SPSR & (1<<SPIF)));
    // empfangende daten einlesen
    Data = SPDR;
    // daten zurueckgeben
    return( Data );
}

#define ADDR_MASK   0x1F

void enc28j60WriteOp( unsigned char op, unsigned char address,  unsigned char data){
  // CS aktive setzen
  PORTA &= ~(1<<PA6);
  // schreibcomando senden
  SPI_ReadWrite( op | (address & ADDR_MASK) );
  // daten senden
  SPI_ReadWrite( data );
  // CS wieder freigeben
  PORTA |= (1<<PA6);
}

// void enc28j60SetBank(unsigned char address){
//   // set the bank (if needed)
//   if((address & BANK_MASK) != Enc28j60Bank) {
//     // set the bank
//     enc28j60WriteOp(ENC28J60_BIT_FIELD_CLR, ECON1, (ECON1_BSEL1|ECON1_BSEL0));
//     enc28j60WriteOp(ENC28J60_BIT_FIELD_SET, ECON1, (address & BANK_MASK)>>5);
//     Enc28j60Bank = (address & BANK_MASK);
//   }
// }
// 
// unsigned char enc28j60Read(unsigned char address){
//   // set the bank
//   enc28j60SetBank(address);
//   // do the read
//   return enc28j60ReadOp(ENC28J60_READ_CTRL_REG, address);
// }

int main(void)
{
    DDRC = 0xff;
    PORTC = 0xff;

    DDRA = 0xff;
    PORTA = 0xff;
    
    DDRB  = 0b11111101;
    PORTB = 0b00000100;
    
    /* Init */
    uart_init(UBRR_VALUE);
    
    SPCR = (0<<SPIE) | (1<<SPE) | (0<<DORD) | (1<<MSTR) | (0<<CPOL) | (0<<CPHA) | (1<<SPR0) | (1<<SPR1);
    
    uart_puts("start\n");
    
    ///
    PORTA |= (1<<PA6);
    
    // perform system reset
    enc28j60WriteOp(0xFF, 0, 0xFF);
    
    // check CLKRDY bit to see if reset is complete
    _delay_us(20); 
    _delay_us(20); 
    _delay_us(20); 
    _delay_us(20); 

    uart_puts("done\n");
    
    //while(!(enc28j60Read(ESTAT) & ESTAT_CLKRDY));
    
}