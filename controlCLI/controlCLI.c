// Loader driver

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>

#include <unistd.h>
#include "serial.h"
//#include "type.h"


enum Command {
    Command_GetVersion = 0x01,
    Command_Start      = 0x02,
    Command_NextPage   = 0x03,
    Command_Reset      = 0x04,

    Command_OK         = 0x40,
    Command_Error      = 0x80,
};

enum State {
    Idle,
    Connecting,
    Connected,
    Starting,
    Sending,
};

#define HEADER_LEN 4 + 4 + 4 + 4 + 16 + 4
#define PAGE_LEN 1024

static ser_handler stm32_ser_id = ( ser_handler )-1;



static FILE *fp;
static u32 fpsize;



// Helper: read a byte from STM32 with timeout
static int stm32h_read_byte()
{
  return ser_read_byte( stm32_ser_id );
}

// Get data function
static u32 writeh_read_data( u8 *dst, u32 len )
{
  size_t readbytes = 0;

  //printf("size:%d",len);

  if( !feof( fp ) )
    readbytes = fread( dst, 1, len, fp );

  //printf("readbytes:%d",readbytes);
  return ( u32 )readbytes;
}



void connect_to_bootloader()
{
  int res = -1;
  int state = Connecting;
  u8 dataBuff[1050];
  int headerLen = 0;
  int p = 0;
  int page = 0;
  int tmpRaedP= 0;

  // Flush all incoming data
  ser_set_timeout_ms( stm32_ser_id, SER_NO_TIMEOUT );
  while(1){

    switch(state) {

      case Connecting:
        printf("Connecting..\n");
        //Command_GetVersion
        int n = 0;
        ser_write_byte( stm32_ser_id, Command_GetVersion );
        while( ((res = stm32h_read_byte()) == -1)  || (res != (Command_GetVersion | Command_OK)) ) //0x41=65
        {
          n++;
          usleep(500000);
          ser_write_byte( stm32_ser_id, Command_GetVersion );
          //printf("Connecting...:%d %d\n",res, n);
        };

        if(res == (Command_GetVersion | Command_OK) ){
          printf("Command_OK | Command_GetVersion  %c  %d\n",res ,res);
          state = Connected;
        }else{
          printf("\n!! No Command_OK | Command_GetVersion !!\n");
          exit(1);
        }
      break;

      case Connected:
        printf("\nConnected\n");
        //read Protocol version 
        n = 4;
        printf("PROTOCOL_VERSION: ");
        while(n)
        {
          usleep(100);
          res = stm32h_read_byte();
          if(res >= 0){
            printf("%x",res);
            n--;
          }
        };
        printf("\n");

        //Read Product ID
        n = 4;
        printf("PRODUCT_ID: ");
        while(n)
        {
          //usleep(100);
          res = stm32h_read_byte();
          if(res >= 0){
            printf("%x",res);
            n--;
          }
        };
        printf("\n");
        state = Starting;
      break;

      case Starting:
        printf("\nStarting\n");
        //Command_Start
        printf("Send start cmd\n");

        headerLen = HEADER_LEN;
        //writeh_read_data(dataBuff, headerLen);
        printf("read from file:%d\n", writeh_read_data(dataBuff+p, headerLen) );

        /*tmpRaedP = 0;
        //printf("file:\n");
        while(tmpRaedP <= headerLen-1){
          printf("%x ", dataBuff[tmpRaedP]);
          tmpRaedP++;
        }
        printf("\n");*/

        usleep(100);
        //ser_write_byte( stm32_ser_id, Command_Start ); //send start command
        //ser_write( stm32_ser_id, dataBuff, headerLen );  //send data
        int error = 0;
        while( ((res = stm32h_read_byte()) == -1) || (res != (Command_Start | Command_OK))) //0x42=66
        {
          usleep(100);
          if(res != -1)
            printf("ERROR:0x%x\n",res);

          error++;
          if(error > 300000){
            printf("ERROR EXIT;0x%x\n",res);
            exit(1);
          }

          if(res == (Command_GetVersion | Command_OK)){
            usleep(500000);
            printf("Resend Command_Start for reply 65\n");
            ser_write_byte( stm32_ser_id, Command_Start ); //send start command
            ser_write( stm32_ser_id, dataBuff, headerLen );  //send data
          }
        };
        state = Sending;
      break;

      case Sending:
        printf("\nSending page %d\n",page);
        error = 0;
        page++;

        headerLen = writeh_read_data(dataBuff, PAGE_LEN);
        printf("read from file:%d\n", headerLen );

        if(headerLen){
          /*tmpRaedP = 0;
          //printf("file:\n");
          while(tmpRaedP <= headerLen-1){
            printf("%x ", dataBuff[tmpRaedP]);
            tmpRaedP++;
          }
          printf("\n");*/

          ser_write_byte( stm32_ser_id, Command_NextPage ); //send start command
          ser_write( stm32_ser_id, dataBuff, headerLen );  //send data
        }else{
          printf("Upgrade Finished\n" );
          ser_write_byte( stm32_ser_id, Command_Reset ); //send reset command
          //ser_write( stm32_ser_id, dataBuff, headerLen );  //send data
          exit(0);
        }


        while( ((res = stm32h_read_byte()) == -1) || (res != (Command_NextPage | Command_OK))) //0x43=67
        {
          usleep(100);
          if(res != -1)
            printf("ERROR2:0x%x\n",res);

          error++;
          if(error> 100000){
            printf("ERROR2 EXIT;0x%x\n",res);
            exit(1);
          }

        }

      break;

      default:
        printf("\nSwitch default\n");
      break;      

    }

  }
}



int main( int argc, const char **argv )
{

  long baud;
 
  // Argument validation
  if( argc < 3 )
  {
    fprintf( stderr, "Usage: controlC <port> <binary crypto image>\n" );
    exit( 1 );
  }
  errno = 0;
  baud = strtol( "115200", NULL, 10 );
  if( ( errno == ERANGE && ( baud == LONG_MAX || baud == LONG_MIN ) ) || ( errno != 0 && baud == 0 ) || ( baud < 0 ) ) 
  {
    fprintf( stderr, "Invalid baud 115200\n" );
    exit( 1 );
  }
  
  if( ( fp = fopen( argv[ 2 ], "rb" ) ) == NULL )
  {
    fprintf( stderr, "Unable to open %s\n", argv[ 2 ] );
    exit( 1 );
  }
  else
  {
    fseek( fp, 0, SEEK_END );
    fpsize = ftell( fp );
    printf("file size:%d\n",fpsize);
    fseek( fp, 0, SEEK_SET );
  }
    
  // Open port
  if( ( stm32_ser_id = ser_open( argv[ 1 ] ) ) == ( ser_handler )-1 )
    return 1;

  // Setup port
  ser_setup( stm32_ser_id, baud, SER_DATABITS_8, SER_PARITY_NONE, SER_STOPBITS_1 ); //EVEN

  connect_to_bootloader();
  
  return 0;
}
           



