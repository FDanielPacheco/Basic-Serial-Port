#include <stdio.h>
#include <unistd.h>
#include "basic_serial.h"

int
main( void ){
  serial_t sr;
  serial_config_t cfg;
  serial_default_config( &cfg );
  cfg.baudrate = B19200;
  cfg.timeout = 25;
  if( -1 == serial_open( "/dev/ttyACM0", &cfg, &sr ) )
    return -1;

  serial_set_line_state( SERIAL_DTR, false, &sr );
  usleep( 1e3 );
  serial_set_line_state( SERIAL_DTR, true, &sr );
  usleep( 2e6 );
  
  serial_flush( &sr, TCIFLUSH );  
  serial_write( &sr, "Some message\n" );
  serial_drain( &sr );

  char buf[ PATH_MAX ];

  for( ; ; ){
    if( serial_available( &sr ) ){
      serial_readLine( buf, sizeof(buf), 0, &sr );
      printf("%s", buf );
    }

    // Other process
  }

  serial_close( &sr );

  return 0;
}