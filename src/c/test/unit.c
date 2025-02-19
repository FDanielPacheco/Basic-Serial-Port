// For these tests is required an real hardware (eg. Arduino UNO)

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdbool.h>
#include <signal.h>
#include <sys/wait.h>

#include "basic_serial.h" 
#include <check.h>

char pathname[ 128 ];

serial_t *
boot( ){
  serial_t * serial = (serial_t *) malloc( sizeof(serial_t) );
  serial_config_t config;
  serial_default_config( &config );
  config.baudrate = B19200;
  config.timeout = 1;

  if( -1 == serial_open( pathname, &config, serial) ){
    free( serial );
    return NULL;
  }

  serial_set_line_state( SERIAL_DTR, false, serial );
  usleep( 1e3 );
  serial_set_line_state( SERIAL_DTR, true, serial );
  usleep( 2e6 );

  return serial;
}

START_TEST(test_serial_open_close_valid){
  serial_t serial;
  serial_config_t config;

  serial_default_config( &config );
  ck_assert_int_eq( serial_open(  pathname, &config, &serial), 0 );
  ck_assert_int_eq( serial_close( &serial ), 0 );
}
END_TEST

START_TEST(test_serial_readLine_valid){
  serial_t * serial = boot( );
  if( NULL == serial )
    return;

  char buf[64];
  serial_write( serial, "UTEST:WRITE_LF\n" );
  const char expected_response[ ] = "UTEST:OK:WRITE_LF\n";
  
  int len = serial_readLine( buf, sizeof(buf), 0, serial);

  ck_assert_str_eq( buf, expected_response );  
  ck_assert_int_eq( len, (int)strlen(expected_response) );
  
  serial_close( serial );
  free( serial );
}
END_TEST

Suite *
serial_suite( void ){
  Suite *s;
  TCase *tc_core;

  s = suite_create("Serial");
  tc_core = tcase_create("Core");

  tcase_add_test(tc_core, test_serial_open_close_valid);
  tcase_add_test(tc_core, test_serial_readLine_valid);

  suite_add_tcase(s, tc_core);
  return s;
}

int 
main( int argc, char ** argv ){
  if( 3 > argc){
    printf("Not enough arguments, perform the tests by doing ./unit.out -p \"<path>\"\n");
    return EXIT_FAILURE;
  }

  for( int i = 0 ; i < argc ; ++i ){
    if( !strcmp( argv[i], "-p" ) )
      strcpy( pathname, argv[i+1] );
  }
  
  Suite *s = serial_suite( );
  SRunner *runner = srunner_create( s );
  
  srunner_run_all( runner, CK_NORMAL );
  int number_failed = srunner_ntests_failed( runner );
  srunner_free( runner );

  return( number_failed == 0 ) ? EXIT_SUCCESS : EXIT_FAILURE;
}