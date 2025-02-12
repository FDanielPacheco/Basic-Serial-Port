 /***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Introduction
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @file      basic_serial.c
 * 
 * @version   1.0
 *
 * @date      12-02-2025
 *
 * @brief     Functions providing control, local settings, input, and output for serial ports on Linux, library developed for interacting with embedded systems in mind (non-cannonical).  
 *  
 * @author    Fábio D. Pacheco, 
 * @email     fabio.d.pacheco@inesctec.pt or pacheco.castro.fabio@gmail.com
 *
 * @copyright Copyright (c) [2025] [Fábio D. Pacheco]
 * 
 * @note      Manuals:
 *            https://man7.org/linux/man-pages/man2/TIOCMSET.2const.html , 
 *            https://people.na.infn.it/~garufi/didattica/CorsoAcq/SerialProgrammingInPosixOSs.pdf
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

/***************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************
 * Imported libraries
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/

#include "basic_serial.h"

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Initializes the serial port interface.
 *  
 * @param[in]  pathname The pathname for the file correspondent to the device, example "/dev/ttyUSB0"
 * @param[in]  config The serial port configuration data structure, this will open the port according to the configuration passed. 
 * @param[out] serial The serial port structure.
 * 
 * @return Upon success performing the opening of the serial port returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_open( const char *pathname, const serial_config_t * config, serial_t * serial  ){
  if( !pathname || !config || !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial and/or pathname and/or config are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( strlen(pathname) >= PATH_MAX ){
    errno = ENAMETOOLONG;
    fprintf(stderr, "ERROR: pathname is greater (%ld) than PATH_MAX at line %d in file %s\n", strlen(pathname), __LINE__, __FILE__);
    return -1;
  }
  
  if( true == config->readonly )
    serial->pointer.fd = open( serial->pathname, O_RDONLY | O_NOCTTY );
  else 
    serial->pointer.fd = open( serial->pathname, O_RDWR | O_NOCTTY );

  if( -1 == serial->pointer.fd ){
    fprintf(stderr, "ERROR:  failed to open %s: %s, at line %d in file %s\n", pathname, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  if( true == config->readonly )
    serial->pointer.fp = fdopen( serial->pointer.fd, "r" );
  else   
    serial->pointer.fp = fdopen( serial->pointer.fd, "r+" );

  if( NULL == serial->pointer.fp ){
    fprintf(stderr, "ERROR:  failed to fdopen %s: %s, at line %d in file %s\n", pathname, strerror(errno), __LINE__, __FILE__);
    return -1;
  } 

  strncpy( serial->pathname, pathname, sizeof(serial->pathname) - 1 );
  
  if( -1 == serial_config_update( config, serial ) ){
    fprintf(stderr, "ERROR:  failed to configure the serial port, at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Closes the connection with the serial port interface.
 *  
 * @param[in] serial The serial port structure associated with desired port.
 * 
 * @return Upon success performing the opening of the serial port returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_close( serial_t * serial ){
  if( !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  if( EOF == fclose( serial->pointer.fp ) ){
    fprintf(stderr, "ERROR:  failed to fclose, %s, at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Sets to the default values the configuration structure passed as argument.
 *  
 * @param[in] config The serial port configuration structure.
 * 
 * @return Upon success performing the opening of the serial port returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_default_config( serial_config_t * config ){
  if( !config ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config is 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }
  config->baudrate = B9600;
  config->dataBits = 8;
  config->flow = NONE;
  config->minBytes = 0;
  config->parity = NONE;
  config->readonly = false;
  config->stopBits = 1;
  config->timeout = 100;

  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the termios parity structure passed as argument.
 *  
 * @param[in] parity The value of parity, options are: `NONE`, `ODD` or `EVEN`.
 * @param[out] tty The termios structure associated with the serial port.
 * 
 * @return Upon success performing the update on the configuration of the termios returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_config_change_parity( const uint8_t parity, struct termios * tty ){
  if( !tty ){
    errno = EINVAL;
    return -1;
  }
  
  switch( parity ){
    default:
      errno = EINVAL;
      return -1;
      
    case NONE:
      tty->c_cflag &= ~(PARENB);                                                //!< Disable parity (Clear bit)
      tty->c_iflag &= ~(INPCK);                                                 //!< Disable parity checking
      break;
    
    case ODD:
      tty->c_cflag |= (PARENB) | (PARODD);                                      //!< Enable parity (Set bit) and Enable odd parity
      tty->c_iflag |= (INPCK);                                                  //!< Enable parity checking
      break;


    case EVEN:
      tty->c_cflag |= (PARENB);                                                 //!< Enable parity (Set bit)
      tty->c_cflag &= ~(PARODD);                                                //!< Enable even parity
      tty->c_iflag |= (INPCK);                                                  //!< Enable parity checking
      break;
  }
  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the termios stop bits structure passed as argument.
 *  
 * @param[in] stopBits The value of stopBits, option are: `1` or `2`.
 * @param[out] tty The termios structure associated with the serial port.
 * 
 * @return Upon success performing the update on the configuration of the termios returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_config_change_stopbits( const uint8_t stopBits, struct termios * tty ){
  if( !tty ){
    errno = EINVAL;
    return -1;
  }

  switch( stopBits ){
    default:
      errno = EINVAL;
      return -1;

    case 1:
      tty->c_cflag &= ~(CSTOPB);                                               //!< Set 1 stop bit 
      break;

    case 2:      
      tty->c_cflag |= (CSTOPB);                                                //!< Set 2 stop bits
      break;
  }
  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the termios data bits structure passed as argument.
 *  
 * @param[in] dataBits The value of dataBits, option are: `5`, `6`, `7` or `8`.
 * @param[out] tty The termios structure associated with the serial port.
 * 
 * @return Upon success performing the update on the configuration of the termios returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_config_change_databits( const uint8_t dataBits, struct termios * tty ){
  if( !tty ){
    errno = EINVAL;
    return -1;
  }

  tty->c_cflag &= ~CSIZE;                                                      //!< Clear all the size bits, then use one of the statements below
  switch( dataBits ){
    default:
      errno = EINVAL;
      return -1;

    case 5:
      tty->c_cflag |= CS5;                                                     //!< 5 bits per byte
      break;
    
    case 6:
      tty->c_cflag |= CS6;                                                     //!< 6 bits per byte
      break;
    
    case 7:
      tty->c_cflag |= CS7;                                                     //!< 7 bits per byte
      break;
    
    case 8:
      tty->c_cflag |= CS8;                                                     //!< 8 bits per byte
      break;
  }
  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the termios flow control structure passed as argument.
 *  
 * @param[in] flowControl The constant of the flow control, option are: `NONE`, `HARDWARE` or `SOFTWARE`.
 * @param[out] tty The termios structure associated with the serial port.
 * 
 * @return Upon success performing the update on the configuration of the termios returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_config_change_flowcontrol( const uint8_t flowControl, struct termios * tty ){
  if( !tty ){
    errno = EINVAL;
    return -1;
  }

  switch( flowControl ){
    default:
      errno = EINVAL;
      return -1;

    case NONE:
      tty->c_iflag &= ~(IXON | IXOFF | IXANY);
      // tty->c_cflag &= ~(CRTSCTS);
      tty->c_cc[VSTART] = 0;                                                   //!< Disable start character (XON) - disable software flow control
      tty->c_cc[VSTOP] = 0;                                                    //!< Disable stop character (XOFF) - disable software flow control
      break;

    case HARDWARE:
      tty->c_iflag &= ~(IXON | IXOFF | IXANY);
      // tty->c_cflag |= (CRTSCTS);
      tty->c_cc[VSTART] = 0;                                                   //!< Disable start character (XON) - disable software flow control
      tty->c_cc[VSTOP] = 0;                                                    //!< Disable stop character (XOFF) - disable software flow control
      break;
    
    case SOFTWARE:
      tty->c_iflag |= (IXON | IXOFF | IXANY);
      // tty->c_cflag &= ~(CRTSCTS);
      tty->c_cc[VSTART] = 1;                                                   //!< Enable start character (XON) - enable software flow control
      tty->c_cc[VSTOP] = 1;                                                    //!< Enable stop character (XOFF) - enable software flow control
      break;
  }
  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the termios other flags and set the timeout and minimum bytes from the structure passed as argument.
 *  
 * @param[in] timeout The time any read function will wait in deciseconds for the information to arrive.
 * @param[in] min The minimum number of bytes to necessary receive before returning the read function.
 * @param[out] tty The termios structure associated with the serial port.
 * 
 * @return Upon success performing the update on the configuration of the termios returns 0. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_config_change_extra( const uint32_t timeout, const uint32_t min, struct termios * tty ){
  if( !tty ){
    errno = EINVAL;
    return -1;
  }

  tty->c_lflag &= ~(ICANON);                                                   //!< Disable canonical mode
  tty->c_lflag &= ~(ECHO);                                                     //!< Disable echo
  tty->c_lflag &= ~(ECHOE);                                                    //!< Disable erasure
  tty->c_lflag &= ~(ECHONL);                                                   //!< Disable new-line echo
  tty->c_lflag &= ~(ISIG);                                                     //!< Disable interpretation of INTR, QUIT and SUSP
  tty->c_oflag &= ~(OPOST);                                                    //!< Set to raw output
  tty->c_oflag &= ~(ONLCR);                                                    //!< Disable the conversion of new line to CR/LF
  tty->c_iflag &= ~(IGNBRK);                                                   //!< Disable ignore break condition
  tty->c_iflag &= ~(BRKINT);                                                   //!< Disable send a SIGINT when a break condition is detected
  tty->c_iflag &= ~(INLCR);                                                    //!< Disable map NL to CR
  tty->c_iflag &= ~(IGNCR);                                                    //!< Disable ignore CR
  tty->c_iflag &= ~(ICRNL);                                                    //!< Disable map CR to NL
  tty->c_cc[VEOF]     = 4;                                                     //!< Set EOF character to EOT (Ctrl+D, ASCII 4) - or 0 if not used
  tty->c_cc[VTIME]    = timeout;                                               //!< Set timeout for read() in tenths of a second
  tty->c_cc[VMIN]     = min;                                                   //!< Set minimum number of bytes for read() to return
  tty->c_cc[VINTR]    = 0;                                                     //!< Disable interrupt character (Ctrl+C)
  tty->c_cc[VQUIT]    = 0;                                                     //!< Disable quit character (Ctrl+\)
  tty->c_cc[VERASE]   = 0;                                                     //!< Disable erase character (backspace) - not relevant in raw mode
  tty->c_cc[VKILL]    = 0;                                                     //!< Disable kill character (Ctrl+U) - not relevant in raw mode
  tty->c_cc[VSWTC]    = 0;                                                     //!< Disable switch character - not usually needed
  tty->c_cc[VSUSP]    = 0;                                                     //!< Disable suspend character (Ctrl+Z)
  tty->c_cc[VEOL]     = 0;                                                     //!< Disable end-of-line character - not relevant in raw mode
  tty->c_cc[VREPRINT] = 0;                                                     //!< Disable reprint character - not relevant in raw mode
  tty->c_cc[VDISCARD] = 0;                                                     //!< Disable discard character - not relevant in raw mode
  tty->c_cc[VWERASE]  = 0;                                                     //!< Disable word erase character - not relevant in raw mode
  tty->c_cc[VLNEXT]   = 0;                                                     //!< Disable literal next character - not relevant in raw mode
  tty->c_cc[VEOL2]    = 0;                                                     //!< Disable alternate end-of-line character - not relevant in raw mode
  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Updates the serial port configuration based on the configuration structure passed as argument.
 *  
 * @param[in] config The serial port configuration structure.
 * @param[out] serial The serial port structure associated.
 * 
 * @return Upon success performing the update on the configuration of the serial port returns 0 and updates the serial structure. Otherwise, -1 is returned and `errno` is set to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t 
serial_config_update( const serial_config_t * config, serial_t * serial ){
  if( !config || !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: config and/or serial are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }

  struct termios tty;

  int result = tcgetattr( serial->pointer.fd, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  result = serial_config_change_parity( config->parity, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  result = serial_config_change_stopbits( config->stopBits, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  result = serial_config_change_databits( config->dataBits, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
 result = cfsetispeed( &tty, config->baudrate );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  result = cfsetospeed( &tty, config->baudrate );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
    
  result = serial_config_change_flowcontrol( config->flow, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  result = serial_config_change_extra( config->timeout, config->minBytes, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: tcgetatrr (%d)(%s) at line %d in file %s\n", result, strerror(errno), __LINE__, __FILE__);
    return -1;
  }
  
  result = tcsetattr( serial->pointer.fd, TCSANOW, &tty );
  if( 0 != result ){
    fprintf(stderr, "ERROR: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  serial->config = *config;  

  return 0;
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Reads a line of data from the serial port (file stream) into a buffer.
 *
 * This function reads data from the file stream associated with the provided `serial` structure,
 * storing it in the buffer pointed to by `buf`, starting at the specified `offset`. Reading
 * stops when a newline character ('\n') is encountered, the buffer is full (up to `size` - 1
 * characters from the start of the buffer), or an error occurs.
 *
 * @param[out] buf The buffer to store the received data.  Must be large enough to hold the expected data,
 *                 taking into account the `offset`.
 * @param[in] size The total size of the buffer `buf`.  This is used to prevent buffer overflows.
 * @param[in] offset The offset within the buffer `buf` where the received data should be stored. This allows
 *                   for appending to existing content in the buffer.
 * @param[in] serial The serial port structure associated with the read operation.  Specifically, the
 *                   file pointer (`serial->pointer.fp`) within this structure is used for reading.
 *
 * @return On success, the function returns 0.
 *         On error, it returns -1 and sets `errno` to indicate the error.
 *         On end-of-file (EOF), it returns 1. Other errors will be printed to `stderr`.
 *
 * @note The received data in `buf` is null-terminated (or as much as possible given the size), even if a newline
 *       character was not received.  If a newline is read, it will be included in the buffer.
 * @note This function uses `fgets()` internally and is designed for reading lines from a file stream associated
 *       with the serial port.
 * @note It is the caller's responsibility to ensure that `size` is large enough to accommodate the data
 *       being read, considering the `offset`.  If the combined length of the existing data in `buf` (up to `offset`)
 *       and the data read exceeds `size`, the function will return -1 with `ENOMEM`.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int8_t
serial_readLine( char * buf, const uint32_t size, const uint32_t offset, serial_t * serial ){
  if( !buf || !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial and/or buf are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }
  
  if( (size < strlen(buf)) || (size < offset) ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: serial_fgets, %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  clearerr( serial->pointer.fp );

  if( NULL == fgets( buf+offset, size-offset, serial->pointer.fp ) ){
    if( feof( serial->pointer.fp ) ){
      clearerr( serial->pointer.fp );
      return 1;
    }
    
    else if( ferror( serial->pointer.fp ) ) 
      fprintf(stderr, "ERROR: fgets failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    else 
      fprintf(stderr, "ERROR: fgets returned NULL for unknown reason at line %d in file %s\n", __LINE__, __FILE__);

    clearerr( serial->pointer.fp );
    return -1;
  }

  return 0;  
}

/**********************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************//**
 * @brief Reads a specified number of bytes from the serial port (file stream) into a buffer.
 *
 * This function reads up to `length` bytes from the file stream associated with the provided
 * `serial` structure and stores them in the buffer pointed to by `buf`, starting at the
 * specified `offset`. The function may return fewer bytes than requested if an end-of-file
 * (EOF) is encountered or a timeout occurs.
 *
 * @param[out] buf The buffer to store the received data. Must be large enough to hold the expected
 *                 data, taking into account the `offset`.
 * @param[in] size The total size of the buffer `buf` in bytes. This is used to prevent buffer overflows.
 * @param[in] offset The offset within the buffer `buf` where the received data should be stored.
 *                   This allows for appending to existing content in the buffer.
 * @param[in] length The maximum number of bytes to read from the serial port.
 * @param[in] serial The serial port structure associated with the read operation. Specifically, the
 *                   file pointer (`serial->pointer.fp`) within this structure is used for reading.
 *
 * @return On success, the function returns the number of bytes actually read (which may be less than
 *         `length` if EOF or a timeout is encountered).
 *         On error, the function returns -1 and sets `errno` to indicate the error.
 * 
 **************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************************/
int32_t
serial_read( char * buf, const uint32_t size, const uint32_t offset, const int32_t length, serial_t * serial ){
  if( !buf || !serial ){
    errno = EINVAL;
    fprintf(stderr, "ERROR: serial and/or buf are 'NULL' at line %d in file %s\n", __LINE__, __FILE__);
    return -1;
  }
  
  if( (size < (strlen(buf) + length)) || (size < (offset + length)) ){
    errno = ENOMEM;
    fprintf(stderr, "ERROR: cserial_read, %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    return -1;
  }

  clearerr( serial->pointer.fp );

  int32_t len = fread( buf+offset, 1, length, serial->pointer.fp );

  if( length > len ){
    if( feof( serial->pointer.fp ) ){
      clearerr( serial->pointer.fp );
      return 1;
    }
    
    else if( ferror( serial->pointer.fp ) ) 
      fprintf(stderr, "ERROR: fgets failed: %s at line %d in file %s\n", strerror(errno), __LINE__, __FILE__);
    else 
      fprintf(stderr, "ERROR: fgets returned NULL for unknown reason at line %d in file %s\n", __LINE__, __FILE__);

    clearerr( serial->pointer.fp );
    return -1;
  }

  return len;  
}
