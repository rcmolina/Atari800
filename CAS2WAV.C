/*****************************************************************************
**
**  Copyright 1998, 1999 by Ernest R. Schreurs.
**  All rights reserved.
**  Use of this source code is allowed under the following conditions:
**  You must inform people that you based your work on this stuff.
**  If you charge a fee in any form for your product, you must inform people
**  that this stuff is available free of charge.
**  Refer to the documentation for more information.
**
*****************************************************************************/
#define MODULE  "CAS2WAV.C"
/*****************************************************************************
**  NAME: CAS2WAV.C
**
**  Author            : Ernest R. Schreurs
**  Date              : January 3, 1999
**  Release           : 01.00
**
**  Description       : This program will convert a .cas cassette image file
**                      to a .wav file.
**
*****************************************************************************/

/*****************************************************************************
==  INCLUDE FILES
*****************************************************************************/
#include <stdio.h>              /* For printf() and gets()              */
#include <ctype.h>              /* For isalnum() and toupper()          */
//#include <fltenv.h>             /* For sin() exceptions                 */
#include <math.h>               /* For sin()                            */
#include <stdlib.h>             /* For the exit function                */
#include <string.h>             /* String and memory stuff              */

/*****************************************************************************
==  DEFINED SYMBOLS
*****************************************************************************/

#ifndef FALSE
#define FALSE               0
#endif
#ifndef TRUE
#define TRUE                1
#endif

#ifndef NULL
#define NULL                0
#endif
#define PATH_LEN            128             /* Maximum path length          */

#define BUF_LEN             80              /* fgets buffer length          */
#define SUCCESS             1               /* Success is non-zero          */
#define FAILURE             0               /* Failure is zero              */
#define ZERO_LEVEL          128             /* Zero level for wav sample    */

/*
**  A tape record starts with a Pre-Record Write Tone.
**  Then we find bytes, usually 132, each starting with a startbit,
**  eight data bits, lsb first, followed by a stopbit.
**  This is all coded in mark and space tones, as defined below.
*/
#define FSK_MARK            1               /* Mark tone represents a 1     */
#define FSK_SPACE           0               /* Space tone represents a 0    */
#define FSK_1               1               /* A 1 bit is a mark            */
#define FSK_0               0               /* A 0 bit is a space           */
#define FSK_PRWT            1               /* PRWT tone is a mark          */
#define FSK_STARTBIT        0               /* Startbit is a space          */
#define FSK_STOPBIT         1               /* Stopbit is a mark            */

/*
**  Definitions for the mark and space tone frequencies.
*/
#define FSK_TONE_MARK       5327            /* Frequency of mark tone       */
#define FSK_TONE_SPACE      3995            /* Frequency of space tone      */
#define MARK_TABLE_LEN      44100L          /* Length of mark tone table    */
#define SPACE_TABLE_LEN     44100L          /* Length of space tone table   */

/*****************************************************************************
==  MACRO DEFINITIONS
*****************************************************************************/
/*
**  Macro for casting stuff to requirements of stupid
**  standard library functions.
*/

#define FGETS( buf, buf_len, file_ptr )                                 \
    (void *)fgets( (char *)buf, (int)buf_len, (FILE *)file_ptr )

#define STRLEN( str )                                                   \
    strlen( (const char *)(str) )

/*
**  Macro for getting input from the terminal,
**  allowing the user to exit with either control Z or
**  inputting the string ^Z to indicate intention of
**  terminating the program.
*/
#define GET_BUF()                                                           \
{                                                                           \
    if ( FGETS( buf, BUF_LEN, stdin )  ==  NULL )                           \
    {                                                                       \
        printf( "Terminated by ^Z\n" );                                     \
        exit(0);                                                            \
    }                                                                       \
    if ( memcmp( buf, "^Z", 2 ) == 0 || memcmp( buf, "^z", 2 ) == 0  )      \
    {                                                                       \
        printf( "Terminated by ^Z\n" );                                     \
        exit(0);                                                            \
    }                                                                       \
}

#define PRINT( lst )                                                        \
{                                                                           \
    if( diagnostics )                                                       \
    {                                                                       \
        printf lst;                                                         \
    }                                                                       \
}

/*****************************************************************************
==  TYPE and STRUCTURE DEFINITIONS
*****************************************************************************/
typedef     unsigned char   bool;   /* Boolean value                        */
typedef     unsigned char   ubyte;  /* Exactly eight bits, unsigned         */
typedef     short           int16;  /* At least 16 bits, signed             */
typedef     unsigned short  uint16; /* At least 16 bits, unsigned           */
typedef     long            int32;  /* At least 32 bits, signed             */
typedef     unsigned long   uint32; /* At least 32 bits, unsigned           */

/*
**  Cassette file header.
*/

typedef struct
{
    ubyte       cas_record_id[4];       /* Cassette record type             */
    ubyte       cas_len_lo;             /* Record length low byte           */
    ubyte       cas_len_hi;             /* Record length high byte          */
    ubyte       cas_aux1;               /* Type dependant data              */
    ubyte       cas_aux2;               /* Type dependant data              */
    ubyte       cas_data[8192];         /* Data                             */
} cas_blk;

/*****************************************************************************
==  IMPORTED VARIABLES
*****************************************************************************/
/*****************************************************************************
==  LOCAL ( HIDDEN ) VARIABLES
*****************************************************************************/
static FILE *   cas_file;               /* Cassette image file              */
static FILE *   wav_file;               /* Wave file                        */

static cas_blk  cas_rec;                /* The cassette record buffer       */
static uint32   cas_len;                /* Length of cassette data          */

static uint32   baudrate;               /* Baudrate                         */
static bool     baudrate_fixed;         /* Fixed baudrate entered           */
static uint32   bitlen;                 /* Number of samples in one bit     */
static uint32   bit_stretch;            /* Number of samples for stretch    */
static uint32   bytelen;                /* Number of samples in one byte    */
static bool     diagnostics;            /* Print diagnostic data            */
static bool     header_written;         /* Did we write out a header yet    */
static bool     format_pure;            /* Format is pure sine waves        */
static bool     format_sine;            /* Format is sine waves             */
static bool     format_square;          /* Format is square waves           */
static bool     zero_transition;        /* Do a transition at zero level    */
static uint32   test_tape;              /* Generate test tape only          */
static uint32   mark_tone;              /* Frequency of mark tone           */
static uint32   space_tone;             /* Frequency of space tone          */
static uint32   leader;                 /* Fixed length of leader           */
static uint32   irg;                    /* Fixed length of Inter Record Gap */
static uint32   pos;                    /* Number of bytes in wav file      */
static uint32   pos_chunk_size;         /* Position in wav file for length  */
static uint32   pos_file_size;          /* Position in wav file for length  */
static ubyte    number[4];              /* Buffer for numbers               */
static ubyte *  mark;                   /* Buffer with mark tone generator  */
static ubyte *  space;                  /* Buffer with space tone generator */
static ubyte *  ptr;                    /* Pointer to tone generator        */
static uint32   table_len;              /* Length of generator table        */
static uint32   prev_bitvalue;          /* Last bit value written           */
static uint32   recno;                  /* Record number                    */

/*****************************************************************************
==  EXPORTED VARIABLES
*****************************************************************************/
/*****************************************************************************
==  IMPORTED FUNCTIONS
*****************************************************************************/
/*****************************************************************************
==  LOCAL ( HIDDEN ) FUNCTIONS
*****************************************************************************/
static void     cleanup( void );
static uint32   process_header( void );
static void     process_record( void );
static uint32   read_record( void );
static void     usage( char * cmd );
static void     write_wav( char * buffer, uint32 buflen );
static void     write_wav_bit( uint32 value, uint32 samples );
static void     write_wav_number( uint32 value, uint32 buflen );

/*****************************************************************************
==  EXPORTED FUNCTIONS
*****************************************************************************/
int                 main();                 /* Normal entry point to it all */

/*****************************************************************************
==  LOCAL ( HIDDEN ) FUNCTIONS
*****************************************************************************/

/*****************************************************************************
**  NAME:  cleanup()
**
**  PURPOSE:
**      Cleanup any mess that was created.
**
**  DESCRIPTION:
**      This function will attempt to close all open files and
**      free allocated memory.
**
**  INPUT:
**      - The file pointers and paths are used.
**
**  OUTPUT:
**      The function returns nothing.
**
*/

static void         cleanup( void )
{
    if( wav_file )
    {
        fclose( wav_file );
    }
    if( cas_file )
    {
        fclose( cas_file );
    }

    if( mark )
        free( (void *)mark );

    if( space )
        free( (void *)space );
    return;
}

/*****************************************************************************
**  NAME:  process_header()
**
**  PURPOSE:
**      Process the data from the header of the .cas file and store
**      relevant information.  This will verify that the file truly is
**      a cassette image file.
**
**  DESCRIPTION:
**      This function will read the relevant data about the .cas file
**      and verify it.
**
**  INPUT:
**      Nothing.
**      Data is read from the cassette image file.
**
**  OUTPUT:
**      Prints results.
**      Stores data related to the contents of the cassette image file.
**      Returns SUCCESS if header was processed successfully.
**      Returns FAILURE if some error occurred.
**
*/

static uint32       process_header( void )
{
    uint32          bytes;                  /* Number of bytes read         */

/*
**  Cassette image files usually look like this
**
**  char[4] = "FUJI", two chars record size lo/hi, two chars null
**  char[4] = "baud", two chars null, two chars baudrate lo/hi
**  char[4] = "data", two chars record size lo/hi, two chars PRWT lo/hi
**  See below for more details.
*/

/*
**  .cas files should begin with FUJI, followed by the size of the description.
*/
    bytes = fread( (char *)&cas_rec, (int)1, (int)4, cas_file );
    if( ( bytes < 4 ) || memcmp( cas_rec.cas_record_id, "FUJI", 4 ) )
    {
        fprintf(stderr, "\nThis is not a valid .cas file, it does not begin with \"FUJI\".\n");
        return( FAILURE );
    }

/*
**  Get the remaining four bytes of the record header.
*/
    bytes = fread( ((char *)&cas_rec)+4, (int)1, (int)4, cas_file );
    if( bytes < 4 )
    {
        fprintf(stderr, "\nThis is not a valid .cas file, description length missing.\n");
        return( FAILURE );
    }

/*
**  This looks like what we wanted, so we will call this success.
**  Position the file at the beginning again.
*/
    fseek( cas_file, 0L, SEEK_SET );    /* Go back to beginning of file */
    return( SUCCESS );
}

/*****************************************************************************
**  NAME:  process_record()
**
**  PURPOSE:
**      There is data in the cassette record buffer.  This must now be
**      converted to the selected format.
**
**  DESCRIPTION:
**      This function will take data from the record buffer and convert it
**      to FSK bits.  The bits are coded as mark and space tones.
**      These tones are output as wave data.
**
**  INPUT:
**      Nothing.
**      Data is taken from the cassette record buffer.
**
**  OUTPUT:
**      Data is output to the file.
**      The function returns nothing.
**
*/

static void         process_record( void )
{
    uint32          bit;                    /* Number of bits processed     */
    uint32          bitvalue;               /* Current bit level counting   */
    ubyte           byte;                   /* Byte decoded from fsk data   */
    uint32          bytes;                  /* Number of bytes read         */
    uint32          change;                 /* Bit change index             */
    uint32          changes;                /* Bit changes index            */
    uint32          msecs;                  /* Number of milli-seconds      */
    uint32          prwt;                   /* Length of the PRWT           */
    uint32          bits[10];               /* Number of bits               */
    uint32          remainder;              /* Left over samples            */
    uint32          samples[10];            /* Number of samples per bit    */
    uint32          total;                  /* Total sample count           */
    uint32          total_bits;             /* Total number of bits         */

/*
**  Currently, we only have three types of record:
**  FUJI  A header/description record.
**  baud  A record telling us the baudrate.
**  data  A record with cassette data.
*/

/*
**  If we find a header, and it is the first one, write the header
**  for the wave file.
*/
    if( memcmp( cas_rec.cas_record_id, "FUJI", 4 ) == 0 )
    {
        PRINT( ("\nDescription \"%.*s\".\n", (int)cas_len, cas_rec.cas_data) );
        if( !header_written )
        {
            write_wav( (char *)"RIFF", (uint32)4L );
            pos_file_size = pos;
            write_wav_number( (uint32)0L, (uint32)4L );
            header_written = TRUE;
        }
        write_wav( (char *)"WAVE", (uint32)4L );

        write_wav( (char *)"fmt ", (uint32)4L );
        write_wav_number( (uint32)16L, (uint32)4L ); /* Header size */

        write_wav_number( (uint32)1L, (uint32)2L );  /* fmt tag 1 */
        write_wav_number( (uint32)1L, (uint32)2L );  /* channels 1 */
        write_wav_number( (uint32)44100L, (uint32)4L );  /* sample rate 44100 */
        write_wav_number( (uint32)44100L, (uint32)4L );  /* Bytes per second */
        write_wav_number( (uint32)1L, (uint32)2L );  /* Buffer alignment */

        write_wav_number( (uint32)8L, (uint32)2L );  /* bits per sample */

        write_wav( (char *)"data", (uint32)4L );
        pos_chunk_size = pos;
        write_wav_number( (uint32)0L, (uint32)4L );

        return;
    }

/*
**  If this is a baudrate record, change the baud rate, unless
**  the user selected a fixed baudrate.
*/
    if( memcmp( cas_rec.cas_record_id, "baud", 4 ) == 0 )
    {
        if( !baudrate_fixed )
        {
            baudrate = (((uint32)cas_rec.cas_aux2) << 8 ) + cas_rec.cas_aux1;
            bytelen = ( 44100L * 10 ) / baudrate;
            bitlen = 44100L / baudrate;
        }
        PRINT( ("\nBaudrate set to %lu.\n", baudrate) );
        return;
    }

/*
**  If this is a plain old data record, encode the bits.
*/
    if( memcmp( cas_rec.cas_record_id, "data", 4 ) == 0 )
    {
        prwt = (((uint32)cas_rec.cas_aux2) << 8 ) + cas_rec.cas_aux1;

/*
**  If there is a fixed leader, and we did not write a leader yet,
**  set the length of the leader.
**  If this is a normal irg or other gap, any gap less than 3 seconds
**  is considered an irg and if specified, the fixed irg is used.
*/
        if( leader )
        {
            prwt = leader;
            leader = 0;
        }
        else
        {
            if( irg )
            {
                if( prwt < 3000 )
                    prwt = irg;
            }
        }

/*
**  Each record starts out with the PRWT.
**  Then we write out all the bytes in the cassette record.
**  The PRWT is measured in milli-seconds.  At a sample rate of
**  44,100, this means the number of samples is 44.1 times the prwt value.
*/
        write_wav_bit( FSK_PRWT, prwt * 44 + prwt / 10 );
        recno++;
        PRINT( ("\nRecord %lu at offset %lu PRWT = %lu with %lu data bytes.",
                 recno, pos - pos_chunk_size - 4, prwt, cas_len ) );

/*
**  Now process the data record byte by byte.
*/
        for( bytes = 0; bytes < cas_len; bytes++ )
        {

/*
**  Since this is FSK, we accumulate the sample counts of the bits that
**  are the same value.  This way, we can get a more precise baudrate.
**  Encode and sum up like bits.  Begin with a startbit, then the databits,
**  and finally the stopbit.
**  We need ten bits to encode one byte (startbit and stopbit included)
**  but since we are using integers here, we will divide the bytelength
**  by ten after summing up the bits, for improved accuracy.
*/
            bitvalue = FSK_STARTBIT;
            samples[0] = bytelen;
            bits[0] = 1;
            changes = 0;
            byte = cas_rec.cas_data[bytes];

/*
**  Convert each byte to a string of 8 bits.
**  Least significant bit is encoded first.
**  If the bitvalue changes, start the new count,
**  otherwise add the length of a bit to the sum.
*/
            for( bit = 0; bit < 8; bit++ )
            {
                if( ( byte & 0x01 ) != bitvalue )
                {
                    bitvalue = byte & 0x01;
                    changes++;
                    samples[changes] = bytelen;
                    bits[changes] = 1;
                }
                else
                {
                    samples[changes] += bytelen;
                    bits[changes]++;
                }
                byte = byte >> 1;
            }

/*
**  The last bit is the stop bit.  Add it to the length of the last bit(s)
**  if they were mark, otherwise, make a final new count.
*/
            if( bitvalue == FSK_STOPBIT )
            {
                samples[changes] += bytelen;
                bits[changes]++;
            }
            else
            {
                changes++;
                samples[changes] = bytelen;
                bits[changes] = 1;
            }

/*
**  Now that we have the bits encoded, we must still divide the bytelength
**  by ten, thus we must divide our sums by ten.  We must make sure that
**  the baudrate is correct, so the total sum of the samples for all bits
**  must be equal to the bytelength.  Distribute the remaining samples,
**  caused by inaccuracy in the division, by checking the sum at every change.
**  Keep a running total, and compute where we should be at, in order
**  to compensate for truncation that occurs during the division.
*/
            total = 0;
            total_bits = 0;
            for( change = 0; change <= changes; change++ )
            {
                samples[change] /= 10;
                total += samples[change];
                total_bits += bits[change];
                remainder = (( total_bits * bytelen ) / 10 ) - total;
                samples[change] += remainder;
                total += remainder;
            }

/*
**  Now that we have the bits encoded, stretch the space bits.
**  The table starts with the startbit, which is a space.  Steal the
**  samples from the following mark bit(s).
**  Even table entries are space, odd entries are mark, since they alternate.
*/
            for( change = 0; change < changes; change++ )
            {
                samples[change++] += bit_stretch;
                samples[change] -= bit_stretch;
            }

/*
**  Now output this stuff.
**  There are at least two changes, since we have a start bit and
**  a stop bit.  The bits may alternate several times, but we are
**  certain that the last one will be a stop bit, which is a mark.
**  Thus, process bits in pairs, alternating between space and mark.
*/
            for( change = 0; change < changes; change++ )
            {
                write_wav_bit( FSK_SPACE, samples[change++] );
                write_wav_bit( FSK_MARK, samples[change] );
            }
        }
        return;
    }

    PRINT( ("\nIn process_record() unknown record type %.4s %lu bytes data\n", &cas_rec, cas_len) );
    return;
}

/*****************************************************************************
**  NAME:  read_record()
**
**  PURPOSE:
**      Read a record into the cassette buffer.
**
**  DESCRIPTION:
**      This function will read data from the cas file and store it in the
**      cassette buffer.  One complete record is read into the buffer.
**
**  INPUT:
**      Nothing.
**      Data is read from the cas file.
**
**  OUTPUT:
**      Data is stored in the buffer.
**      Buffer status is updated.
**      Returns SUCCESS if a record was read successfully.
**      Returns FAILURE if some error occurred.
**
*/

static uint32       read_record( void )
{
    uint32          bytes;                  /* Number of bytes read         */

/*
**  First read the header bytes, because we need to know the length of the
**  record.  If we cannot read any bytes, this must be the end of file.
*/
    bytes = fread( (char *)&cas_rec, (int)1, (int)8, cas_file );
    if( bytes == 0 )
        return( FAILURE );

/*
**  Found header bytes, compute the record length from the header info.
**  Then read the data portion of the record, if any.
*/
    if( bytes != 8 )
    {
        fprintf(stderr, "\nThis is not a valid .cas file, record header damaged.\n");
        return( FAILURE );
    }
    cas_len = (((uint32)cas_rec.cas_len_hi) << 8 ) + cas_rec.cas_len_lo;
    if( cas_len )
    {
        bytes = fread( (char *)cas_rec.cas_data, (int)1, (int)cas_len, cas_file );
        if( bytes != cas_len )
        {
            fprintf(stderr, "\nThis is not a valid .cas file, record damaged.\n");
            return( FAILURE );
        }
    }
    return( SUCCESS );
}

/*****************************************************************************
**  NAME:  usage()
**
**  PURPOSE:
**      Display the command line format for the program.
**
**  DESCRIPTION:
**      This function will explain the usage of the program to the user.
**      The program name is taken from the first command line argument.
**
**  INPUT:
**      - The address of the command line, containing the program name.
**
**  OUTPUT:
**      The usage is displayed on the terminal.
**      The function returns nothing.
**
*/

static void         usage( cmd )
char * cmd;                         /* Program name                     */
{
    char * whoami;      /* For searching program name in command line   */
    char * name;        /* Pointer to actual program name in command    */
    int    len;         /* Length of program name                       */
    int    found_dot;   /* Nonzero if we found a dot in the name        */

/*
**  Get program name and print usage message.
**  The complete pathname including extension is part of the first
**  argument as passed by the operating system.
*/
    for( whoami = cmd, len = 0, found_dot = 0; *whoami; whoami++ )
    {
        if( *whoami == '.' )
        {
            found_dot = 1;
            continue;
        }
        if( *whoami == '\\' )   /* if this was part of the path, */
        {
            name = whoami + 1;  /* record position */
            len = 0;            /* then restart counting length */
            found_dot = 0;
            continue;
        }
        if( *whoami == ' ' )    /* end of name found            */
            break;
        if( found_dot )         /* skip .exe or .com stuff      */
            continue;
        len++;                  /* Increment program name length */
    }

/*
**  Let me explain...
*/
    fprintf(stderr, "\nUsage: %.*s [cassette file] [/d] [/w=x] [/t=nnnn] [/m=nnnn] [/s=nnnn]\n", len, name );
    fprintf(stderr, "                               [/b=nnnn] [/l=nnnn] [/i=nnnn]\n");
    fprintf(stderr, "to convert a .cas cassette image file to a .wav file.\n\n");
    fprintf(stderr, "cassette file an Atari classic tape image file.\n");
    fprintf(stderr, "/d            to print diagnostic information.\n");
    fprintf(stderr, "/w=x          to select the waveform of the tone used,\n");
    fprintf(stderr, "              where x is s for sine waves, b for block waves, p for pure tones.\n");
    fprintf(stderr, "/z            to select transition at zero level.\n");
    fprintf(stderr, "/t=nnnn       to generate a test tape only,\n");
    fprintf(stderr, "              where nnnn is the duration in milli-seconds.\n");
    fprintf(stderr, "/m=nnnn       frequency of mark tone in Hertz,\n");
    fprintf(stderr, "              where nnnn is a number around 5327.\n");
    fprintf(stderr, "/s=nnnn       frequency of space tone in Hertz,\n");
    fprintf(stderr, "              where nnnn is a number around 3995.\n");
    fprintf(stderr, "/b=nnnn       fixed baudrate to use,\n");
    fprintf(stderr, "              where nnnn is a number around 600, from 425 to 875.\n");
    fprintf(stderr, "/l=nnnn       fixed length of leader in milli-seconds,\n");
    fprintf(stderr, "              where nnnn is a number around 20000.\n");
    fprintf(stderr, "/i=nnnn       fixed length of Inter Record Gap in milli-seconds,\n");
    fprintf(stderr, "              where nnnn is a number around 250.\n");
    fprintf(stderr, "Refer to the documentation for more information.\n");

    return;
}

/*****************************************************************************
**  NAME:  write_test_tape()
**
**  PURPOSE:
**      Write data for a test tape to the wav file.
**
**  DESCRIPTION:
**      This function will write test data to the wav file.
**      A test tape consists of only mark and space bits in some test pattern.
**      Use an oscilloscope to view the output of the cassette unit.
**
**  INPUT:
**      - The file pointers and paths are used.
**
**  OUTPUT:
**      The data is written to the wav file.
**      The function returns nothing.
**
*/

static void         write_test_tape( void )
{
    uint32          sample_count;           /* Count of samples test tape   */
    uint32          samples;                /* Number of samples test tape  */

/*
**  We use a fixed filename for test tapes.
*/
    fprintf(stderr, "\nProcessing test tape, please wait!\n");
    wav_file = fopen( (char *)"testtape.wav", "wb" );
    if( wav_file == NULL )
    {
        fprintf(stderr, "\nCannot open testtape.wav file!\n");
        cleanup();
        exit( 255 );
    }

    if( !header_written )
    {
        write_wav( (char *)"RIFF", (uint32)4L );
        pos_file_size = pos;
        write_wav_number( (uint32)0L, (uint32)4L );
        header_written = TRUE;
    }
    write_wav( (char *)"WAVE", (uint32)4L );

    write_wav( (char *)"fmt ", (uint32)4L );
    write_wav_number( (uint32)16L, (uint32)4L ); /* Header size */

    write_wav_number( (uint32)1L, (uint32)2L );  /* fmt tag 1 */
    write_wav_number( (uint32)1L, (uint32)2L );  /* channels 1 */
    write_wav_number( (uint32)44100L, (uint32)4L );  /* sample rate 44100 */
    write_wav_number( (uint32)44100L, (uint32)4L );  /* Bytes per second */
    write_wav_number( (uint32)1L, (uint32)2L );  /* Buffer alignment */

    write_wav_number( (uint32)8L, (uint32)2L );  /* bits per sample */

    write_wav( (char *)"data", (uint32)4L );
    pos_chunk_size = pos;
    write_wav_number( (uint32)0L, (uint32)4L );

/*
**  Compute the number of samples to generate.
**  Multiply the number of milli-seconds by the sample rate.
**  The sample rate is per second, so divide by 1000.
*/
    samples = ( test_tape * 441 ) / 10;
    sample_count = 0;

/*
**  Generate the test pattern.
*/
    while( sample_count < samples )
    {

/*
**  Alternate mark and space bits.
*/
#if 0
        write_wav_bit( FSK_MARK, bitlen );
        sample_count += bitlen;
        write_wav_bit( FSK_SPACE, bitlen );
        sample_count += bitlen;
#endif

/*
**  A small piece of silence.
*/
#if 0
        for( byte = 0; byte < 881; byte++ )
        {
            write_wav( "\200", 1L );
            sample_count++;
        }
#endif

/*
**  Large piece of mark, and alternating space and mark,
**  with elongated space bits.
*/
#if 0
        write_wav_bit( FSK_MARK, bitlen * 3 );
        sample_count += bitlen * 3;
        ptr = mark;
        write_wav_bit( FSK_MARK, bitlen * 3 );
        sample_count += bitlen * 3;
        ptr = mark;
        write_wav_bit( FSK_SPACE, bitlen );
        sample_count += bitlen;
        ptr = mark;
        write_wav_bit( FSK_MARK, bitlen - 8);
        sample_count += bitlen;
        ptr = mark;
        write_wav_bit( FSK_SPACE, bitlen + 8);
        sample_count += bitlen;
        ptr = mark;
        write_wav_bit( FSK_MARK, bitlen - 8);
        sample_count += bitlen;
        ptr = mark;
        write_wav_bit( FSK_SPACE, bitlen + 8);
        sample_count += bitlen;
        ptr = mark;
        write_wav_bit( FSK_MARK, bitlen - 8);
        sample_count += bitlen;
#endif

/*
**  Large piece of mark, and alternating space and mark,
**  with abrupt change over.
*/
#if 0
        ptr = mark;
        write_wav( (char *)&(mark[0]), 414 );
        sample_count += 414;
        write_wav( (char *)&(space[414]), 73 );
        sample_count += 73;
        write_wav( (char *)&(mark[487]), 73 );
        sample_count += 73;
        write_wav( (char *)&(space[560]), 73 );
        sample_count += 73;
        write_wav( (char *)&(mark[633]), 73 );
        sample_count += 73;
        write_wav( (char *)&(space[706]), 73 );
        sample_count += 73;
        write_wav( (char *)&(mark[779]), 73 );
        sample_count += 73;
#endif

/*
**  Large piece of mark, and alternating space and mark,
**  with normal change over.
*/
#if 1
        ptr = mark;
        table_len = MARK_TABLE_LEN;
        write_wav_bit( FSK_MARK, bitlen * 3 );
        sample_count += bitlen * 3;
        write_wav_bit( FSK_MARK, bitlen * 3 );
        sample_count += bitlen * 3;
        write_wav_bit( FSK_SPACE, bitlen );
        sample_count += bitlen;
        write_wav_bit( FSK_MARK, bitlen );
        sample_count += bitlen;
        write_wav_bit( FSK_SPACE, bitlen );
        sample_count += bitlen;
        write_wav_bit( FSK_MARK, bitlen );
        sample_count += bitlen;
        write_wav_bit( FSK_SPACE, bitlen );
        sample_count += bitlen;
        write_wav_bit( FSK_MARK, bitlen );
        sample_count += bitlen;
#endif

/*
**  Large piece of mark, and alternating space and mark,
**  with normal change over and elongated space bits.
*/
#if 0
        ptr = mark;
        table_len = MARK_TABLE_LEN;
        write_wav_bit( FSK_MARK, bitlen * 3 );
        sample_count += bitlen * 3;
        write_wav_bit( FSK_MARK, bitlen * 3 );
        sample_count += bitlen * 3;
        write_wav_bit( FSK_SPACE, bitlen + 9 );
        sample_count += bitlen + 9;
        write_wav_bit( FSK_MARK, bitlen - 9);
        sample_count += bitlen - 9;
        write_wav_bit( FSK_SPACE, bitlen + 9);
        sample_count += bitlen + 9;
        write_wav_bit( FSK_MARK, bitlen - 9);
        sample_count += bitlen - 9;
        write_wav_bit( FSK_SPACE, bitlen + 9);
        sample_count += bitlen + 9;
        write_wav_bit( FSK_MARK, bitlen );
        sample_count += bitlen;
#endif
    }

    return;
}

/*****************************************************************************
**  NAME:  write_wav()
**
**  PURPOSE:
**      Write data to the wav file.
**
**  DESCRIPTION:
**      This function will write the specified buffer to the wav file.
**
**  INPUT:
**      - The address of the data to be written.
**      - The amount of data to be written.
**
**  OUTPUT:
**      The data is written to the wav file.
**      The function returns nothing.
**
*/

static void         write_wav( buffer, buflen )
char * buffer;                      /* Address of buffer to be written  */
uint32 buflen;                      /* Number of bytes to be written    */
{
    uint32 bytes;       /* Number of bytes actually written             */

    bytes = fwrite( buffer, (int)1, (int)buflen, wav_file );
    if( bytes != buflen )
    {
        fprintf(stderr, "\nWrite error on wav file.\n");
        cleanup();
        exit( 255 );
    }
    pos += bytes;
    return;
}

/*****************************************************************************
**  NAME:  write_wav_bit()
**
**  PURPOSE:
**      Write a bit or a prwt to the wav file.
**
**  DESCRIPTION:
**      This function will write the specified number of samples to the wav
**      file, representing the selected bit value.
**      The way we make the transition from mark to space or from space to
**      mark depends on the selected format of the waves.
**
**  INPUT:
**      - The bit value to be represented.
**      - The amount of data to be written.
**
**  OUTPUT:
**      The data is written to the wav file.
**      The function returns nothing.
**
*/

static void         write_wav_bit( bitvalue, samples )
uint32 bitvalue;                    /* The bit value to be represented  */
uint32 samples;                     /* Number of bytes to be written    */
{
    uint32 bytes;                   /* Number of bytes actually written */
    bool   falling;                 /* Level of signal is falling       */
    ubyte  last_value;              /* Last byte value output           */

/*
**  Here is where we have to do special things in order to make the
**  selected type of transition from mark to space or from space to mark,
**  if we have a transition at all.  If there is no transition, things
**  are really simple.  If there is a transition, the transition from
**  mark to space is the most important.  Unless we are writing pure
**  tones, the mark tone is always written from the end of the mark table,
**  and the space tone is written from the start of the space table.
**  Unfortunately, sometimes we write a very long mark tone, which is longer
**  than the table.  In this case we will be wrapping around to the start of
**  the table one or more times, so compute where to start at, such that we
**  we arrive at the end of the mark table.
**  With pure tones, we always make a smooth transition.
*/

/*
**  No bytes written yet.
*/
    bytes = 0;

/*
**  If only the result counts, we do not care whether or not the wave format
**  looks nice, we just want it to load reliable.  For mark tones, write the
**  tone from the end of the table.  For space tones, write it from the start
**  of the table.  This makes the mark to space transition occur at the zero
**  level.
*/
    if( zero_transition )
    {
        if( bitvalue == FSK_MARK )
        {
            if( samples < MARK_TABLE_LEN )
            {
                ptr = &(mark[MARK_TABLE_LEN - samples]);
                table_len = samples;
            }
            else
            {
                ptr = &(mark[MARK_TABLE_LEN - (samples % MARK_TABLE_LEN)]);
                table_len = samples % MARK_TABLE_LEN;
            }
        }
        else
        {
            ptr = space;
            table_len = SPACE_TABLE_LEN;
        }
    } /* end if zero transition */
    else

/*
**  If we are writing pure tones, we must orderly end the current tone
**  before changing over to the next.
**  Complete the half period in progress before changing the tone.
*/
    if( format_pure )
    {

/*
**  When we pass the zero level, change the tone.
**  Try to make a clean transition from mark to space.
**  Finish up the last half period of the previous bit.
**  If the bit is the same, continue with the current tone where we
**  left off.
*/
        if( bitvalue != prev_bitvalue )
        {

/*
**  If we are at the start of our generator buffer, no need to do
**  complicated things, we can simply change to the start of the other
**  generator tone buffer.
**  Besides, we would not have a previous value in this case to determine
**  rising or falling, although we know it is rising.
*/
            if( ( prev_bitvalue == FSK_MARK ) && ( ptr == mark ) )
            {
                ptr = space;
                table_len = SPACE_TABLE_LEN;
            }
            else
            if( ( prev_bitvalue == FSK_SPACE ) && ( ptr == space ) )
            {
                ptr = mark;
                table_len = MARK_TABLE_LEN;
            }
            else

/*
**  Otherwise, figure out whether the signal was rising or falling.
**  If we just crossed the zero level, we can see whether we are rising
**  or falling.  If the current value is below the zero level, we are now
**  falling.  If it is above the zero level, we are now rising.
*/
            {
                if( ( *(ptr - 1) >= ZERO_LEVEL ) && ( *ptr <= ZERO_LEVEL ) )
                {
                    falling = TRUE;
                }
                else
                if( ( *(ptr - 1) <= ZERO_LEVEL ) && ( *ptr >= ZERO_LEVEL ) )
                {
                    falling = FALSE;
                }
                else

/*
**  Since we did not cross the zero level just now, look at the last value.
**  If it is above the zero level, continue to write samples until we pass
**  the zero level falling down, otherwise continue to write samples until
**  we pass the zero level going up.
*/
                if( *(ptr - 1) > ZERO_LEVEL )
                {
                    falling = TRUE;
                    while( *ptr > ZERO_LEVEL )
                    {
                        write_wav( (char *)ptr, 1 );
                        bytes++;
                        ptr++;
                        if( --table_len == 0 )
                            break;
                    }
                }
                else
                {
                    falling = FALSE;
                    while( *ptr < ZERO_LEVEL )
                    {
                        write_wav( (char *)ptr, 1 );
                        bytes++;
                        ptr++;
                        if( --table_len == 0 )
                            break;
                    }
                }

/*
**  At this point, we finished the last half period of the previous
**  tone.  Now switch tone.  Make sure we are going in the same direction
**  as the previous tone, either falling or rising.
**  This means that if we need to be falling, we must move ahead until
**  we are.  The tables start out rising.
*/
                if( bitvalue == FSK_MARK )
                {
                    ptr = mark;
                    table_len = MARK_TABLE_LEN;
                }
                else
                {
                    ptr = space;
                    table_len = SPACE_TABLE_LEN;
                }

/*
**  If falling, search for the zero crossing in the other table.
*/
                if( falling )
                {
                    ptr++;
                    table_len--;
                    for( ; table_len; ptr++, table_len-- )
                    {
                        if( *ptr <= ZERO_LEVEL )
                            break;
                    }
                }
            } /* end else if at start of buffer */
        } /* end if bit value changed */
    } /* end if pure tones */

    else

/*
**  If we are writing normal sine waves, change from one sine wave to
**  the other.  Do so by finding the corresponding value in the other table,
**  and continue writing data from the other table.
*/
    if( format_sine )
    {
        if( bitvalue != prev_bitvalue )
        {

/*
**  Check for start of buffer.
*/
            if( ( prev_bitvalue == FSK_MARK ) && ( ptr == mark ) )
            {
                ptr = space;
                table_len = SPACE_TABLE_LEN;
            }
            else
            if( ( prev_bitvalue == FSK_SPACE ) && ( ptr == space ) )
            {
                ptr = mark;
                table_len = MARK_TABLE_LEN;
            }
            else

/*
**  Compare the current value to the last value.  If it is smaller, we
**  are falling.  If it is larger, we are rising.  If it is the same, we
**  must be at the top or bottom of the sine wave, and we are then moving
**  away from that.
*/
            {
                last_value = *(ptr - 1);

#if 0

/*
**  If the space value was larger than any value in the mark table,
**  we will never find the correct value, so limit it to the top value.
*/
                if( last_value > 192 )
                {
                    falling = TRUE;
                    last_value = 192;
                }
                else
                if( last_value < 64 )
                {
                    falling = FALSE;
                    last_value = 64;
                }
                else

#endif

                if( last_value > *ptr )
                {
                    falling = TRUE;
                }
                else
                if( last_value < *ptr )
                {
                    falling = FALSE;
                }
                else

/*
**  Values are the same, moving away from top, determine which top.
*/
                {
                    if( *ptr > ZERO_LEVEL )
                        falling = TRUE;
                    else
                        falling = FALSE;
                }

/*
**  Now that we know whether we are rising or falling, find the value we
**  wrote last, but this time in the buffer for the other frequency, and
**  find it either falling or rising, whatever it was we were doing.
*/
                if( bitvalue == FSK_MARK )
                {
                    ptr = mark + 1;
                    table_len = MARK_TABLE_LEN - 1;
                }
                else
                {
                    ptr = space + 1;
                    table_len = SPACE_TABLE_LEN - 1;
                }

/*
**  Search for the value in the other table.
*/
                for( ; table_len; ptr++, table_len-- )
                {
                    if( *ptr == last_value )
                    {
                        if( falling )
                        {
                            ptr++;
                            table_len--;
                            if( table_len == 0 )
                                break;
                            if( *ptr < last_value )
                                break;
                        }
                        else
                        {
                            ptr++;
                            table_len--;
                            if( table_len == 0 )
                                break;
                            if( *ptr > last_value )
                                break;
                        }
                    } /* end if matching value found */
                } /* end search for matching value and direction */
            } /* end else if at start of buffer */
        } /* end if bit value changed */
    } /* end if sine wave */

    else

/*
**  Nothing left but square waves.  We cannot really make them smooth.
**  We do make sure that at least the level changes to the opposite,
**  so that the length of the first period is correct.
*/
    if( format_square )
    {
        if( bitvalue == FSK_MARK )
        {
            if( *ptr > ZERO_LEVEL )
            {
                ptr = mark;
                table_len = MARK_TABLE_LEN;
                while( *ptr > ZERO_LEVEL )
                {
                    ptr++;
                    table_len--;
                }
            }
            else
            {
                ptr = mark;
                table_len = MARK_TABLE_LEN;
            }
        } /* end if mark */
        else
        {
            if( *ptr > ZERO_LEVEL )
            {
                ptr = space;
                table_len = SPACE_TABLE_LEN;
                while( *ptr > ZERO_LEVEL )
                {
                    ptr++;
                    table_len--;
                }
            }
            else
            {
                ptr = space;
                table_len = SPACE_TABLE_LEN;
            }
        } /* end else if mark */
    } /* end if square waves */

/*
**  We are now pointing to the correct spot in the buffer.
**  Write the requested number of samples.  Deduct the number of samples
**  we used to complete the previous tone.
*/
    if( samples > bytes )
    {
        while( samples - bytes > table_len )
        {
            write_wav( (char *)ptr, table_len );
            bytes += table_len;

            if( bitvalue == FSK_MARK )
            {
                ptr = mark;
                table_len = MARK_TABLE_LEN;
            }
            else
            {
                ptr = space;
                table_len = SPACE_TABLE_LEN;
            }
        }
        write_wav( (char *)ptr, samples - bytes );
        ptr += ( samples - bytes );
        table_len -= ( samples - bytes );
    }

    prev_bitvalue = bitvalue;

    if( table_len == 0 )
    {
        if( bitvalue == FSK_MARK )
        {
            ptr = mark;
            table_len = MARK_TABLE_LEN;
        }
        else
        {
            ptr = space;
            table_len = SPACE_TABLE_LEN;
        }
    }
}

/*****************************************************************************
**  NAME:  write_wav_number()
**
**  PURPOSE:
**      Write a number to the wav file.
**
**  DESCRIPTION:
**      This function will write the specified number to the wav file.
**
**  INPUT:
**      - The number to be written.
**      - The amount of data to be written.
**
**  OUTPUT:
**      The data is written to the wav file.
**      The function returns nothing.
**
*/

static void         write_wav_number( value, buflen )
uint32 value;                       /* The number to be written         */
uint32 buflen;                      /* Number of bytes to be written    */
{
    number[0] = value;
    number[1] = value >> 8;
    number[2] = value >> 16;
    number[3] = value >> 24;
    write_wav( (char *)number, buflen );
}

/*****************************************************************************
==  EXPORTED FUNCTIONS
*****************************************************************************/

/*****************************************************************************
**  NAME:  MAIN()
**
**  PURPOSE:
**      An entry point for testing or running this utility.
**
**  DESCRIPTION:
**      Prompt for the file to open and then go process it.
**
**  INPUT:
**      argc and argv.
**
**  OUTPUT:
**      Returns an int as it should.
**
*/
int                 main( argc, argv )
int                 argc;               /* Command line argument count  */
char              * argv[];             /* Command line argument ptrs   */
{
    ubyte           answer;                 /* Response to yes/no question  */
    uint32          arg_ndx;                /* Argument number index        */
    uint32          arg_no;                 /* Argument number              */
    uint32          byte;                   /* Byte index                   */
    uint32          wrk_ndx;                /* Work index                   */
    bool            end_of_str;             /* Null terminator seen?        */
    ubyte           input_path[PATH_LEN];   /* Input cas file spec          */
    uint32          len_chunk;              /* Chunck length                */
    uint32          len_file;               /* File length                  */
    ubyte           wav_path[PATH_LEN];     /* Output wave file spec        */
    ubyte           buf[BUF_LEN];           /* Buffer string                */
    double          rad;                    /* Radians intermediate value   */
    double          rad_mark;               /* Radians intermediate value   */
    double          rad_space;              /* Radians intermediate value   */
    uint32          stat;                   /* Status from function         */
    ubyte           proceed;                /* Proceed with conversion      */

/*
**  Allocate buffer space for the tone generators.
*/
    mark = (ubyte *)malloc((unsigned long)MARK_TABLE_LEN * sizeof( ubyte ) );
    space = (ubyte *)malloc((unsigned long)SPACE_TABLE_LEN * sizeof( ubyte ) );

    if( !mark || !space )
    {
        fprintf( stderr, "\nCannot allocate buffer, insufficient memory.\n" );
        cleanup();
        exit( 255 );
    }

/*
**  Process command line arguments.
**  We do not treat the options switch as an argument.  It may be placed
**  anywhere on the command line.  So we have to count the arguments ourselves
**  so that we know what argument we are processing.
*/
    arg_no = 0;
    diagnostics = FALSE;
    test_tape = 0;
    baudrate = 600;
    baudrate_fixed = FALSE;
    leader = 0;
    irg = 0;
    mark_tone = FSK_TONE_MARK;
    space_tone = FSK_TONE_SPACE;
    table_len = MARK_TABLE_LEN;
    recno = 0;
    format_pure = FALSE;
    format_sine = TRUE;
    format_square = FALSE;
    zero_transition = FALSE;

    for( arg_ndx = 1; arg_ndx < argc; arg_ndx++ )
    {

/*
**  If we encounter the options switch, process the options.
**  The options must start with a slash.
*/
        if( argv[arg_ndx][0] == '/' )
        {

            for( wrk_ndx = 1; argv[arg_ndx][wrk_ndx]; wrk_ndx++ )
            {

/*
**  If the user is confused, seeking help, she/he should read the * manual.
**  We can give them a hint though.
*/
                if( argv[arg_ndx][wrk_ndx] == '?' )
                {
                    usage( argv[0] );
                    exit( 0 );
                }

/*
**  The /d option selects the diagnostics output.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'D' )
                {
                    diagnostics = TRUE;
                    break;
                }

/*
**  The /z option selects the transition at the zero level.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'Z' )
                {
                    zero_transition = TRUE;
                    break;
                }

/*
**  The /w option selects the wave format.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'W' )
                {
                    while( argv[arg_ndx][++wrk_ndx] )
                    {
                        if( toupper( argv[arg_ndx][wrk_ndx] ) == 'S' )
                        {
                            format_pure = FALSE;
                            format_sine = TRUE;
                            format_square = FALSE;
                        }
                        if( toupper( argv[arg_ndx][wrk_ndx] ) == 'B' )
                        {
                            format_pure = FALSE;
                            format_sine = FALSE;
                            format_square = TRUE;
                        }
                        if( toupper( argv[arg_ndx][wrk_ndx] ) == 'P' )
                        {
                            format_pure = TRUE;
                            format_sine = TRUE;
                            format_square = FALSE;
                        }
                    }
                    break;
                }

/*
**  The /t option selects a test tape output.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'T' )
                {
                    test_tape = 0;
                    while( argv[arg_ndx][++wrk_ndx] )
                    {
                        if( ( argv[arg_ndx][wrk_ndx] >= '0' ) &&
                            ( argv[arg_ndx][wrk_ndx] <= '9' ) )
                        {
                            test_tape *= 10;
                            test_tape += argv[arg_ndx][wrk_ndx] - '0';
                        }
                    }
                    break;
                }

/*
**  The /m option selects the mark tone frequency.
**  The format of this switch is /m=nnnn where nnnn is a numeric value
**  that should be around 5327 Hertz.  We accept any number the user enters
**  though.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'M' )
                {
                    mark_tone = 0;
                    while( argv[arg_ndx][++wrk_ndx] )
                    {
                        if( ( argv[arg_ndx][wrk_ndx] >= '0' ) &&
                            ( argv[arg_ndx][wrk_ndx] <= '9' ) )
                        {
                            mark_tone *= 10;
                            mark_tone += argv[arg_ndx][wrk_ndx] - '0';
                        }
                    }
                    break;
                }

/*
**  The /s option selects the space tone frequency.
**  The format of this switch is /s=nnnn where nnnn is a numeric value
**  that should be around 3995 Hertz.  We accept any number the user enters
**  though.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'S' )
                {
                    space_tone = 0;
                    while( argv[arg_ndx][++wrk_ndx] )
                    {
                        if( ( argv[arg_ndx][wrk_ndx] >= '0' ) &&
                            ( argv[arg_ndx][wrk_ndx] <= '9' ) )
                        {
                            space_tone *= 10;
                            space_tone += argv[arg_ndx][wrk_ndx] - '0';
                        }
                    }
                    break;
                }

/*
**  The /b option selects the baudrate.
**  The format of this switch is /b=nnnn where nnnn is a numeric value
**  that should be around 600 baud.  We accept any number the user enters
**  though.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'B' )
                {
                    baudrate = 0;
                    baudrate_fixed = TRUE;
                    while( argv[arg_ndx][++wrk_ndx] )
                    {
                        if( ( argv[arg_ndx][wrk_ndx] >= '0' ) &&
                            ( argv[arg_ndx][wrk_ndx] <= '9' ) )
                        {
                            baudrate *= 10;
                            baudrate += argv[arg_ndx][wrk_ndx] - '0';
                        }
                    }
                    break;
                }

/*
**  The /l option selects the length of the leader.
**  The format of this switch is /l=nnnn where nnnn is a numeric value
**  that should be around 20000 milli-seconds.  We accept any number the user
**  enters though.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'L' )
                {
                    leader = 0;
                    while( argv[arg_ndx][++wrk_ndx] )
                    {
                        if( ( argv[arg_ndx][wrk_ndx] >= '0' ) &&
                            ( argv[arg_ndx][wrk_ndx] <= '9' ) )
                        {
                            leader *= 10;
                            leader += argv[arg_ndx][wrk_ndx] - '0';
                        }
                    }
                    break;
                }

/*
**  The /i option selects the length of inter record gaps.
**  The format of this switch is /i=nnnn where nnnn is a numeric value
**  that should be around 250 milli-seconds.  We accept any number the user
**  enters though.
*/
                if( toupper( argv[arg_ndx][wrk_ndx] ) == 'I' )
                {
                    irg = 0;
                    while( argv[arg_ndx][++wrk_ndx] )
                    {
                        if( ( argv[arg_ndx][wrk_ndx] >= '0' ) &&
                            ( argv[arg_ndx][wrk_ndx] <= '9' ) )
                        {
                            irg *= 10;
                            irg += argv[arg_ndx][wrk_ndx] - '0';
                        }
                    }
                    break;
                }

/*
**  Ignore other options.
*/
                fprintf(stderr,"\nOption %c invalid.\n", argv[arg_ndx][wrk_ndx] );
                break;
            } /* end for all characters after options switch */

/*
**  No further processing for the options switches.
*/
            continue;
        } /* end if options switch */
        arg_no++;

/*
**  First argument is the file spec.
*/
        if( arg_no == 1 )
        {
            for ( wrk_ndx = 0, end_of_str = FALSE;
                wrk_ndx < PATH_LEN; wrk_ndx++ )
            {
                if ( argv[arg_ndx][wrk_ndx] == '\0' ) /* End of argument string?            */
                    end_of_str = TRUE;
                if ( end_of_str )
                    input_path[wrk_ndx] = '\0';
                else
                    input_path[wrk_ndx] = toupper( argv[arg_ndx][wrk_ndx] );
            }

            cas_file = fopen( (char *)input_path, "rb" );
            if( cas_file == NULL )
            {
                fprintf(stderr, "Cannot open cassette image file %s\n", input_path);
                exit( 255 );
            }
        }
    } /* end for all command line arguments */

/*
**  If there is no filename on the command line, ask for it.
*/
    if( arg_no == 0 && !test_tape )
    {

/*
**  Open hailing frequencies.
**  No command line arguments, so ask what it is we have to do.
*/
        printf( "\n\nClassic Atari cassette tape recovery version April 26, 1999\n" );
        printf( "\n\nCopyright 1998, 1999 by Ernest R. Schreurs\n" );
        printf( "\n\nAll rights reserved\n" );


        while( TRUE )                       /* until terminated by control Z*/
        {
            printf( "\nEnter ^Z or hit Control Z to terminate\n" );

            do                              /* until .cas file entered    */
            {
                printf("\nEnter .cas file to be converted : ");
                GET_BUF();

                for ( wrk_ndx = 0, end_of_str = FALSE;
                    wrk_ndx < PATH_LEN; wrk_ndx++ )
                {
                    if ( wrk_ndx < BUF_LEN )
                    {
                        if ( buf[wrk_ndx] == '\n' ) /* End of inputted string?      */
                            end_of_str = TRUE;
                        if ( buf[wrk_ndx] == '\0' ) /* Overkill, End marked by \n   */
                            end_of_str = TRUE;
                        if ( end_of_str )
                            input_path[wrk_ndx] = '\0';
                        else
                            input_path[wrk_ndx] = toupper( buf[wrk_ndx] );
                    }
                }
            } while ( input_path[0] == ' ' );

            do                              /* until answer is Y or N       */
            {
                printf("\nConvert file %s\n", input_path);
                printf("\nIs this correct [Y]es or N)o : ");
                GET_BUF();
                proceed = toupper( buf[0] );

/*
**  If blank, default is correct
*/
                if ( proceed == '\n' )
                    proceed = 'Y';

            } while ( proceed != 'Y' && proceed != 'N' );

            if ( proceed == 'N' )
                continue;

            cas_file = fopen( (char *)input_path, "rb" );
            if( cas_file == NULL )
            {
                fprintf(stderr, "Cannot open cassette image file\n");
                continue;
            }
            break;
        } /* end while need a valid filename */

        do                              /* until answer is Y or N       */
        {
            printf("\nPrint diagnostic data [Y]es or N)o : ");
            GET_BUF();
            answer = toupper( buf[0] );

/*
**  If blank, default is correct
*/
            if ( answer == '\n' )
                answer = 'Y';
        } while ( answer != 'Y' && answer != 'N' );

        diagnostics = ( answer == 'Y' ) ? TRUE : FALSE;

/*
**  Ask for the wave format.
*/
        format_pure = FALSE;
        format_sine = FALSE;
        format_square = FALSE;
        do                              /* until answer is S or B or P */
        {
            printf("\nDo you want s)ine waves, b)lock waves or p)ure waves? [s]: ");
            GET_BUF();
            answer = toupper( buf[0] );

/*
**  If blank, default is block waves
*/
            if ( answer == '\n' )
                answer = 'S';

        } while ( (answer != 'S') && (answer != 'B') && (answer != 'P') );

        if( answer == 'S' )
        {
            format_pure = FALSE;
            format_sine = TRUE;
            format_square = FALSE;
        }
        if( answer == 'B' )
        {
            format_pure = FALSE;
            format_sine = FALSE;
            format_square = TRUE;
        }
        if( answer == 'P' )
        {
            format_pure = TRUE;
            format_sine = TRUE;
            format_square = FALSE;
        }

        do                              /* until answer is Y or N       */
        {
            printf("\nUse space/mark transition at zero level Y)es or [N]o : ");
            GET_BUF();
            answer = toupper( buf[0] );

/*
**  If blank, default is no
*/
            if ( answer == '\n' )
                answer = 'N';

        } while ( answer != 'Y' && answer != 'N' );

        zero_transition = ( answer == 'Y' ) ? TRUE : FALSE;

/*
**  Ask for the mark tone frequency.
*/
        printf("\nEnter mark frequency [5327]: ");
        GET_BUF();
        mark_tone = 0;

        for( wrk_ndx = 0; wrk_ndx < BUF_LEN; wrk_ndx++ )
        {
            if ( buf[wrk_ndx] == '\n' ) /* End of inputted string?      */
                break;
            if ( buf[wrk_ndx] == '\0' ) /* Overkill, End marked by \n   */
                break;
            if( ( buf[wrk_ndx] >= '0' ) &&
                ( buf[wrk_ndx] <= '9' ) )
            {
                mark_tone *= 10;
                mark_tone += buf[wrk_ndx] - '0';
            }
        }
        if( mark_tone == 0 )
            mark_tone = FSK_TONE_MARK;

/*
**  Ask for the space tone frequency.
*/
        printf("\nEnter space frequency [3995]: ");
        GET_BUF();
        space_tone = 0;

        for( wrk_ndx = 0; wrk_ndx < BUF_LEN; wrk_ndx++ )
        {
            if ( buf[wrk_ndx] == '\n' ) /* End of inputted string?      */
                break;
            if ( buf[wrk_ndx] == '\0' ) /* Overkill, End marked by \n   */
                break;
            if( ( buf[wrk_ndx] >= '0' ) &&
                ( buf[wrk_ndx] <= '9' ) )
            {
                space_tone *= 10;
                space_tone += buf[wrk_ndx] - '0';
            }
        }
        if( space_tone == 0 )
            space_tone = FSK_TONE_SPACE;

/*
**  Ask for a fixed baudrate.
*/
        printf("\nEnter fixed baudrate if desired 425 - 875 : ");
        GET_BUF();
        baudrate = 0;

        for( wrk_ndx = 0; wrk_ndx < BUF_LEN; wrk_ndx++ )
        {
            if ( buf[wrk_ndx] == '\n' ) /* End of inputted string?      */
                break;
            if ( buf[wrk_ndx] == '\0' ) /* Overkill, End marked by \n   */
                break;
            if( ( buf[wrk_ndx] >= '0' ) &&
                ( buf[wrk_ndx] <= '9' ) )
            {
                baudrate *= 10;
                baudrate += buf[wrk_ndx] - '0';
            }
        }
        if( baudrate == 0 )
            baudrate = 600;
        else
            baudrate_fixed = TRUE;

/*
**  Ask for the fixed length of the leader.
*/
        printf("\nLength of leader if fixed length in milli-seconds : ");
        GET_BUF();
        leader = 0;

        for( wrk_ndx = 0; wrk_ndx < BUF_LEN; wrk_ndx++ )
        {
            if ( buf[wrk_ndx] == '\n' ) /* End of inputted string?      */
                break;
            if ( buf[wrk_ndx] == '\0' ) /* Overkill, End marked by \n   */
                break;
            if( ( buf[wrk_ndx] >= '0' ) &&
                ( buf[wrk_ndx] <= '9' ) )
            {
                leader *= 10;
                leader += buf[wrk_ndx] - '0';
            }
        }
 
/*
**  Ask for the fixed length of the irg.
*/
        printf("\nLength of Inter Record Gap if fixed length in milli-seconds : ");
        GET_BUF();
        irg = 0;

        for( wrk_ndx = 0; wrk_ndx < BUF_LEN; wrk_ndx++ )
        {
            if ( buf[wrk_ndx] == '\n' ) /* End of inputted string?      */
                break;
            if ( buf[wrk_ndx] == '\0' ) /* Overkill, End marked by \n   */
                break;
            if( ( buf[wrk_ndx] >= '0' ) &&
                ( buf[wrk_ndx] <= '9' ) )
            {
                irg *= 10;
                irg += buf[wrk_ndx] - '0';
            }
        }
 
    } /* end else if command line arguments */

/*
**  Compute the sine value for all sample positions.
**  The mark tone is 5327 Hertz.  One period is 2 PI radians,
**  One second contains 5327 periods.  We have 44,100 samples in one second.
**  To evenly divide these 5327 * 2 PI radians over 44,100 samples, we
**  have to divide by 44,100.  To get the radians value, multiply by the
**  sample position number.  Similar computations are done for the space tone.
**  Convert the sine value to a PCM value with proper audio volume by
**  multiplying it by 64.
**  The center of the PCM values is at 128, thus we add 128.
*/
//    rad = (double)mark_tone * 2.0 * PI / MARK_TABLE_LEN;
    rad = (double)mark_tone * 2.0 * M_PI / MARK_TABLE_LEN;
    for( byte = 0; byte < MARK_TABLE_LEN; byte++ )
    {
        //mark[byte] = sin( rad * byte ) * 64 + ZERO_LEVEL;
        mark[byte] = sin( rad * byte ) * 127 + ZERO_LEVEL;
		
        if( format_square )
        {
            if( mark[byte] >= ZERO_LEVEL )
                mark[byte] = 192;
            else
                mark[byte] = 64;
        }

    }
//    rad = (double)space_tone * 2.0 * PI / SPACE_TABLE_LEN;
    rad = (double)space_tone * 2.0 * M_PI / SPACE_TABLE_LEN;
    for( byte = 0; byte < SPACE_TABLE_LEN; byte++ )
    {
        //space[byte] = sin( rad * byte ) * 64 + ZERO_LEVEL;
        space[byte] = sin( rad * byte ) * 127 + ZERO_LEVEL;
		
        if( format_square )
        {
            if( space[byte] >= ZERO_LEVEL )
                space[byte] = 128;
            else
                space[byte] = 64;
        }

    }

/*
**  Initialize the pointer to the table.
*/
    bytelen = ( 44100L * 10 ) / baudrate;
    bitlen = 44100L / baudrate;

#if 0

    bit_stretch = 9;
    if( zero_transition && format_square )
        bit_stretch = 0;

#endif

    header_written = FALSE;
    prev_bitvalue = FSK_MARK;
    ptr = mark;
    table_len = MARK_TABLE_LEN;

/*
**  If we must write a test-tape, do so, and then quit.
*/
    if( test_tape )
    {
        write_test_tape();
    } /* end if test tape */
    else

/*
**  Generate a wave file based on the .cas file.
*/
    {

/*
**  Process the header of the .cas file.
*/
        stat = process_header();
        if( stat == FAILURE )
        {
            cleanup();
            exit( 255 );
        }

        PRINT( ("File : %s\n", input_path ) );

        memcpy( wav_path, input_path, PATH_LEN );

/*
**  Find the dot and replace by or add .wav extension.
*/
        wrk_ndx = STRLEN(wav_path);
        for( ;; )
        {
            if( wav_path[wrk_ndx] == '.' )
            {
                memcpy( &(wav_path[wrk_ndx]), ".wav", 5 );
                break;
            }
            if( ( wrk_ndx == 0 ) ||
                ( wav_path[wrk_ndx] == '/' ) ||
                ( wav_path[wrk_ndx] == '\\' ) )
            {
                memcpy( &(wav_path[STRLEN(wav_path)]), ".wav", 5 );
                break;
            }
            wrk_ndx--;
        }

        wav_file = fopen( (char *)wav_path, "wb" );
        if( wav_file == NULL )
        {
            fprintf(stderr, "\nCannot open .wav file!\n");
            cleanup();
            exit( 255 );
        }

        fprintf(stderr, "\nProcessing, please wait!\n");

/*
**  Read records, and process them.
*/

        while( read_record() == SUCCESS )
            process_record();

    } /* end else if a test tape */

/*
**  Fix up the file size and the chunk size in the wave file.
**  Compute the lengths.  Write an alignment byte if needed.
**  Position the file at the chunk size and update it and do the
**  same for the file size.
*/
    len_chunk = pos - pos_chunk_size - 4;
    len_file  = pos - pos_file_size - 4;
    if( len_chunk & 0x01L )
        write_wav( (char *)"\0", (uint32)1L );     /* Write alignment byte */
    fseek( wav_file, pos_chunk_size, SEEK_SET );   /* Go back to chunk size */
    write_wav_number( len_chunk, (uint32)4L );
    fseek( wav_file, pos_file_size, SEEK_SET );    /* Go back to file size */
    write_wav_number( len_file, (uint32)4L );

    cleanup();
    return 0;
}

/*****************************************************************************
**  MODIFICATION HISTORY
**
**  DATE        BY   Description
**  ----------  ---  ---------------------------------------------------------
**  1998/12/30  ERS  Project start
**  1999/04/26  ERS  Release 01.00
**
*****************************************************************************/

