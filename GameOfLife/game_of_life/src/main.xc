// COMS20001 - Cellular Automaton Farm - Initial Code Skeleton
// (using the XMOS i2c accelerometer demo code)

#include <platform.h>
#include <xs1.h>
#include <stdio.h>
#include "pgmIO.h"
#include "i2c.h"

#define  IMHT           400                  //image height
#define  IMWD           400                  //image width
#define  ALIVE          1
#define  DEAD           0

//16x16   round time:  60942340
//64x64   round time:  65305667
//128x128 round time: 75632055
//512x512 round time: 97064285
//400x400 round time: 88773763

#define  CORE_COUNT     4

//Height definitions for the picture
#define  HT_PCKD        IMHT
#define  HT_PCKD_P_OUT  IMHT        / CORE_COUNT
#define  HT_PCKD_P_IN   HT_PCKD_P_OUT + 2

//We do not accept the image if it's image width is not a multiple of 8 AND CORE_COUNT, from the beginnning
#define  WD_PCKD        IMWD        / 8
#define  WD_PCKD_P_OUT  WD_PCKD
#define  WD_PCKD_P_IN   WD_PCKD_P_OUT


typedef  unsigned short uchar_16;
typedef  unsigned char  uchar;      //using uchar as shorthand

int hash_table[8] = {0,1,1,2,1,2,2,3};

#define  infname        "square.pgm"
#define  outfname       "test.pgm"

on tile[0]: port p_scl = XS1_PORT_1E;         //interface ports to orientation
on tile[0]: port p_sda = XS1_PORT_1F;

on tile[0] : in port BUTTON = XS1_PORT_4E; //port to access xCore-200 buttons
on tile[0] : out port LSD = XS1_PORT_4F;   //port to access xCore-200 LEDs


#define FXOS8700EQ_I2C_ADDR         0x1E  //register addresses for orientation
#define FXOS8700EQ_XYZ_DATA_CFG_REG 0x0E
#define FXOS8700EQ_CTRL_REG_1       0x2A
#define FXOS8700EQ_DR_STATUS        0x0
#define FXOS8700EQ_OUT_X_MSB        0x1
#define FXOS8700EQ_OUT_X_LSB        0x2
#define FXOS8700EQ_OUT_Y_MSB        0x3
#define FXOS8700EQ_OUT_Y_LSB        0x4
#define FXOS8700EQ_OUT_Z_MSB        0x5
#define FXOS8700EQ_OUT_Z_LSB        0x6

void print_byte(uchar byte) {
    for ( uchar b = 0; b < 8; b++) {
        printf( "%d", (byte >> (7-b)) & 1);
    }
    printf( " " );
}

void print_packed_picture(uchar picture[HT_PCKD][WD_PCKD]) {
     uchar x,y;
     for ( y = 0; y < HT_PCKD; y++) {
         for ( x = 0; x < WD_PCKD; x++) {
             print_byte(picture[y][x]);
         }
         printf( "\n" );
     }
}

void print_output_pack_of_processor(uchar picture[HT_PCKD_P_IN][WD_PCKD_P_OUT]) {
     char x,y;
     for ( y = 0; y < HT_PCKD_P_IN; y++) {
         for ( x = 0; x < WD_PCKD_P_OUT; x++) {
             print_byte(picture[y][x]);
         }
         printf( "\n" );
     }
}

uchar mod(char num, char div) {
    return (num + div) % div;
}

void showLEDs(out port p, chanend fromDist) {
  int pattern; //1st bit...separate green LED, 2nd bit...blue LED, 3rd bit...green LED, 4th bit...red LED
  while (1) {
    fromDist :> pattern;   //receive new pattern from visualiser
    p <: pattern;        //send pattern to LED port
  }
}

//READ BUTTONS and send button pattern to distibutor
void buttonListener(in port b, chanend toDist) {
    int r;
    while (1) {
    b when pinseq(15)  :> r;              // check that no button is pressed
    b when pinsneq(15) :> r;              // check if some buttons are pressed
    if ((r==13) || (r==14)) {
      toDist <: r;  // send button pattern to userAnt
      }
    }
}

void DataInStream(chanend c_out, chanend c_in){

        //Hold to start in
        c_out :> int dataInStart;
        timer t;
        uchar line[IMWD], packed_byte;
        uint y, x, b, ts, te;
        printf(" DataInStream: Start...\n" );
        t :> ts;

        if (IMWD % 8 != 0)                    { printf("\n DataInStream: ERROR: Image width must be multiple of 8.\n");                        return;}
        if (HT_PCKD % CORE_COUNT != 0)        { printf("\n DataInStream: ERROR: Packed image height must be multiple of the core count.\n");   return;}
        if (_openinpgm( infname, IMWD, IMHT)) { printf("\n DataInStream: ERROR: Could not open the file. %s\n.", infname);                     return;}

        for(y = 0; y < HT_PCKD; y++ ) {
            _readinline( line, IMWD );
            for(x = 0; x < WD_PCKD; x++ ) {
                packed_byte        = 0;
                    for(b = 0; b < 8; b++ ) {
                        packed_byte |= (line [(8 * x) + b] != 0) << (7-b);
                    }
                    c_out <: packed_byte;
            }
        }
        _closeinpgm();
        t:> te;
        printf( " DataInStream: Done...                 Time: %d ms\n", te - ts);

        //Data Out Start
        while(1){
            c_in :> int stream;
            uchar byte;
            uchar line[ IMWD ];
            printf( "DataOutStream: Start...\n" );
            if( _openoutpgm( outfname, IMWD, IMHT)) printf("\n DataOutStream: ERROR: Could not open the file. %s\n.", outfname);
            printf( "DataOutStream: Receiving from distributor...\n" );
            for(y = 0; y < HT_PCKD; y++ ) {
                for(x = 0; x < WD_PCKD; x++ ) {
                    c_in :> byte;
                    for (int b = 0; b < 8; b++) {
                        line[8*x + b] = (uchar) ((byte >> (7-b) & 1) == 1) * 255;
                    }
                }
                _writeoutline( line, IMWD );
//                printf( "DataOutStream: Line written...\n" );
            }
            //Close the PGM image
            _closeoutpgm();
            printf( "DataOutStream: Done...\n" );
        }
}

//void DataOutStream() {
//    while(1){
//        int res;
//        uchar line[ IMWD ];
//
//        //Open PGM file
//        printf( "DataOutStream: Start...\n" );
//        res = _openoutpgm( outfname, IMWD, IMHT );
//        if( res ) {
//            printf( "DataOutStream: Error opening %s\n.", outfname );
//        }
//
//
//        uchar byte;
//        //Compile each line of the image and write the image line-by-line
//        for( int y = 0; y < HT_PCKD; y++ ) {
//            for( int x = 0; x < WD_PCKD; x++ ) {
//                c_in :> byte;
//                for (int b = 0; b < 8; b++) {
//                    line[8*x + b] = 255 * ((byte >> (7-b)) & 1);
//                }
//            }
//            _writeoutline( line, IMWD );
//            printf( "DataOutStream: Line written...\n" );
//        }
//
//        //Close the PGM image
//        _closeoutpgm();
//        printf( "DataOutStream: Done...\n" );
//    }
//}
//
///////////////////////////////////////////////////////////////////////////////////////////
////
//// Initialise and  read orientation, send first tilt event to channel
////
///////////////////////////////////////////////////////////////////////////////////////////
//void orientation( client interface i2c_master_if i2c, chanend toDist) {
//  i2c_regop_res_t result;
//  char status_data = 0;
//  int tilted = 0;
//
//  // Configure FXOS8700EQ
//  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
//  if (result != I2C_REGOP_SUCCESS) {
//    printf("I2C write reg failed\n");
//  }
//
//  // Enable FXOS8700EQ
//  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
//  if (result != I2C_REGOP_SUCCESS) {
//    printf("I2C write reg failed\n");
//  }
//
//  //Probe the orientation x-axis forever
//  while (1) {
//
//    //check until new orientation data is available
//    do {
//      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
//    } while (!status_data & 0x08);
//
//    //get new x-axis tilt value
//    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);
//
//    //send signal to distributor after first tilt
//    if (!tilted) {
//      if (x>30) {
//        tilted = 1 - tilted;
//        toDist <: 1;
//      }
//    }
//  }
//}
//
//
//Applies game of life algorithm on the center tile and returns its value

//  printf( "\n Distributor: Start..." );
void distributor(chanend c_in,chanend c_out,chanend fromAcc,chanend processor[n], unsigned int n, chanend toButtons, chanend toLSD) {
    uint rounds = 0, printed_round = 0, alive_count = 0, x, y, value;
    char picture[HT_PCKD][WD_PCKD];
    int counters[CORE_COUNT];
    unsigned long long distStart = 0,distEnd = 0;

    //Wait for start signal
    printf( " Distributor: Image Size = %dx%d\n", IMHT, IMWD );
    printf( " Distributor: Waiting for button signal 13...\n" );
    toButtons :> value;
    if(value != 13) {
        printf( " Distributor: Received button signal 14, waiting for button signal 13...\n" );
        while(value != 13)  toButtons :> value;
    }
    //Start reading in
    printf( " Distributor: Signal received, starting read in...\n" );
    toLSD <: 4;    // Turns on green light
    c_in <: value; //Tell data in stream to begin
    rounds++;      //Update rounds

    printf( " Distributor: Receiving from DataInStream...\n" );
    for ( y = 0; y < HT_PCKD; y++)
        for ( x = 0; x < WD_PCKD; x++)
            c_in :> picture[y][x];
    toLSD <: 0;
    printf( " Distributor: Image received, distributing to processors...\n");
    for ( y = 0; y < HT_PCKD_P_OUT; y++)
        for ( x = 0; x < WD_PCKD_P_OUT; x++)
            par (int z = 0; z < CORE_COUNT; z++)
                processor[z] <: picture[(z * HT_PCKD_P_OUT) + y][x];
    printf( " Distributor: Image distributed, waiting for button signals...\n");
    printed_round++;
    while(1){
        select{
            case fromAcc :> int pause:
                if (pause == 1 && printed_round == rounds) {
                    toLSD <: 8;
                    printf(" Distributor: Round number %d, alive cell count %d, time passed %d\n", rounds, alive_count, 1/*(currentTime - distStart)*/);
                    printed_round++;
                }
                toLSD <: 0;
                break;
            case toButtons :> value:
                uchar p_index, k, i, j;
                //Every case where not round 0
                if(value == 13) {
                    rounds++;
                    printf( " Distributor: Signal received, starting Round %d...\n",rounds);
                    timer t;
                    t :> distStart;

                    toLSD <: 1;
                    for (int z = 0; z < CORE_COUNT; z++)    {
                        processor[z] <: 1;
                        counters[z] = 0;
                    }
                    uchar p_index, k, i, j;
                    //Receive images back from processors
                    printf( " Distributor: Waiting for processors...\n" );

//
//                    if (rounds != 1) {
//                        int alive_counts[CORE_COUNT];
//                        for (int z = 0; z < CORE_COUNT; z++)
//                            processor[z] :> alive_counts[z];
//                        for (int z = 0; z < CORE_COUNT; z++)
//                            alive_count += alive_counts[z];
//                    }
//                    printf("\nAlive count done...");
                    alive_count = 0;
                    for (x = 0; x < HT_PCKD * WD_PCKD; x++)
                        select {
                            case processor[uchar p_index] :> uchar pixel8:
                                if(counters[p_index] == 0) {
                                    alive_count += pixel8;
                                    processor[p_index] :> pixel8;
                                }
                                j = (p_index * HT_PCKD_P_OUT) + (counters[p_index] / ((int) WD_PCKD_P_OUT));
                                i = counters[p_index] % ((int) WD_PCKD_P_OUT);
                                picture[j][i] = pixel8;
                                counters[p_index]++;
                                break;
                        }
                    t :> distEnd;
                    printf(" Distributor: Round %d finished, time: %d...\n",rounds, (distEnd - distStart));

                    toLSD <: 0;
                    //CHANGE reset value, and hold ps until all ready to update
                    value = 0;
                    for(int w = 0; w < CORE_COUNT; w++){
                        processor[w] <: 1;
                    }

                }
                //button 2
                if(value == 14) {
                    printf(" Distributor: Received output signal...\n");
                    toLSD <: 2;
                    c_out <: 1;

//                    print_packed_picture(picture);
                    for(y = 0; y < HT_PCKD; y++ )           //go through all lines
                        for(x = 0; x < WD_PCKD; x++ )       //go through each pixel per line
                            c_out <: (uchar) picture[y][x]; //send some modified pixel out
                    toLSD <: 0;
                    printf(" Distributor: Output done...\n");
                }
          break;
        }
    }
}

void orientation( client interface i2c_master_if i2c, chanend toDist) {
  i2c_regop_res_t result;
  char status_data = 0;

  // Configure FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_XYZ_DATA_CFG_REG, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  // Enable FXOS8700EQ
  result = i2c.write_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_CTRL_REG_1, 0x01);
  if (result != I2C_REGOP_SUCCESS) {
    printf("I2C write reg failed\n");
  }

  //Probe the orientation x-axis forever
  while (1) {
    //check until new orientation data is available
    do {
      status_data = i2c.read_reg(FXOS8700EQ_I2C_ADDR, FXOS8700EQ_DR_STATUS, result);
    } while (!status_data & 0x08);

    //get new x-axis tilt value
    int x = read_acceleration(i2c, FXOS8700EQ_OUT_X_MSB);

    //send signal to distributor after first tilt, if is tilting else is not tilting
    if (x>30)           toDist <: 1;
    else if (x < 25 )   toDist <: 0;
    else                toDist <: 2;
  }
}

void processor(chanend distributor, streaming chanend upper, streaming chanend lower, int n) {
    //printf(" Processor %d: Start...\n", n);
    uint  x, y, b;
    uchar new_byte, n_count, alive_count;
    uchar picture_section   [HT_PCKD_P_IN]  [WD_PCKD_P_IN];
    uchar picture_out       [HT_PCKD_P_OUT] [WD_PCKD_P_OUT];

    //printf(" Processor %d: Receiving data...\n", n);
    for (y = 0; y < HT_PCKD_P_IN - 2; y++)
        for (x = 0; x < WD_PCKD_P_IN; x++)
            distributor :> picture_section [y + 1][x];
    printf(" Processor %d: Received data...\n", n);
    //Loop Start
    while(1){
        //Share top/bottom
        for (x = 0; x < WD_PCKD_P_OUT; x++) {
            par {
                lower <: picture_section[HT_PCKD_P_IN - 2][x];
                upper <: picture_section[1][x];
            }
            par {
                lower :> picture_section[HT_PCKD_P_IN - 1][x];
                upper :> picture_section[0][x];
            }
        }
        //Wait for start signal
        distributor :> int GO;
        printf(" Processor %d: Starting...\n", n);
        //Apply algorithm
        alive_count = 0;
        for (y = 1; y < HT_PCKD_P_OUT + 1; y++) {
            for (x = 0; x < WD_PCKD_P_OUT; x++) {
                new_byte = 0;
                for (b = 0; b < 8; b++) {
                      int is_alive      = (picture_section[y]    [x] >> (7-b)) & 1;
                      int three_line[3];
                      //CHANGE
                      if (b == 7){
                          if(x == WD_PCKD_P_OUT - 1){
                                //Get two bits above and top left, then get top right from byte x+1
                                three_line[2] = ((picture_section[y + 1][x] & 3) << 1) | ((picture_section[y+1][0] >> 7) & 1);
                                three_line[1] = ((picture_section[y][x] & 2) << 1)     | ((picture_section[y][0] >> 7) & 1);
                                three_line[0] = ((picture_section[y - 1][x] & 3) << 1) | ((picture_section[y-1][0] >> 7) & 1);
                          }
                          else{
                              //Get two bits above and top left, then get top right from byte x+1
                              three_line[2] = ((picture_section[y + 1][x] & 3) << 1) | ((picture_section[y+1][x+1] >> 7) & 1);
                              three_line[1] = ((picture_section[y][x] & 2) << 1)     | ((picture_section[y][x+1] >> 7) & 1);
                              three_line[0] = ((picture_section[y - 1][x] & 3) << 1) | ((picture_section[y-1][x+1] >> 7) & 1);
                          }
                      }
                      //CHANGE all ^^ new
                      else{
                      //CHANGE added else around below
                          for (int l = -1; l < 2; l++) {
                              three_line[l + 1] = (picture_section[y + l][x] >> (6-b)) & 7;
                              if (b == 0) {
                                  if (x == 0)       three_line[l + 1] |= (picture_section[y + l][WD_PCKD_P_OUT - 1]   << 2) & 4;
                                  else              three_line[l + 1] |= (picture_section[y + l][x - 1]               << 2) & 4;
                              }
//                              else if (b == 7) {
//                                  if (x == WD_PCKD_P_OUT - 1)  three_line[l + 1]   |= ((picture_section[y + l] [0] >> 7) & 1);
//                                  else                         three_line[l + 1]   |= ((picture_section[y + l] [x + 1] >> 7) & 1);
//                              }
                          }
                      }
                      three_line[1] &= 5;
                      n_count = hash_table[three_line[0]] + hash_table[three_line[1]] + hash_table[three_line[2]];

//                      if((b == 7) && (y == 10)) printf("\n\nn_count: %d\n\n", n_count);

                      int result = ((n_count == 3) || (is_alive & (n_count == 2)));
                      new_byte |=  result << (7-b);
                      alive_count += result;
                }
                picture_out[y - 1][x] = new_byte;
            }
        }
        printf( " Processor %d: Feeding back...\n", n);
        distributor <: alive_count;
        for (y = 0; y < HT_PCKD_P_OUT; y++)
            for (x = 0; x < WD_PCKD_P_OUT; x++) {
                picture_section[y + 1][x] = picture_out[y][x];
                distributor              <: picture_out[y][x];
            }
        printf( "\n Processor %d: Done...", n);

        //CHANGE dont update until all processors are done
        distributor :> int all_p_done;

    }
}

int main(void) {


    i2c_master_if i2c[1];               //interface to orientation

    chan c_inIO, c_outIO, c_control, c_leds, c_buttons; //extend your channel definitions here

    chan c_processors[CORE_COUNT];

    streaming chan c_interprocessor[CORE_COUNT];

    par {
        on tile[0] : showLEDs(LSD, c_leds);
        on tile[0] : buttonListener(BUTTON, c_buttons);
        on tile[0] : i2c_master(i2c, 1, p_scl, p_sda, 10);   //server thread providing orientation data
        on tile[0] : orientation(i2c[0],c_control);        //client thread reading orientation data
        on tile[0] : DataInStream(c_inIO, c_outIO);          //thread to read in a PGM image
//        on tile[0] : DataOutStream(c_outIO);       //thread to write out a PGM image
        on tile[0] : distributor(c_inIO, c_outIO, c_control,c_processors, CORE_COUNT, c_buttons, c_leds);//thread to coordinate work on image
        par (int x = 0; x < CORE_COUNT; x++)
             on tile[1] : processor(c_processors[x], c_interprocessor[x],c_interprocessor[(x + 1) % (int) CORE_COUNT], x + 1);
    }

  return 0;
}
