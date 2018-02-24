// Firefly artwork for RDU at St. Thomas' Hospital.

// Written to run on Adafruit Feather M0 Basic Proto.

// Constructed from 20 strips of 150 WWW Dotstar LEDs each.
// https://www.adafruit.com/product/2435?length=5

// These strips are daisy-chained in two chains of 10 strips each
// with power feed-in every two strips. We used two SPI ports
// (one for each chain) to keep the data-rate and overall chain length
// down in order to keep a nice snappy 50 fps and avoiding jitter-induced
// noise due to the ropey reclocking implementation on each Dotstar.

// Heavily influenced by Nicky Case's excellent work here
// http://ncase.me/fireflies/

// ---------- BEGIN TWEAK ZONE ---------- 
// stuff in here is easy to play with and tweak
const uint16_t firefly_count_max = 180;  // how many fireflies in the artwork
// all uint16_t and uint32_t below are in frames (20ms per frame)
// all floats are 0..1
const uint16_t envelope_on_frames = 45000; // ~15 minutes
const uint16_t envelope_off_frames = 0; // ~ 0 minutes
const uint16_t firefly_period_min = 7000;
const uint16_t firefly_period_max = 10000;
const uint16_t firefly_pulsewidth_min = 60;
const uint16_t firefly_pulsewidth_max = 80;
const uint32_t firefly_lifetime_min = 30000;
const uint32_t firefly_lifetime_max = 90000;
const uint16_t firefly_nudge_min = 80; // minimum amount of shift caused by flash-effect
const uint16_t firefly_nudge_max = 240; // maximum amount of shift caused by flash-effect
const float firefly_initial_birth_probablilty = 0.1 ; // what proportion of fireflies will be alive at the start
const float firefly_birth_probablilty = 0.00133 ; // limited by a maximum count
const float firefly_effect_probability = 0.01; // probability that one firefly flash affects any other
// ----------- END  TWEAK ZONE ----------- 

#include <SPI.h>
#include "wiring_private.h" 
#include <Adafruit_ASFcore.h>
#include "status_codes.h"
#include <Adafruit_ZeroDMA.h>
#include "utility/dmac.h"
#include "utility/dma.h"

// Dotstar strips
const uint16_t num_cols = 150;
const uint16_t num_rows = 20;
const uint16_t bytes_per_pixel = 4;
const uint16_t start_frame_size = 4;
const uint16_t end_frame_size = 1*((( num_cols * num_rows / 2 ) / ( 2 * 8 )) + 1);
const uint16_t num_pixels = num_cols * num_rows;
const uint16_t output_data_size = ( num_pixels / 2 ) * bytes_per_pixel;
const uint16_t output_frame_size = start_frame_size + output_data_size + end_frame_size;
const uint32_t spi_clock_Hz = 2500000;
const uint8_t  global_led_brightness = 31; // valid 0..31
const uint16_t max_pixel_value = 0xFF;

// 7 segment display for mode indication
const int seg_c_pin = 9;
const int seg_dp_pin = 10;
const int seg_b_pin = 11;
const int seg_a_pin = 12;
const int seg_g_pin = A2;
const int seg_f_pin = A3;
const int seg_e_pin = A4;
const int seg_d_pin = A5;

// button for changing modes (momentary, active low)
const int button_pin = 5;
const uint8_t button_bounce_ignore_frames = 15;
uint8_t button_bounce_frame_count = 0;

// common SPI settings
SPISettings spi_settings = SPISettings( spi_clock_Hz, MSBFIRST, SPI_MODE0 );

// second SPI port
const int SPI2_MOSI_pin = 20;
const int SPI2_MISO_pin = 6;
const int SPI2_SCK_pin = 21;
SPIClass SPI2 ( &sercom3, SPI2_MISO_pin, SPI2_SCK_pin, SPI2_MOSI_pin, SPI_PAD_0_SCK_1, SERCOM_RX_PAD_2 );

// DMA
volatile bool transfer_is_done_top = false;
volatile bool transfer_is_done_bottom = false;
Adafruit_ZeroDMA spi_dma_top, spi_dma_bottom;
status_code dma_status_top, dma_status_bottom;

// generate a more random seed for random number generation
const int entropy_source_pin = A0;

// test / artwork modes
const uint8_t l_r_sweep_spacing = num_cols;
const uint8_t u_d_sweep_frame_dwell = 7; // slow down so we're speed comparable with l_r_sweep
uint8_t u_d_sweep_frame_count = 0;
enum artwork_mode_type { fireflies, l_r_sweep, u_d_sweep } artwork_mode = fireflies;
const uint8_t num_artwork_modes = 3;
uint16_t test_index = 0;
uint16_t firefly_count = 0;
uint16_t envelope_frame_counter = 0;
bool allow_new_activations = true;
enum firefly_state_type { active, flashing, dormant };
struct firefly_type {
   uint16_t age;
   uint16_t period;
   uint8_t pulsewidth;
   uint16_t position;
   uint32_t lifetime;
   bool schedule_for_hibernation;   
   firefly_state_type state;
};
firefly_type firefly_array[ firefly_count_max ];

// timing
uint32_t this_micros = 0;
uint32_t last_micros = 0;
const uint32_t tick_interval_micros = 20000; // 50 fps

// data buffers
uint8_t output_frame_a[ output_frame_size ];
uint8_t output_frame_b[ output_frame_size ];
uint8_t output_frame_c[ output_frame_size ];
uint8_t output_frame_d[ output_frame_size ];
uint8_t *output_frame_top = output_frame_a;
uint8_t *buffer_frame_top = output_frame_b;
uint8_t *output_frame_bottom = output_frame_c;
uint8_t *buffer_frame_bottom = output_frame_d;

void dma_callback_top( struct dma_resource* const resource ) {
    transfer_is_done_top = true;
}

void dma_callback_bottom( struct dma_resource* const resource ) {
    transfer_is_done_bottom = true;
}

void setup() {
  // GPIO setup
  pinMode( seg_a_pin, OUTPUT );
  digitalWrite( seg_a_pin, LOW );
  pinMode( seg_b_pin, OUTPUT );
  digitalWrite( seg_b_pin, LOW );
  pinMode( seg_c_pin, OUTPUT );
  digitalWrite( seg_c_pin, LOW );
  pinMode( seg_d_pin, OUTPUT );
  digitalWrite( seg_d_pin, LOW );
  pinMode( seg_e_pin, OUTPUT );
  digitalWrite( seg_e_pin, LOW );
  pinMode( seg_f_pin, OUTPUT );
  digitalWrite( seg_f_pin, LOW );
  pinMode( seg_g_pin, OUTPUT );
  digitalWrite( seg_g_pin, LOW );
  pinMode( seg_dp_pin, OUTPUT );
  digitalWrite( seg_dp_pin, LOW );
  pinMode( button_pin, INPUT_PULLUP );
  
  pinMode( LED_BUILTIN, OUTPUT );
  digitalWrite( LED_BUILTIN, LOW );

  SPI.begin();

  spi_dma_top.configure_peripheraltrigger( SERCOM4_DMAC_ID_TX );  // SERMCOM4 == SPI native SERCOM
  spi_dma_top.configure_triggeraction( DMA_TRIGGER_ACTON_BEAT );

  dma_status_top = spi_dma_top.allocate();
  dma_status_top = spi_dma_top.add_descriptor();
  
  spi_dma_top.register_callback( dma_callback_top ); // by default, called when xfer done
  spi_dma_top.enable_callback(); // by default, for xfer done registers   

  SPI2.begin();

  pinPeripheral( SPI2_MISO_pin, PIO_SERCOM_ALT );
  pinPeripheral( SPI2_MOSI_pin, PIO_SERCOM );
  pinPeripheral( SPI2_SCK_pin, PIO_SERCOM );  

  spi_dma_bottom.configure_peripheraltrigger( SERCOM3_DMAC_ID_TX );  // SERMCOM3
  spi_dma_bottom.configure_triggeraction( DMA_TRIGGER_ACTON_BEAT );

  dma_status_bottom = spi_dma_bottom.allocate();
  dma_status_bottom = spi_dma_bottom.add_descriptor();
  
  spi_dma_bottom.register_callback( dma_callback_bottom ); // by default, called when xfer done
  spi_dma_bottom.enable_callback(); // by default, for xfer done registers   

  set_mode( artwork_mode );

  // quick loop to seed random with better entropy than analogRead alone
  uint32_t random_seed = 0;
  for( uint8_t i=0; i<16; i++ ){
    random_seed |= analogRead( entropy_source_pin ) & 0x03; // add bottom two bits of reading to seed
    random_seed = random_seed << 2;         // shift up two bits
  }

  randomSeed( random_seed );

  // initialise firefly array
  for( uint16_t i=0; i<firefly_count_max; i++ ){
    firefly_array[ i ].position = num_pixels; // out of bounds for clash check avoidance
    firefly_array[ i ].state= dormant;
  }

  for( uint16_t i=0; i<firefly_count_max; i++ ){
    if( randomFloat() <= firefly_initial_birth_probablilty ){
      activate_firefly( firefly_array, i );
    }
  }
}

void generate_next_frame(){
  // always write to buffer_frame in here
  digitalWrite( LED_BUILTIN, HIGH );

  uint16_t buffer_index;
  bool nudge = false;  
  uint16_t num_periods = 0;

  switch( artwork_mode ){
    case fireflies:

      // calculate global envelope state
      envelope_frame_counter++;
      if( allow_new_activations ){
        if( envelope_frame_counter >= envelope_on_frames ){
          envelope_frame_counter = 0;
          allow_new_activations = false;
        }
      } else {
        if( envelope_frame_counter >= envelope_off_frames ){
          envelope_frame_counter = 0;
          allow_new_activations = true;
        }
      }

      memset( buffer_frame_top + start_frame_size, 0x00, output_data_size );
      memset( buffer_frame_bottom + start_frame_size, 0x00, output_data_size );

      for( uint16_t pixel_number = 0; pixel_number < ( num_pixels / 2 ); pixel_number++ ){
        buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  ]       = 0xE0 | global_led_brightness; 
        buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  ]       = 0xE0 | global_led_brightness; 
      }
      for( uint16_t i=0; i<firefly_count_max; i++ ){
        switch( firefly_array[ i ].state ){

          case dormant:
            if( allow_new_activations && ( firefly_count < firefly_count_max ) && ( randomFloat() <= firefly_birth_probablilty )){
              activate_firefly( firefly_array, i );
            }
            break;

          case active:
            firefly_array[ i ].age++;
            if( firefly_array[ i ].age >= firefly_array[ i ].lifetime ){
              hibernate_firefly( firefly_array, i );
            } else if( firefly_array[ i ].age % firefly_array[ i ].period == 0 ){
              firefly_array[ i ].state = flashing;
              if( randomFloat() <= firefly_effect_probability ){
                nudge = true;
              }
            }           
            break;

          case flashing:
            firefly_array[ i ].age++;

            if( firefly_array[ i ].age % firefly_array[ i ].period == firefly_array[ i ].pulsewidth ){
              firefly_array[ i ].state = active;
            } else {
              uint8_t flash_value = ( uint8_t )(( float )( max_pixel_value ) * 0.5 * ( 1.f - cos( TWO_PI * ( float )( firefly_array[ i ].age % firefly_array[ i ].period ) / ( float )( firefly_array[ i ].pulsewidth ))));
              if( firefly_array[ i ].position < num_pixels / 2 ){
                buffer_frame_top[ start_frame_size + ( bytes_per_pixel * firefly_array[ i ].position)  + 1 ] = flash_value;
                buffer_frame_top[ start_frame_size + ( bytes_per_pixel * firefly_array[ i ].position)  + 2 ] = flash_value;
                buffer_frame_top[ start_frame_size + ( bytes_per_pixel * firefly_array[ i ].position)  + 3 ] = flash_value;                        
              } else {
                buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * ( firefly_array[ i ].position - ( num_pixels / 2 )))  + 1 ] = flash_value;
                buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * ( firefly_array[ i ].position - ( num_pixels / 2 )))  + 2 ] = flash_value;
                buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * ( firefly_array[ i ].position - ( num_pixels / 2 )))  + 3 ] = flash_value;                        
              }
            }
            break;

          default:
            break;
        }
      }

      if( nudge ){
        for( uint16_t i=0; i<firefly_count_max; i++ ){
          if( firefly_array[ i ].state==active ){
            num_periods = firefly_array[ i ].age / firefly_array[ i ].period;
            firefly_array[ i ].age += random( firefly_nudge_min, firefly_nudge_max );
            if( firefly_array[ i ].age >= firefly_array[ i ].lifetime ){
              hibernate_firefly( firefly_array, i );
            } else if( num_periods < ( firefly_array[ i ].age / firefly_array[ i ].period )){
              // if we've nudged into a new flash...
              // round age down to nearest multiple of a period using integer division
              firefly_array[ i ].age = ( firefly_array[ i ].age / firefly_array[ i ].period ) * firefly_array[ i ].period;
              firefly_array[ i ].state = flashing;
            }           
          }
        }        
      }
      break;

    case l_r_sweep:
      if( ++test_index >= l_r_sweep_spacing ){
        test_index = 0;
      }      
      memset( buffer_frame_top + start_frame_size, 0x00, output_data_size );
      memset( buffer_frame_bottom + start_frame_size, 0x00, output_data_size );

      for( uint16_t pixel_number = 0; pixel_number < ( num_pixels / 2 ); pixel_number++ ){
        buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  ]       = 0xE0 | global_led_brightness; 
        buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  ]       = 0xE0 | global_led_brightness; 
        if(((( pixel_number / num_cols ) % 2 == 0 ) && ( pixel_number % l_r_sweep_spacing == test_index )) || 
           ((( pixel_number / num_cols ) % 2 == 1 ) && ( pixel_number % l_r_sweep_spacing == num_cols - 1 - test_index ))){
          buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 1 ] = max_pixel_value;
          buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 2 ] = max_pixel_value;
          buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 3 ] = max_pixel_value;
          buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 1 ] = max_pixel_value;
          buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 2 ] = max_pixel_value;
          buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 3 ] = max_pixel_value;          
        }
      }
      break;

    case u_d_sweep:
      // need to do this for both buffers
      if( u_d_sweep_frame_count <= 1 ){
        if( u_d_sweep_frame_count == 0){ // only increment test_index for first buffer
          if( ++test_index >= num_rows ){
            test_index = 0;
          }
        }
        memset( buffer_frame_top + start_frame_size, 0x00, output_data_size );
        memset( buffer_frame_bottom + start_frame_size, 0x00, output_data_size );

        for( uint16_t pixel_number = 0; pixel_number < ( num_pixels / 2 ); pixel_number++ ){
          buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  ]       = 0xE0 | global_led_brightness; 
          buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  ]       = 0xE0 | global_led_brightness; 
          if( pixel_number  / num_cols == test_index ){
            buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 1 ] = max_pixel_value;
            buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 2 ] = max_pixel_value;
            buffer_frame_top[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 3 ] = max_pixel_value;
          } else if(( pixel_number + ( num_pixels / 2 )) / num_cols == test_index ){
            buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 1 ] = max_pixel_value;
            buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 2 ] = max_pixel_value;
            buffer_frame_bottom[ start_frame_size + ( bytes_per_pixel * pixel_number)  + 3 ] = max_pixel_value;
          }
        }
      }
      if( ++u_d_sweep_frame_count >= u_d_sweep_frame_dwell ){
        u_d_sweep_frame_count = 0;
      }
      break; 

    default:
      break;
  }

  digitalWrite( LED_BUILTIN, LOW );
}

void loop() {
  this_micros = micros();
 
  if(( this_micros - last_micros ) > tick_interval_micros ){
    last_micros = this_micros;

    transfer_is_done_top = false;
    transfer_is_done_bottom = false;
  
    SPI.begin();

    spi_dma_top.setup_transfer_descriptor( output_frame_top,              // move data from here
                                    ( void * )( &SERCOM4->SPI.DATA.reg ), // to here
                                    output_frame_size,                    // this many...
                                    DMA_BEAT_SIZE_BYTE,                   // bytes/hword/words
                                    true,                                 // increment source addr?
                                    false );                              // increment dest addr?
    
    SPI.beginTransaction( spi_settings );
    
    dma_status_top = spi_dma_top.start_transfer_job();

    SPI2.begin();

    spi_dma_bottom.setup_transfer_descriptor(output_frame_bottom,         // move data from here
                                    ( void * )( &SERCOM3->SPI.DATA.reg ), // to here
                                    output_frame_size,                    // this many...
                                    DMA_BEAT_SIZE_BYTE,                   // bytes/hword/words
                                    true,                                 // increment source addr?
                                    false );                              // increment dest addr?
    
    SPI2.beginTransaction( spi_settings );
    
    dma_status_bottom = spi_dma_bottom.start_transfer_job();

    if( button_bounce_frame_count > 0 ){
      if( ++button_bounce_frame_count > button_bounce_ignore_frames ){
        button_bounce_frame_count = 0;
      }
    } else if( !digitalRead( button_pin )){
      // start at 1 so we have count and state in one var
      button_bounce_frame_count = 1;
      artwork_mode = ( artwork_mode_type )((( uint8_t )artwork_mode + 1 ) % num_artwork_modes );
      set_mode( artwork_mode );
    }

    generate_next_frame();    
    
    while( !( transfer_is_done_top && transfer_is_done_bottom ));

    SPI.endTransaction(); 
    SPI2.endTransaction();  

    // swap output and buffer frames
    if( output_frame_top == output_frame_a ){
      output_frame_top = output_frame_b;
      buffer_frame_top = output_frame_a;
      output_frame_bottom = output_frame_d;
      buffer_frame_bottom = output_frame_c;      
    } else {
      output_frame_top = output_frame_a;
      buffer_frame_top = output_frame_b;
      output_frame_bottom = output_frame_c;
      buffer_frame_bottom = output_frame_d;      
    }
  }  
}

void set_mode( artwork_mode_type mode ){
  memset( output_frame_a, 0x00, output_frame_size - end_frame_size );
  memset( output_frame_b, 0x00, output_frame_size - end_frame_size );  
  memset( output_frame_a + output_frame_size - end_frame_size, 0xFF, end_frame_size );
  memset( output_frame_b + output_frame_size - end_frame_size, 0xFF, end_frame_size );

  memset( output_frame_c, 0x00, output_frame_size - end_frame_size );
  memset( output_frame_d, 0x00, output_frame_size - end_frame_size );  
  memset( output_frame_c + output_frame_size - end_frame_size, 0xFF, end_frame_size );
  memset( output_frame_d + output_frame_size - end_frame_size, 0xFF, end_frame_size );  

  switch( mode ){
    case fireflies:
      // 'r' (run)
      digitalWrite( seg_a_pin, LOW );
      digitalWrite( seg_b_pin, HIGH );
      digitalWrite( seg_c_pin, LOW );
      digitalWrite( seg_d_pin, LOW );
      digitalWrite( seg_e_pin, LOW );
      digitalWrite( seg_f_pin, LOW );
      digitalWrite( seg_g_pin, HIGH );
      digitalWrite( seg_dp_pin, LOW );
      break;

    case l_r_sweep:
      // '1'
      digitalWrite( seg_a_pin, LOW );
      digitalWrite( seg_b_pin, LOW );
      digitalWrite( seg_c_pin, LOW );
      digitalWrite( seg_d_pin, LOW );
      digitalWrite( seg_e_pin, HIGH );
      digitalWrite( seg_f_pin, HIGH );
      digitalWrite( seg_g_pin, LOW );
      digitalWrite( seg_dp_pin, LOW );
      break;      

    case u_d_sweep:
      // '2'
      digitalWrite( seg_a_pin, HIGH );
      digitalWrite( seg_b_pin, HIGH );
      digitalWrite( seg_c_pin, LOW );
      digitalWrite( seg_d_pin, HIGH );
      digitalWrite( seg_e_pin, HIGH );
      digitalWrite( seg_f_pin, LOW );
      digitalWrite( seg_g_pin, HIGH );
      digitalWrite( seg_dp_pin, LOW );
      break;      

    default:
      break;
  }
}

void activate_firefly( firefly_type firefly_array[], uint16_t index ){
  firefly_array[ index ].age = 0;
  firefly_array[ index ].period = random( firefly_period_min, firefly_period_max );
  firefly_array[ index ].pulsewidth = random( firefly_pulsewidth_min, firefly_pulsewidth_max );
  
  // find a position for the firefly, checking for clashes
  uint16_t proposed_position = random( num_pixels );
  for( uint16_t j=0; j<firefly_count_max; j++ ){
    if( firefly_array[ j ].position == proposed_position ){
      proposed_position = random( num_pixels );
      j = 0; // this runs a VERY small chance of never finishing, but it should be negligible in the lifetime of the artwork
    }
  }
  firefly_array[ index ].position = proposed_position;
  firefly_array[ index ].lifetime = random( firefly_lifetime_min, firefly_lifetime_max );
  firefly_array[ index ].schedule_for_hibernation = false; 
  firefly_array[ index ].state = active;  

  firefly_count++;
}

void hibernate_firefly( firefly_type firefly_array[], uint16_t index ){
  firefly_array[ index ].state = dormant;  
  firefly_count--;
}

float randomFloat(){
  return( static_cast <float> ( random( INT32_MAX )) / static_cast <float> ( INT32_MAX - 1 ));
} 
