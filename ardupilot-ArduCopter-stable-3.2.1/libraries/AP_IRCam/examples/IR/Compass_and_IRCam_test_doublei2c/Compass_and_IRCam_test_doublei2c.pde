/*
 *       Example of APM_Compass library (HMC5843 sensor).
 *       Code by Jordi MuÒoz and Jose Julio. DIYDrones.com
 */

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_Empty.h>
#include <AP_HAL_VRBRAIN.h>

#include <AP_Math.h>    // ArduPilot Mega Vector/Matrix math Library
#include <AP_Declination.h>
#include <AP_Compass.h> // Compass Library
#include <IR_camera.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_Math.h>
#include <AP_HAL.h>
#include <AP_Buffer.h>
#include <Filter.h>
#include <AP_Baro.h>
#include <AP_Notify.h>
#include <AP_GPS.h>
#include <GCS_MAVLink.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>
#include <AP_InertialSensor.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Linux.h>
#include <AP_HAL_FLYMAPLE.h>
#include <AP_HAL_PX4.h>
#include <AP_HAL_Empty.h>

#include <AP_HAL.h>
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_ADC_AnalogSource.h>
#include <AP_Baro.h>            // ArduPilot Mega Barometer Library
#include <AP_GPS.h>
#include <AP_AHRS.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <AP_Airspeed.h>
#include <AP_Baro.h>
#include <GCS_MAVLink.h>
#include <AP_Mission.h>
#include <StorageManager.h>
#include <AP_Terrain.h>
#include <Filter.h>
#include <SITL.h>
#include <AP_Buffer.h>
#include <AP_Notify.h>
#include <AP_Vehicle.h>
#include <DataFlash.h>

#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_HAL_Empty.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

#define CONFIG_COMPASS HAL_COMPASS_DEFAULT

#if CONFIG_COMPASS == HAL_COMPASS_PX4
static AP_Compass_PX4 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_VRBRAIN
static AP_Compass_VRBRAIN compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HMC5843
static AP_Compass_HMC5843 compass;
#elif CONFIG_COMPASS == HAL_COMPASS_HIL
static AP_Compass_HIL compass;
#else
 #error Unrecognized CONFIG_COMPASS setting
#endif

#define CONFIG_BARO HAL_BARO_DEFAULT

#if CONFIG_BARO == HAL_BARO_BMP085
static AP_Baro_BMP085 barometer;
#elif CONFIG_BARO == HAL_BARO_PX4
static AP_Baro_PX4 barometer;
#elif CONFIG_BARO == HAL_BARO_VRBRAIN
static AP_Baro_VRBRAIN barometer;
#elif CONFIG_BARO == HAL_BARO_HIL
static AP_Baro_HIL barometer;
#elif CONFIG_BARO == HAL_BARO_MS5611
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::i2c);
#elif CONFIG_BARO == HAL_BARO_MS5611_SPI
static AP_Baro_MS5611 barometer(&AP_Baro_MS5611::spi);
#else
 #error Unrecognized CONFIG_BARO setting
#endif

//AP_InertialSensor_MPU6000 ins;
//AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

AP_InertialSensor ins;

AP_GPS  gps;

//AP_Baro_MS5611 baro(&AP_Baro_MS5611::spi);

AP_AHRS_DCM ahrs(ins, barometer, gps);

//AP_GPS gps;

//AP_AHRS_DCM  ahrs(ins, baro, gps);

uint32_t timer;

float x = 0.0;
float y = 0.0;
float x_ahrs = 0.0;
float y_ahrs = 0.0;
float pix_range = 0.0;
float height = 0.0;
float difference = 0.0;

IRCamera ircam(ahrs, barometer);

#define HIGH 1
#define LOW 0

void setup() {
 //   hal.console->println("Compass library test");

    if (!compass.init()) {
      //  hal.console->println("compass initialisation failed!");
        while (1) ;
    }
 //   hal.console->println("init done");

    compass.set_and_save_offsets(0,0,0,0); // set offsets to account for surrounding interference
    compass.set_declination(ToRad(0.0)); // set local difference between magnetic north and true north

  //  hal.console->print("Compass auto-detected as: ");
    switch( compass.product_id ) {
    case AP_COMPASS_TYPE_HIL:
     //   hal.console->println("HIL");
        break;
    case AP_COMPASS_TYPE_HMC5843:
     //   hal.console->println("HMC5843");
        break;
    case AP_COMPASS_TYPE_HMC5883L:
     //   hal.console->println("HMC5883L");
        break;
    case AP_COMPASS_TYPE_PX4:
     //   hal.console->println("PX4");
        break;
    default:
     //   hal.console->println("unknown");
        break;
    }

    hal.scheduler->delay(1000);
    timer = hal.scheduler->micros();
    
     if(ircam.init(3))
     { 
     //hal.console->println("IRCamera Init OK");
     }
        else{
        //hal.console->println("IRCamera Init FAILED");
        };
    hal.scheduler->delay(1000);
    
 //   hal.console->println("Barometer library test");

    hal.scheduler->delay(1000);

    #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    // disable CS on MPU6000
    hal.gpio->pinMode(63, HAL_GPIO_OUTPUT);
    hal.gpio->write(63, 1);
    #endif

    barometer.init();
    barometer.calibrate();
    
    #ifdef APM2_HARDWARE
    // we need to stop the barometer from holding the SPI bus
    hal.gpio->pinMode(40, HAL_HAL_GPIO_OUTPUT);
    hal.gpio->write(40, HIGH);
#endif

    ins.init(AP_InertialSensor::COLD_START, 
			 AP_InertialSensor::RATE_100HZ);
    ins.init_accel();

    ahrs.init();

    if( compass.init() ) {
      //  hal.console->printf("Enabling compass\n");
        ahrs.set_compass(&compass);
    } else {
       // hal.console->printf("No compass detected\n");
    }
    gps.init(NULL);
    
   // hal.console->println("READY");
    
     hal.scheduler->delay(1000);
    
}

void loop()
{
    static uint16_t counter;
    static uint32_t last_t, last_print, last_compass;
    uint32_t now = hal.scheduler->micros();
    float heading = 0;

    static float min[3], max[3], offset[3];

    compass.accumulate();

    if((hal.scheduler->micros()- timer) > 100000L)
    {
        timer = hal.scheduler->micros();
        compass.read();
        unsigned long read_time = hal.scheduler->micros() - timer;
        float heading;

        if (!compass.healthy()) {
            hal.console->println("not healthy");
            return;
        }
	Matrix3f dcm_matrix;
	// use roll = 0, pitch = 0 for this example
	dcm_matrix.from_euler(0, 0, 0);
        heading = compass.calculate_heading(dcm_matrix);
        compass.learn_offsets();

        // capture min
        const Vector3f &mag = compass.get_field();
        if( mag.x < min[0] )
            min[0] = mag.x;
        if( mag.y < min[1] )
            min[1] = mag.y;
        if( mag.z < min[2] )
            min[2] = mag.z;

        // capture max
        if( mag.x > max[0] )
            max[0] = mag.x;
        if( mag.y > max[1] )
            max[1] = mag.y;
        if( mag.z > max[2] )
            max[2] = mag.z;

        // calculate offsets
        offset[0] = -(max[0]+min[0])/2;
        offset[1] = -(max[1]+min[1])/2;
        offset[2] = -(max[2]+min[2])/2;

  /*      // display all to user
        hal.console->printf("Heading: %.2f (%3d,%3d,%3d) i2c error: %u",
			    ToDeg(heading),
			    (int)mag.x,
			    (int)mag.y,
			    (int)mag.z, 
			    (unsigned)hal.i2c->lockup_count());

        // display offsets
        hal.console->printf(" offsets(%.2f, %.2f, %.2f)",
                      offset[0], offset[1], offset[2]);

        hal.console->printf(" t=%u", (unsigned)read_time);

        hal.console->println();*/
    } else {
	    hal.scheduler->delay(1);
    }
    
     compass.read(); 
        heading = compass.calculate_heading(ahrs.get_dcm_matrix());
        // read compass at 10Hz
        last_compass = now;
    #if WITH_GPS
        g_gps->update();
    #endif
    ahrs.update();
    
    
    timer = hal.scheduler->micros();
    if(ircam.read())
            {   barometer.read();
                x = ircam.get_compensated_error_x();
                y = ircam.get_compensated_error_y();      
                ircam.processing_test();
                height = ircam.get_cm_alt();
                unsigned long read_time = hal.scheduler->micros() - timer;
             //   hal.console->println(read_time);
            }
        else
            {   
                hal.console->println("Read Failed");
            };
  /*  hal.console->print(x);
    hal.console->print("[cm]");
    hal.console->print(".....");
    hal.console->print(y);
    hal.console->println("[cm]   ");*/ 
  //  height = (20.0/(tanf(pix_range/1314.9834499663)));
   // hal.console->print("Height: ");
   // hal.console->println(height);
    
    
   // float alt = barometer.get_altitude();
    
    if (!barometer.healthy()) {
            hal.console->println("not healthy");
            return;
        }
    hal.scheduler->delay(15);
  //  difference = ((height-(alt*100.0)) + 23.0); // barometr jest umieszczony na wys. okolo 23 cm i podaje wynik w m
        
    //hal.console->print("baro-cam difference: ");
  //  hal.console->println(difference);
    
  //  hal.scheduler->delay(500);
    
   // hal.console->print(" Altitude:");
    // hal.console->println(alt*100);    
    
    //DUPADUPA DUPA DUPA DUPA
    
}

AP_HAL_MAIN();