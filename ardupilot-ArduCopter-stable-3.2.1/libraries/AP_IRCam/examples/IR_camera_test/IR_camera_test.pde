#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_Progmem.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <IR_camera.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

IRCamera ircam;

uint32_t start_t;
uint32_t end_t;

float x = 0.0;
float y = 0.0;

void setup()
{       
        hal.scheduler->delay(3000);
        
        hal.console->println("start");
        start_t = hal.scheduler->micros();
        
        if(ircam.init(3)){ hal.console->println("Init OK");}
        else{hal.console->println("Init FAILED");};
        end_t = hal.scheduler->micros() - start_t;
        hal.console->println(end_t);
        
        hal.scheduler->delay(3000);
}

void loop()
{      
        //hal.scheduler->delay(3000);
       /* hal.console->println("start");
        start_t = hal.scheduler->micros();*/

       // ircam.read();
       // float x = ircam.get_landpoint_midpoint_distance_x();
       // float y = ircam.get_landpoint_midpoint_distance_y();
        if(ircam.read())
            {
                x = ircam.get_landpoint_midpoint_distance_x();
                y = ircam.get_landpoint_midpoint_distance_y();
                
            }
        else
            {   
                //hal.console->print("Read Failed");
            }
        end_t = hal.scheduler->micros() - start_t; 
       
        
        
 
        
      /*  hal.console->print("X DIST: ");
        hal.console->print(static_cast<double>(x));
        hal.console->print(" -- Y DIST: ");
        hal.console->print(static_cast<double>(y));
        
        hal.console->print(" -- END :");
        hal.console->println(end_t);*/
}

AP_HAL_MAIN();