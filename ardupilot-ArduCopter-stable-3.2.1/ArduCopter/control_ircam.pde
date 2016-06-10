/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_newflightmode.cpp - init and run calls for new flight mode
 */

// newflightmode_init - initialise flight mode
static bool ircam_init(bool ignore_checks)
{
    //if (GPS_ok() || ignore_checks) 
   // {
        // initialize vertical speed and acceleration
        pos_control.set_speed_z(-g.pilot_velocity_z_max, g.pilot_velocity_z_max);
        pos_control.set_accel_z(g.pilot_accel_z);

        // initialise altitude target to stopping point
        pos_control.set_target_to_stopping_point_z();
        
        target_ir_yaw = ahrs.yaw_sensor;
        
        land_flag = 0;

        init_ircam();
        
        return true;

}


// will be called at 100hz or more
static void ircam_run()
{
    int16_t target_roll, target_pitch;
    float target_yaw_rate = 0;
    float target_climb_rate = 0;
    int16_t pilot_throttle_scaled;  //althold/throttle

    // if not armed or throttle at zero, set throttle to zero and exit immediately
     if(!ap.auto_armed) {
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        attitude_control.set_throttle_out(0, false);
        pos_control.set_alt_target_to_current_alt();
        reset_ircam_I();
        return;
    }
    
    
     // process pilot inputs
    if (!failsafe.radio) {
        // apply SIMPLE mode transform to pilot inputs
        update_simple_mode();

        // convert pilot input to lean angles
        get_pilot_desired_lean_angles(g.rc_1.control_in, g.rc_2.control_in, target_roll, target_pitch);

        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);

        // get pilot desired climb rate
       
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in); //nie uzywamy climb rate do testow
        
     //   pilot_throttle_scaled = get_pilot_desired_throttle(g.rc_3.control_in); //throttle do testow

        // check for pilot requested take-off
     /*   if (ap.land_complete && pilot_throttle_scaled > 0) { //zmienione do testów  throttle/althold
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
        }*/
        
         if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }

    }
    
     // when landed reset targets and output zero throttle
    if (ap.land_complete) { //althold/throttle
        attitude_control.relax_bf_rate_controller();
        attitude_control.set_yaw_target_to_current_heading();
        // move throttle to between minimum and non-takeoff-throttle to keep us on the ground
        attitude_control.set_throttle_out(get_throttle_pre_takeoff(g.rc_3.control_in), false);
        pos_control.set_alt_target_to_current_alt();
        reset_ircam_I();
    }else{
        // mix in user control with IR Camera
        
        target_roll = get_ir_roll(target_roll, ir_reset);
        target_pitch = get_ir_pitch(target_pitch, ir_reset);
        target_climb_rate = get_climb_rate(target_climb_rate);
        
        ir_reset=0;
        
        target_yaw_rate = hold_ir_yaw(target_yaw_rate);
        
        /*hal.console->print("Target climb: "); (zakres -250.0 do 250.0)
        hal.console->println(target_climb_rate);*/ 
        

        //automatyczny yaw z momentu inicjalizacji
        attitude_control.angle_ef_roll_pitch_rate_ef_yaw_smooth(target_roll, target_pitch, target_yaw_rate, get_smoothing_gain());
        

        // attitude_control.set_throttle_out(pilot_throttle_scaled, true); //uzywam czystego throttle

        // update altitude target and call position controller
        pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
        pos_control.update_z_controller();
        

    }
}
    static float get_climb_rate(float input_climb_rate)
    {
     static uint32_t climb_timer;
     static uint32_t land_timer;
     static float climb_rate;
     float dist_center = (powf(ircam.get_compensated_error_x(), 2) + powf(ircam.get_compensated_error_y(),2));
     
    // hal.console->print("climb_flag: ");
    // hal.console->println(climb_flag);
    
     
     if( dist_center < 400.0 && ircam.get_lock_check()>0.0 && baro_alt > 150){ 
           if(climb_flag==0){
              climb_timer=micros();
              climb_flag = 1;}                  
             
        }else{climb_flag = 0;
              climb_rate = 0;}       
        
        
    if( dist_center < 225.0 && ircam.get_lock_check()>0.0 && baro_alt < 150){
           if(land_flag==0){              
              land_timer=micros();
              land_flag=1;}
  
        }else if(land_flag==1){
              land_flag=0;
              climb_rate=0;};
     
     if(input_climb_rate <= 25 && input_climb_rate >= -25 ){     
     
        if(climb_flag==1 && micros()-climb_timer > 100 ){
        
          climb_rate = -15.0;
          
        };
        
        if(land_flag==1 && micros()-land_timer > 2000000 ){
        
          climb_rate = -50.0;
          land_flag = 2;
        };
        
        if(land_flag==2){
        
          climb_rate = -50.0;
        }
      };
      
     /*hal.console->print("input_climb_rate: ");
     hal.console->print(input_climb_rate);
     
     hal.console->print(" baro alt: ");
     hal.console->print(baro_alt);
      
     hal.console->print(" climb_rate: ");
     hal.console->println(climb_rate);*/
     
     
    
     return (constrain_float(climb_rate+input_climb_rate, -250.0, 250.0));
    }
    
    static int32_t get_ir_roll(int32_t input_roll, uint8_t reset)
    {
        static float x_error=0;
    //  static float x_error_compensated;
        static float last_ircam_data;
        int32_t new_roll = 0;
        int32_t p,i,d;
        
        if(reset==1){
            x_error=0;
            last_ircam_data=0;
            };
                
                
           
                x_error = ircam.get_compensated_error_x();
                           
                //last_ircam_data = x_error;

                if(input_roll == 0)
                    {
                        p = g.pid_ircam_roll.get_p(x_error);
                        i = g.pid_ircam_roll.get_i(x_error,1.0f);
                        d = g.pid_ircam_roll.get_d(x_error,1.0f);
                        new_roll = p+i+d;
                       
                }else
                {
                        g.pid_ircam_roll.reset_I();
                        x_error = 0;
                        p = 0;              // for logging
                        i = 0;
                        d = 0;
                }  
                ir_roll = constrain_int32(new_roll, (ir_roll-20), (ir_roll+20));

            
        ir_roll = constrain_int32(ir_roll, -750, 750);
        

      /*  hal.console->print("x_error:");
        hal.console->print(x_error);
        
        hal.console->print("  ir_roll:");
        hal.console->println(ir_roll);*/
        
        return (input_roll+ir_roll);
    };
    
    static int32_t get_ir_pitch(int32_t input_pitch, uint8_t reset)
    {
        static float y_error=0;
      //static float y_error_compensated = 0;
        static float last_ircam_data;
        int32_t new_pitch = 0;
        int32_t p,i,d;
        
        if(reset==1){
            y_error=0;
            last_ircam_data=0;
            };
        
            y_error = ircam.get_compensated_error_y();
       
           // last_ircam_data = y_error;
        
        if( input_pitch == 0) 
        {
            p = g.pid_ircam_pitch.get_p(y_error);
            i = g.pid_ircam_pitch.get_i(y_error,1.0f);              // we could use the last update time to calculate the time change
            d = g.pid_ircam_pitch.get_d(y_error,1.0f);
            new_pitch = p+i+d;
        }else
        {
            g.pid_ircam_pitch.reset_I();
            y_error = 0;
            p = 0;              // for logging
            i = 0;
            d = 0;
        }
        ir_pitch = constrain_int32(new_pitch, (ir_pitch-20), (ir_pitch+20));
        
        
        ir_pitch = constrain_int32(ir_pitch, -750, 750);
        
    /*  hal.console->print(" --- Wysokosc: ");
        hal.console->print(ircam.get_cm_alt());
        
        hal.console->print(" --- Pitch: ");
        hal.console->print((ahrs.pitch*57.2957795));
        
        hal.console->print(" --- Error: ");
        hal.console->println(y_error);
                
        hal.console->print(" --- Kompensacja: ");
        hal.console->println(y_error_compensated);*/

        return (input_pitch+ir_pitch);
    };
    
    
    static float hold_ir_yaw(float input_yaw)
    {
        static float yaw_error;
        static float last_yaw_error;
        static float last_yaw_reading=0.0;
        static float last_target_ir_yaw=0.0;
        static float yaw_reading;
        int32_t new_yaw = 0;
        int32_t p,i,d;

        yaw_reading = ahrs.yaw_sensor;
        
        //dont check on the first run;
        if (last_target_ir_yaw==target_ir_yaw)
        {     
            //detect passing over 0 to 360;
            if ((last_yaw_reading + 35000.0) < yaw_reading)
            {
                yaw_reading = (yaw_reading - 36000.0);
            }
        
            // detect passing over 360 to 0
            if((last_yaw_reading - 35000.0) > yaw_reading)
            {
                yaw_reading = (yaw_reading + 36000.0);
            }
        };
        last_target_ir_yaw = target_ir_yaw;
        last_yaw_reading = yaw_reading;
        yaw_error = yaw_reading-target_ir_yaw; // calculate error
        
        if (last_yaw_error != yaw_error)
        {
            last_yaw_error=yaw_error;
            if (input_yaw==0)
            {
                p = g.pid_ircam_yaw.get_p(yaw_error);
                i = g.pid_ircam_yaw.get_i(yaw_error,1.0f);
                d = g.pid_ircam_yaw.get_d(yaw_error,1.0f);
                new_yaw = p+i+d;
            }else
            {
                g.pid_ircam_yaw.reset_I();
                yaw_error = 0;
                p = 0;              // for logging
                i = 0;
                d = 0;
            }
            ir_yaw = constrain_float(new_yaw, (ir_yaw-200), (ir_yaw+200));
        }
       
        
        ir_yaw = constrain_float(ir_yaw, -20000, 20000);
        
        return (-ir_yaw+input_yaw);
    
    }
    
    static void reset_ircam_I(void)
    {
        g.pid_ircam_roll.reset_I();
        g.pid_ircam_pitch.reset_I();
        g.pid_ircam_yaw.reset_I();
    }
    