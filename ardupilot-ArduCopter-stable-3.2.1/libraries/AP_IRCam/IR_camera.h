/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// IR 3df Radio camera library

#ifndef IR_camera_h
#define IR_camera_h

#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <StorageManager.h>
#include <AP_InertialSensor.h>
#include <AP_Progmem.h>
#include <AP_AHRS.h>
#include <AP_Airspeed.h>
#include <AP_Vehicle.h>

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

/*#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif*/


class IRCamera
{
    private:
    
    struct              point
    {
        uint16_t x;
        uint16_t y;
        uint8_t size;
    };
    
    struct              pointf
    {
        float x;
        float y;
        uint8_t size;
    };
    
    float                _cm_alt;
    float                _pixel_per_cm_ratio;
    float                _pixel_range;
    bool                 _read_successful;
    bool                 _camera_lock_acquired;
    bool                 _one_diode_mode;
    uint8_t              _detected_id;
    uint8_t              _number_of_blobs;
    pointf               _error_compensated;
    pointf               _landpoint_midpoint_distance;
    point                _blob_tab[4];
    pointf               _last_blob_tab[4];
    pointf               _midpoint;
    pointf               _landpoint =
                        {
                            512.0, 
                            384.0, 
                            0
                        };
                        
    
    uint8_t             Write_2bytes(uint8_t byte_1, uint8_t byte_2);
    
    bool                blob_visible(point *blob);
    
    uint8_t             count_blobs(void);
    
    bool                read_raw_data(void);
    
    bool                compute_raw_data(point *blob, uint8_t buff_start);
    
    bool                compute_blobs(void);
    
    bool                camera_lock(void);
    
    bool                check_blob_update(void);
    
    bool                calculate_midpoint(void);// do usuniecia
    
    bool                calculate_midpoint_v2(void);
    
    bool                calculate_landpoint_midpoint_distance(void);
    
    bool                calculate_cm_alt(void);
    
    bool                compensate_angle_error(void);
    
    bool                blob_print(pointf blob); //do usuniecia
    
    bool                blobs_avilable(void); // do usuniecia
    
    uint8_t             data_buf[16];
    
    AP_HAL::Semaphore*  _i2c_sem;
    
    const AP_AHRS_DCM             &_ahrs;
    AP_Baro                       &_baro;
    
    public:
    
    IRCamera(AP_AHRS_DCM &ahrs, AP_Baro &baro): _ahrs(ahrs), _baro(baro)
        {
            _camera_lock_acquired=0;
        };// constructor
        
       /* IRCamera(AP_AHRS_DCM &ahrs): _ahrs(ahrs)
        {
            _camera_lock_acquired=0;
        };// constructor*/
    
    float       get_landpoint_midpoint_distance_x(void);
    float       get_landpoint_midpoint_distance_y(void);
    float       get_compensated_error_x(void);
    float       get_compensated_error_y(void);
    float       get_cm_alt(void); //do usuniecia
    uint8_t     get_lock_check(void);
    bool        get_read_successful(void);
    bool        init(uint8_t mode);  //mode '3' (extended data) is proper for currently used compute_blobs(); 
    bool        read(void);
    void        processing_test(void);
    

    
};

#endif