/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_Math.h>
#include <AP_HAL.h>

#include "IR_camera.h"

#define IRCAM     0x58
#define DIODES_DIAGONAL_DISTANCE 5.0f
#define IR_CAMERA_CONSTANT 1338.4905875886f
#define SENSOR_ABOVE_GROUND_CM 23.0f

extern const AP_HAL::HAL& hal;

 /*        IRCamera::IRCamera(AP_AHRS_DCM &ahrs) //konstruktor
    {
        _ahrs = ahrs;
        _camera_lock_acquired=0;
    }*/

 uint8_t IRCamera::Write_2bytes(uint8_t byte_1, uint8_t byte_2) //zapis dwóch bajtów po i2c - adaptacja proceudry z ircam wiki
 {
     uint8_t stat;
     uint8_t buff[2];
     buff[0]=byte_1;
     buff[1]=byte_2;
     stat = hal.i2c->write(IRCAM, 2, buff); 
    // hal.scheduler->delay(100);
  
     return stat;
 }

 bool    IRCamera::init(uint8_t mode) // PUBLICZNE, inicjalizacja z możliwością zmiany czułości kamery, aktualnie obsługiwane tylko Extended Mode = 3
 {
    _i2c_sem = hal.i2c->get_semaphore();
    _camera_lock_acquired = 0;
    _landpoint_midpoint_distance.x = 0.0;
    _landpoint_midpoint_distance.y = 0.0;
    _midpoint.x = 0.0;
    _midpoint.y = 0.0;
    
    //_last_blob_tab[0]
    if (!_i2c_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.scheduler->panic(PSTR("Failed to get IRCAM semaphore"));
    }
     
    if(Write_2bytes(0x30,0x01)!=0){_i2c_sem->give(); return false;}; 
    if(Write_2bytes(0x30,0x08)!=0){_i2c_sem->give(); return false;};  
    if(Write_2bytes(0x06,0x90)!=0){_i2c_sem->give(); return false;};
    if(Write_2bytes(0x08,0xC0)!=0){_i2c_sem->give(); return false;};
    if(Write_2bytes(0x1A,0x40)!=0){_i2c_sem->give(); return false;}; 
    if(Write_2bytes(0x33,mode)!=0){_i2c_sem->give(); return false;};  //mode = 3;

    _i2c_sem->give();
    
    for (uint8_t i; i<=3; i++)//zerowanie tablicy dla midpoint_v2
        {
            _last_blob_tab[i].x=512.0;
            _last_blob_tab[i].y=384.0;
            _last_blob_tab[i].size=0;
        }
        
    return true;
 }
 
 bool    IRCamera::read_raw_data(void) //odczyt surowych danych z rejestrów kamery do bufora, zwraca blad przy bledzie i2c
 {
    if (!_i2c_sem->take(1)) {
       // the bus is busy - try again later
       return false;
   }
   
    uint8_t i;
    uint8_t buff[1];
    buff[0]=0x36;
    
    if(hal.i2c->write(IRCAM, 1, buff)!=0)// prośba o dane nie powiodła się
    {
        _i2c_sem->give();
        return false;
    }
     
    for (i=0;i<16;i++) { data_buf[i]=0; }
    i=0;
    
    hal.i2c->read(IRCAM,16, data_buf);
    _i2c_sem->give();
    return true;
 } 
 
 bool    IRCamera::compute_raw_data(point *blob, uint8_t buff_start) //wyciągnięcie danych z bufora dla jednego bloba
 {
    blob->x =  data_buf[buff_start];
    blob->y = data_buf[buff_start+1];
    blob->size = data_buf[buff_start+2];
    blob->x += (blob->size & 0x30) <<4;
    blob->y += (blob->size & 0xC0) <<2;
    blob->size = blob->size & 0x0F;
    
    return true;
 }

 bool    IRCamera::compute_blobs(void) //wyciągnięcie danych z bufora dla wszystkich blobów
 {
    
    for (uint8_t i=0; i<=3; i++)
    {
        compute_raw_data(&_blob_tab[i], ((i*3)+1));
        
    };
    
     return true;
 }
 
 bool    IRCamera::camera_lock(void) //wszystkie 4 bloby widoczne, komplet danych 
 {
    if(count_blobs()>=4)
    {
        _camera_lock_acquired=1;
        return true;        
    }else
    {
       // _camera_lock_acquired=1;//tymczasowo
        return false;
    }
    
 }
 
 bool    IRCamera::blob_visible(point *blob) //sprawdzenie dostępności danych jednego bloba 
 {
     if(
        blob->x==1023 && 
        blob->y==1023 && 
        blob->size==15)
            {return false;}
     else
            {return true;};        
 }
 
 uint8_t IRCamera::count_blobs(void) //zwraca ilość widzianych blobów
 {
        uint8_t number_of_blobs=0;
        
        for(uint8_t i=0; i<=3; i++){
            number_of_blobs+=blob_visible(&_blob_tab[i]);
        }
        _number_of_blobs = number_of_blobs;
        return number_of_blobs;
 }

 bool    IRCamera::blobs_avilable(void) //JAKIKOLWIEK blob jest widziany - do usunięcia
 {
    if (count_blobs()>0 && count_blobs()<5)
        {
            return true;   
        }else
        {
            return false;
        };
 } 
 
 bool    IRCamera::check_blob_update(void) //aktualizacja pozycji blobów które się zmieniły od ostatniego odczytu
 { 
   pointf copy_blob;
   
   if(camera_lock()) // zmienic na camera_lock() = po prostu jeżeli są 4 bloby
        {
            for(uint8_t i=0; i<=3; i++)
                {
                    _last_blob_tab[i].x    = static_cast<float>(_blob_tab[i].x);
                    _last_blob_tab[i].y    = static_cast<float>(_blob_tab[i].y);
                    _last_blob_tab[i].size = _blob_tab[i].size;
                    
                }
        }
        
    if(count_blobs() == 3 || count_blobs() == 2)
        {
            for(uint8_t i=0; i<=3; i++)
                {
                    if(blob_visible(&_blob_tab[i]))
                    {
                        _last_blob_tab[i].x    = static_cast<float>(_blob_tab[i].x);
                        _last_blob_tab[i].y    = static_cast<float>(_blob_tab[i].y);
                        _last_blob_tab[i].size = _blob_tab[i].size;    
                        _detected_id = i;
                    }
                }     
        }    
    if(count_blobs() == 1)
        {       
                _one_diode_mode=true;
                for(uint8_t i=0; i<=3; i++)
                {
                    if(blob_visible(&_blob_tab[i]))
                    {
                        _last_blob_tab[i].x    = static_cast<float>(_blob_tab[i].x);
                        _last_blob_tab[i].y    = static_cast<float>(_blob_tab[i].y);
                        _last_blob_tab[i].size = _blob_tab[i].size;    
                        _detected_id = i;
                    }
                }            
        }
    if(count_blobs()>1)
        {        
    for(uint8_t i=0; i<=3; i++) //znajduję widzianego bloba i przepisuje co copy_blob
                {
                    if(blob_visible(&_blob_tab[i]))
                    {
                    copy_blob.x    = static_cast<float>(_blob_tab[i].x);
                    copy_blob.y    = static_cast<float>(_blob_tab[i].y);
                    copy_blob.size = _blob_tab[i].size;
                    }
                }
                
    for(uint8_t i=0; i<=3; i++) //przepisuję copy_blob do wszystkich nie widzianych blobow
                {
                    if(!blob_visible(&_blob_tab[i]))
                    {
                    _last_blob_tab[i].x    = copy_blob.x;
                    _last_blob_tab[i].y    = copy_blob.y;
                    _last_blob_tab[i].size = copy_blob.size; 
                    }
                }
        }
    return true;
 }
 
 bool    IRCamera::calculate_midpoint(void) //stara wersja obliczania pozycji drona, działa nawet z jednym blobem; skokowe zmiany pozycji nie do użytku z filtrem kalmana
 {
    //check_blob_update(); // to jest tu chyba niepotrzebne???
    
     if(count_blobs()==0)
     {
        _pixel_per_cm_ratio = IR_CAMERA_CONSTANT* fast_atan((DIODES_DIAGONAL_DISTANCE/_cm_alt));
        return false;
     }
     
     if(count_blobs()==1)
     {
    // hal.console->println("JEDEN BLOB");
        for (uint8_t i=0; i<=3; i++)
        {
            if(blob_visible(&_blob_tab[i]))
                {
                    _midpoint.x=_blob_tab[i].x;
                    _midpoint.y=_blob_tab[i].y;
                  
                    }
            _last_blob_tab[i]=_midpoint;
        }
        _pixel_range = IR_CAMERA_CONSTANT* fast_atan((DIODES_DIAGONAL_DISTANCE/_cm_alt));
        _pixel_per_cm_ratio = (_pixel_range/DIODES_DIAGONAL_DISTANCE);  
        return true;
     }
     
     if(count_blobs()==2)
     {
    // hal.console->println("DWA BLOBY");
        IRCamera::pointf temp1={0.0,0.0,0};
        IRCamera::pointf temp2={0.0,0.0,0};
        
        for (uint8_t i=0; i<=3; i++)
        {
            if(blob_visible(&_blob_tab[i]))
                {
                    if(temp1.size==0 && temp2.size!=0)
                    {
                        temp1.x=static_cast<float>(_blob_tab[i].x);
                        temp1.y=static_cast<float>(_blob_tab[i].y);
                        temp1.size=255;        //slot taken
                    }
                    
                    if(temp1.size==0 && temp2.size==0)
                    {
                        temp2.x=static_cast<float>(_blob_tab[i].x);
                        temp2.y=static_cast<float>(_blob_tab[i].y);   
                        temp2.size=255;        // slot taken      
                    }
                };                         
        }
        _last_blob_tab[0]=temp1;
        _last_blob_tab[1]=temp1;
        _last_blob_tab[2]=temp2;
        _last_blob_tab[3]=temp2;
        _midpoint.x=((temp1.x+temp2.x)/2);
        _midpoint.y=((temp1.y+temp2.y)/2); 
        _pixel_range = IR_CAMERA_CONSTANT* fast_atan((DIODES_DIAGONAL_DISTANCE/_cm_alt));
        _pixel_per_cm_ratio = (_pixel_range/DIODES_DIAGONAL_DISTANCE);     
        return true;
     }
     
     if(count_blobs()==3)
    {
     //   hal.console->println("TRZY BLOBY");
        
        float dist[3][3];
        uint8_t blob_to_measure_1=0;
        uint8_t blob_to_measure_2=0;
        IRCamera::pointf temp_tab[3];
        
        for (uint8_t i=0; i<=2; i++) //zerowanie tymczasowej tablicy blobow
        {
                temp_tab[i].x=0.0;
                temp_tab[i].y=0.0;
                temp_tab[i].size=0;
        }
      
        for (uint8_t i=0; i<=3; i++) //wyznaczanie 3 rozpoznanych blobow z 4
        {
            if(blob_visible(&_blob_tab[i]))
                {
                    if(temp_tab[0].size==255 && temp_tab[1].size==255)
                    {
                        temp_tab[2].x=_blob_tab[i].x;
                        temp_tab[2].y=_blob_tab[i].y;
                        temp_tab[2].size=255;  //slot taken
                    }
                    
                    if(temp_tab[0].size==255 && temp_tab[1].size==0)
                    {
                        temp_tab[1].x=_blob_tab[i].x;
                        temp_tab[1].y=_blob_tab[i].y;   
                        temp_tab[1].size=255;   //slot taken                        
                    }
                    
                    if(temp_tab[0].size==0)                   
                    {
                        temp_tab[0].x=_blob_tab[i].x;
                        temp_tab[0].y=_blob_tab[i].y;   
                        temp_tab[0].size=255;    //slot taken 
                    }
                }             
        }
            
            _last_blob_tab[0]=temp_tab[0];
            _last_blob_tab[1]=temp_tab[1];
            _last_blob_tab[2]=temp_tab[2];
            _last_blob_tab[3]=temp_tab[2];

        for (uint8_t i=0; i<=2; i++) // obliczanie dystansu miedzy najodleglejszymi blobami
        {
            for(uint8_t j=0; j<=2; j++)
            {
                dist[i][j] = safe_sqrt(
                                    pow(
                                        fabs(temp_tab[i].x - temp_tab[j].x),2
                                        ) +
                                    pow(
                                        fabs(temp_tab[i].y - temp_tab[j].y),2
                                        )
                                          );
                    
                   //hal.console->print("DIST: ");
                   //hal.console->println(static_cast<double>(dist[i][j]));
                 
                    if (dist[i][j] > dist[blob_to_measure_1][blob_to_measure_2]) //wyznaczanie pary najodleglejszych blobow
                        {
                            blob_to_measure_1 = i;
                            blob_to_measure_2 = j;
                            //_pixel_range=dist[blob_to_measure_1][blob_to_measure_2];
                        }
                
            }
        }
        _midpoint.x=((temp_tab[blob_to_measure_1].x + temp_tab[blob_to_measure_2].x)/2);
        _midpoint.y=((temp_tab[blob_to_measure_1].y + temp_tab[blob_to_measure_2].y)/2); 
        
        _pixel_range = IR_CAMERA_CONSTANT* fast_atan((DIODES_DIAGONAL_DISTANCE/_cm_alt));
        _pixel_per_cm_ratio = (_pixel_range/DIODES_DIAGONAL_DISTANCE);        
        return true;
    }
     
     if(count_blobs()==4)
     {
        float dist[4][4];
        uint8_t blob_to_measure_1=0;
        uint8_t blob_to_measure_2=0;
      
        
        for (uint8_t i=0; i<=3; i++) // obliczanie dystansu miedzy najodleglejszymi blobami
        {
            for(uint8_t j=0; j<=3; j++)
            {  
                    dist[i][j] = safe_sqrt(
                                pow(
                                    fabs(_last_blob_tab[i].x - _last_blob_tab[j].x),2
                                        ) +
                                pow(
                                    fabs(_last_blob_tab[i].y - _last_blob_tab[j].y),2
                                        )
                                          );                     
                 
                    if (dist[i][j] > dist[blob_to_measure_1][blob_to_measure_2]) //wyznaczanie pary najodleglejszych blobow
                        {
                            blob_to_measure_1 = i;
                            blob_to_measure_2 = j;
                            //_pixel_range=dist[blob_to_measure_1][blob_to_measure_2];
                        }
                
            }
        }
        
        //_last_blob_tab[i] = _temp_tab[i];
        
        
        _midpoint.x=((_last_blob_tab[blob_to_measure_1].x + _last_blob_tab[blob_to_measure_2].x)/2);
        _midpoint.y=((_last_blob_tab[blob_to_measure_1].y + _last_blob_tab[blob_to_measure_2].y)/2);
        _pixel_range = IR_CAMERA_CONSTANT* fast_atan((DIODES_DIAGONAL_DISTANCE/_cm_alt));
        _pixel_per_cm_ratio = (_pixel_range/DIODES_DIAGONAL_DISTANCE);
        
        return true;
     }
     
                    
     return false; //value of blobs larger than 5
 }
 
 bool    IRCamera::calculate_midpoint_v2(void) //nowa wersja obliczania pozycji drona, zapamiętuje poprzednią pozycję
 {
    check_blob_update();
    
    float dist[4][4];
    uint8_t blob_to_measure_1=0;
    uint8_t blob_to_measure_2=0;
    //IRCamera::pointf temp_tab[4];
        
    /*for (uint8_t i=0; i<=3; i++)
        {
            temp_tab[i].x=_blob_tab[i].x;
            temp_tab[i].y=_blob_tab[i].y;
        }*/
        
    for (uint8_t i=0; i<=3; i++) // obliczanie dystansu miedzy najodleglejszymi blobami
        {
            for(uint8_t j=0; j<=3; j++)
            {  
                dist[i][j] = safe_sqrt(
                                pow(
                                    fabs(_last_blob_tab[i].x - _last_blob_tab[j].x),2
                                        ) +
                                pow(
                                    fabs(_last_blob_tab[i].y - _last_blob_tab[j].y),2
                                        )
                                          );                   
                 
                    if (dist[i][j] > dist[blob_to_measure_1][blob_to_measure_2]) //wyznaczanie pary najodleglejszych blobow
                        {
                            blob_to_measure_1 = i;
                            blob_to_measure_2 = j;
                            _pixel_range=dist[blob_to_measure_1][blob_to_measure_2];
                        }
                
            }
        }
                
         
                /*_pixel_range=safe_sqrt(
                                pow(
                                    fabs(_last_blob_tab[blob_to_measure_1].x - _last_blob_tab[blob_to_measure_2].x),2
                                        )+
                                pow(
                                    fabs(_last_blob_tab[blob_to_measure_1].y - _last_blob_tab[blob_to_measure_2].y),2
                                        )
                                          );*/
        
        if(_one_diode_mode==true)
        {      
            _midpoint=_last_blob_tab[_detected_id];
            _one_diode_mode=false;
        };
        
        _midpoint.x=((_last_blob_tab[blob_to_measure_1].x + _last_blob_tab[blob_to_measure_2].x)/2);
        _midpoint.y=((_last_blob_tab[blob_to_measure_1].y + _last_blob_tab[blob_to_measure_2].y)/2);
        
        return true;
    
 }
 
 bool    IRCamera::calculate_landpoint_midpoint_distance(void) //obliczanie błędu pozycji, odlegość między pożadanym punktem, a aktualnym
 {
    
    _landpoint_midpoint_distance.x = (((_midpoint.x - _landpoint.x)+1)/_pixel_per_cm_ratio);
    
    _landpoint_midpoint_distance.y = (((_midpoint.y - _landpoint.y)+1)/_pixel_per_cm_ratio);
    
    //_landpoint_midpoint_distance.x = ((_midpoint.x - _landpoint.x)+1)/5.12;
    
    //_landpoint_midpoint_distance.y = ((_midpoint.y - _landpoint.y)+1)/5.12;
    
    return true;
 }
 
 bool    IRCamera::calculate_cm_alt(void)
 {
 
            _baro.read();
            
            _cm_alt=_inertial_nav.get_altitude();
            
            if (_cm_alt<0.0){_cm_alt=1.0;}; 
            
            _pixel_range = IR_CAMERA_CONSTANT * fast_atan((DIODES_DIAGONAL_DISTANCE/_cm_alt));
        
            _pixel_per_cm_ratio = (_pixel_range/DIODES_DIAGONAL_DISTANCE);
            
            return true;
 }
 float   IRCamera::get_landpoint_midpoint_distance_x(void) //PUBLICZNE zwraca error w osi x
 {
     
    return _landpoint_midpoint_distance.x;
 }

 float   IRCamera::get_landpoint_midpoint_distance_y(void) //PUBLICZNE zwraca error w osi y
 {
       
    return _landpoint_midpoint_distance.y;
 }
 
  float IRCamera::get_compensated_error_x(void)
 {    
     return _error_compensated.x;
 }
 
  float IRCamera::get_compensated_error_y(void)
 {
     return _error_compensated.y;
 }
 
 float   IRCamera::get_cm_alt(void)
 {

     return _cm_alt; //altitude in cm
 }
 
 uint8_t IRCamera::get_lock_check(void) //zwraca camera lock - czy zobaczyl 4 diody
 {

    return _number_of_blobs;
      
 }
 
 bool IRCamera::compensate_angle_error(void)
 {
     if (count_blobs()>0){
     _error_compensated.x = (-_landpoint_midpoint_distance.x -((tanf(_ahrs.roll))*_cm_alt));
     _error_compensated.y = (_landpoint_midpoint_distance.y -((tanf(_ahrs.pitch))*_cm_alt));
    };
    
     //_error_compensated.x = -_landpoint_midpoint_distance.x;
     //_error_compensated.y = _landpoint_midpoint_distance.y;
     
     return true;
 }
 
 bool    IRCamera::get_read_successful(void)
 {
     if(_read_successful)
        {return 1;}
     else
        {return 0;};
 }
 
 bool    IRCamera::read(void) //PUBLICZNE wykonuje wszystkie obliczenia, aktualizuje error x oraz y
 {
     
     
    //_camera_lock_acquired=1; //for testing
    if(!read_raw_data())
    {   
        _read_successful=false;
        return false;
    }        
    compute_blobs(); //nie przewiduje błędu
    check_blob_update(); //albo tutaj jest niepotrzebne albo w metodzie calculate_midpoint()
    calculate_cm_alt();
    
    if(1)                 //Raz znalezione 4 bloby - mozna zaczynac - trzeba resetowac po wyjsciu z trybu!!!
    { //_camera_lock_acquired==1
        _read_successful=true;
        calculate_midpoint_v2();
        //calculate_midpoint();
        calculate_landpoint_midpoint_distance();
       // calculate_cm_alt();
        compensate_angle_error();

    
        return true;
    }else 
    {
        _read_successful=false;
        return true;
    }
 
 }  
 
 
 bool    IRCamera::blob_print(pointf blob) //do testów
 {
     
     if (static_cast<int>(blob.x) < 1000)
         hal.console->print("");
      if (static_cast<int>(blob.x) < 100)  
         hal.console->print("");
      if (static_cast<int>(blob.x) < 10)  
         hal.console->print("");
       hal.console->print(static_cast<int>(blob.x));
       hal.console->print(",");
      if (static_cast<int>(blob.y) < 1000)
         hal.console->print("");
      if (static_cast<int>(blob.y) < 100)  
         hal.console->print("");
      if (static_cast<int>(blob.y) < 10)  
         hal.console->print("");
       hal.console->print(static_cast<int>(blob.y));
        hal.console->print(",");
       hal.console->print(blob.size);

  /*  hal.console->print("---");
    hal.console->print(static_cast<double>(blob.x));
    hal.console->print(",");
    hal.console->print(static_cast<double>(blob.y));
    hal.console->print("-(");
    hal.console->print(blob.size);
    hal.console->print(")---");    */
    
    return true;
 }
 
 void   IRCamera::processing_test(void)
 {
    for (int i=0; i<4; i++)
        {       
          
                blob_print(_last_blob_tab[i]);
                if (i<3){ hal.console->print(",");};
                
        }
        hal.console->print(",");
        hal.console->print(static_cast<int>(_midpoint.x)); //_midpoint.x
        hal.console->print(",");
        hal.console->print(static_cast<int>(_midpoint.y)); //_midpoint.y
        hal.console->print(",");
        hal.console->print(static_cast<int>(_cm_alt));
        hal.console->print(",");
        hal.console->print(static_cast<int>(_error_compensated.x));
        hal.console->print(",");
        hal.console->print(static_cast<int>(_error_compensated.y));
        hal.console->print(",");
        hal.console->print(static_cast<int>(_ahrs.roll*1000));
        hal.console->print(",");
        hal.console->print(static_cast<int>(_ahrs.pitch*1000));
        //hal.console->print(",PIXEL PER CM RATIO:");
       // hal.console->print(_pixel_per_cm_ratio);
        hal.console->println(""); 
 };