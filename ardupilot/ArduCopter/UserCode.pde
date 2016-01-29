/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#ifdef USERHOOK_INIT
void userhook_init()
{
    // put your initialisation code here
    // this will be called once at start-up
    teensy.init();
}
#endif

#ifdef USERHOOK_FASTLOOP
void userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void userhook_50Hz()
{
    // put your 50Hz code here
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void userhook_MediumLoop()
{
    // put your 10Hz code here
}
#endif

#ifdef USERHOOK_SLOWLOOP
void userhook_SlowLoop()
{
    // put your 3.3Hz code here
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
uint8_t sensorCounter = 0;
bool measuring = false;
void userhook_SuperSlowLoop()
{
    // put your 1Hz code here
    sensorCounter++;

    int16_t interval = g.teensy_sen_f; // frequency
    int16_t timeToMeasure = g.teensy_sen_t; // time needed to perform measure
    
    hal.console->printf("Time: %d/%d.\n", sensorCounter, interval);
    
    //Start measuring so it is available when needed
    if(!measuring && interval - timeToMeasure <= sensorCounter){
        measuring = true;
        hal.console->println("Requesting measurement...");
        teensy.measure();
    }

    if(interval <= sensorCounter){
        
        int16_t value = 0;
        const Location &loc = gps.location();

        //hal.console->printf("Lat: %d, Lon: %d\n", loc.lat, loc.lng);
        uint64_t usec = gps.time_epoch_usec();
        //uint32_t fix = gps.last_fix_time_ms();
        
        if (teensy.read(&value)) {
            //Reset counting variables, etc.
            measuring = false;
            sensorCounter = 0;    
            
            mavlink_msg_int_sensor_send(MAVLINK_COMM_1, value, usec, loc.alt, loc.lng, loc.lat);
            hal.console->printf("Value: %d, interval: %d. ", value, interval);
            hal.console->println("Mavlink message (200) sent");
            
        } else {
            hal.console->printf("No or old sensor data...\n");
        }
    }
}
#endif