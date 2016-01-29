package org.droidplanner.android.dataManager.collecting;

import com.o3dr.services.android.lib.drone.property.IntSensor;

/**
 * Created by lynd on 17/03/15.
 */
public class IntSensorFromDB extends IntSensor {
    private final long dbId;

    public IntSensorFromDB(long id, double latitude, double longitude, double altitude, long time, int sensorValue){
        super(latitude, longitude, altitude, time, sensorValue);
        this.dbId = id;
    }

    public long getId(){
        return dbId;
    }
}
