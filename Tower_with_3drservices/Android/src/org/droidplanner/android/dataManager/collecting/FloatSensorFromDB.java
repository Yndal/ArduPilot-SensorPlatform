package org.droidplanner.android.dataManager.collecting;

import com.o3dr.services.android.lib.drone.property.FloatSensor;

/**
 * Created by lynd on 17/03/15.
 */
public class FloatSensorFromDB extends FloatSensor {
    private final long dbId;

    public FloatSensorFromDB(long id, double latitude, double longitude, double altitude, long time, float sensorValue){//, String sensorType, long time, double altitude, double longitude, double latitude, int sensorValue){
        super(latitude, longitude, altitude, time, sensorValue);
        this.dbId = id;
    }

    public long getId(){
        return dbId;
    }
}
