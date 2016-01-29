package org.droidplanner.core.drone.variables;

import com.MAVLink.ardupilotmega.msg_float_sensor;

import org.droidplanner.core.drone.DroneInterfaces;
import org.droidplanner.core.drone.DroneVariable;
import org.droidplanner.core.model.Drone;


public class FloatSensor extends DroneVariable {
    private double latitude;
    private double longitude;
    private double altitudeInCm;
    private long usecSince1970;
    private float sensorValue;

    public FloatSensor(Drone drone){super(drone);}

    public void setSensor(double latitude, double longitude, double altitudeInCm, long usecSince1970, float sensorValue){
        this.latitude = latitude;
        this.longitude = longitude;
        this.altitudeInCm = altitudeInCm;
        this.usecSince1970 = usecSince1970;
        this.sensorValue = sensorValue;
    }

    public void setFloatSensor(msg_float_sensor msg){
        this.latitude = msg.latitude/10000000d;
        this.longitude = msg.longitude/10000000d;
        this.altitudeInCm = msg.altitude;
        this.usecSince1970 = msg.time_usec;
        this.sensorValue = msg.value;
        myDrone.notifyDroneEvent(DroneInterfaces.DroneEventsType.INT_SENSOR_DATA);
    }

    public double getAltitudeInCm() {
        return altitudeInCm;
    }

    public double getLatitude() {
        return latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public long getUsecSince1970() {
        return usecSince1970;
    }

    public float getSensorValue() {
        return sensorValue;
    }
}