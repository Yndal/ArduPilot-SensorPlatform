package com.o3dr.services.android.lib.drone.property;

import android.os.Parcel;
import android.os.Parcelable;

import java.util.Date;

/**************************************************************
 *
 * This class is the one, that the client will have access to.
 *
 *************************************************************/
public class IntSensor implements Parcelable {
    private double latitude;
    private double longitude;
    private double altitudeInCm;
    private long usecsSince1970;
    private int sensorValue;
    private static int globalMin = 0;
    private static int globalMax = 0;


    public IntSensor(){}

    public IntSensor(double latitude, double longitude, double altitudeInCm, long usecsSince1970, int sensorValue){
        this.setLatitude(latitude);
        this.setLongitude(longitude);
        this.setAltitudeInCm(altitudeInCm);
        this.setTime(usecsSince1970);
        this.setSensorValue(sensorValue);
        updateGlobalMinMax();
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof IntSensor)) return false;

        IntSensor sensor = (IntSensor) o;

        if(Double.compare(sensor.altitudeInCm, getAltitudeInCm()) != 0) return false;
        if(Double.compare(sensor.latitude, getLatitude()) != 0) return false;
        if(Double.compare(sensor.longitude, getLongitude()) != 0) return false;
        if(sensor.usecsSince1970 != getTime()) return false;
     //   if(sensor.sensorType != null || !sensor.sensorType.equals(getSensorType())) return false;
        if(sensor.sensorValue != getSensorValue()) return false;

        return true;
    }

    @Override
    public int hashCode() {
        int result;
        long temp;
        temp = Double.doubleToLongBits(getAltitudeInCm());
        result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(getLatitude());
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(getLongitude());
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = getSensorValue();
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        temp = usecsSince1970;
        result = 31 * result + (int) (temp ^ (temp >>> 32));

        return result;
    }

    @Override
    public String toString() {
        return "Sensor{" +
                "latitude=" + getLatitude() +
                ", longitude=" + getLongitude() +
                ", altitudeInCm=" + getAltitudeInCm() +
                ", usec=" + getTime() +
                ", sensorValue=" + getSensorValue() +
                '}';
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeDouble(this.latitude);
        dest.writeDouble(this.longitude);
        dest.writeDouble(this.altitudeInCm);
        dest.writeLong(this.usecsSince1970);
        dest.writeInt(this.sensorValue);
    }

    private IntSensor(Parcel in){
        this.latitude = in.readDouble();
        this.longitude = in.readDouble();
        this.altitudeInCm = in.readDouble();
        this.usecsSince1970 = in.readLong();
        this.sensorValue = in.readInt();
        updateGlobalMinMax();
    }

    private void updateGlobalMinMax(){
        if(sensorValue < globalMin)
            globalMin = sensorValue;
        else if(globalMax < sensorValue)
            globalMax = sensorValue;
    }


    public static final Parcelable.Creator<IntSensor> CREATOR = new Parcelable.Creator<IntSensor>() {
        public IntSensor createFromParcel(Parcel source) {
            return new IntSensor(source);
        }

        public IntSensor[] newArray(int size) {
            return new IntSensor[size];
        }
    };

    public double getAltitudeInCm() {
        return altitudeInCm;
    }

    public void setAltitudeInCm(double altitudeInCm) {
        this.altitudeInCm = altitudeInCm;
    }

    public double getLatitude() {
        return latitude;
    }

    public void setLatitude(double latitude) {
        this.latitude = latitude;
    }

    public double getLongitude() {
        return longitude;
    }

    public void setLongitude(double longitude) {
        this.longitude = longitude;
    }

    public long getTime() {
        return usecsSince1970;
    }

    public void setTime(long usecsSince1970) {
        this.usecsSince1970 = usecsSince1970;
    }

    public int getSensorValue() {
        return sensorValue;
    }

    public void setSensorValue(int sensorValue) {
        updateGlobalMinMax();
        this.sensorValue = sensorValue;
    }

    public int getGlobalMin(){
        return globalMin;
    }

    public int getGlobalMax(){
        return globalMax;
    }
}
