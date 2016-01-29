package com.o3dr.services.android.lib.drone.property;

import android.os.Parcel;
import android.os.Parcelable;

import java.util.Date;


public class FloatSensor implements Parcelable {
    private double latitude;
    private double longitude;
    private double altitudeInCm;
    private long usecsSince1970;
    private float sensorValue;


    public FloatSensor(){}

    public FloatSensor(double latitude, double longitude, double altitudeInCm, long usecsSince1970, float sensorValue){
        this.setLatitude(latitude);
        this.setLongitude(longitude);
        this.setAltitudeInCm(altitudeInCm);
        this.setTime(usecsSince1970);
        this.setSensorValue(sensorValue);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (!(o instanceof FloatSensor)) return false;

        FloatSensor sensor = (FloatSensor) o;

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
        temp = Float.floatToIntBits(getSensorValue());
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
        dest.writeFloat(this.sensorValue);
    }

    private FloatSensor(Parcel in){
        this.latitude = in.readDouble();
        this.longitude = in.readDouble();
        this.altitudeInCm = in.readDouble();
        this.usecsSince1970 = in.readLong();
        this.sensorValue = in.readFloat();
    }


    public static final Creator<FloatSensor> CREATOR = new Creator<FloatSensor>() {
        public FloatSensor createFromParcel(Parcel source) {
            return new FloatSensor(source);
        }

        public FloatSensor[] newArray(int size) {
            return new FloatSensor[size];
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

    public float getSensorValue() {
        return sensorValue;
    }

    public void setSensorValue(float sensorValue) {
        this.sensorValue = sensorValue;
    }
}
