package com.o3dr.services.android.lib.drone.property;

import android.os.Parcel;
import android.os.Parcelable;
import android.text.TextUtils;

/**
 * Created by fhuya on 10/28/14.
 */
public class State implements Parcelable {

    public static final int INVALID_MAVLINK_VERSION = -1;

    private boolean isConnected;
    private boolean armed;
    private boolean isFlying;
    private String calibrationStatus;
    private VehicleMode vehicleMode = VehicleMode.UNKNOWN;
    private String failsafeWarning;
    private int mavlinkVersion = INVALID_MAVLINK_VERSION;
    private long flightStartTime;

    public State(){}

    public State(boolean isConnected, VehicleMode mode, boolean armed, boolean flying, String failsafeWarning,
                 int mavlinkVersion, String calibrationStatus, long flightStartTime){
        this.isConnected = isConnected;
        this.vehicleMode = mode;
        this.armed = armed;
        this.isFlying = flying;
        this.flightStartTime = flightStartTime;
        this.failsafeWarning = failsafeWarning;
        this.mavlinkVersion = mavlinkVersion;
        this.calibrationStatus = calibrationStatus;
    }

    public boolean isConnected() {
        return isConnected;
    }

    public void setConnected(boolean isConnected) {
        this.isConnected = isConnected;
    }

    public void setArmed(boolean armed) {
        this.armed = armed;
    }

    public void setFlying(boolean isFlying) {
        this.isFlying = isFlying;
    }

    public void setCalibrationStatus(String calibrationStatus) {
        this.calibrationStatus = calibrationStatus;
    }

    public void setVehicleMode(VehicleMode vehicleMode) {
        this.vehicleMode = vehicleMode;
    }

    public void setMavlinkVersion(int mavlinkVersion) {
        this.mavlinkVersion = mavlinkVersion;
    }

    public boolean isArmed() {
        return armed;
    }

    public boolean isFlying() {
        return isFlying;
    }

    public VehicleMode getVehicleMode() {
        return vehicleMode;
    }

    public String getFailsafeWarning() {
        return failsafeWarning;
    }

    public void setFailsafeWarning(String failsafeWarning) {
        this.failsafeWarning = failsafeWarning;
    }

    public boolean isWarning(){
        return TextUtils.isEmpty(failsafeWarning);
    }

    public boolean isCalibrating(){
        return calibrationStatus != null;
    }

    public void setCalibration(String message){
        this.calibrationStatus = message;
    }

    public String getCalibrationStatus(){
        return this.calibrationStatus;
    }

    public int getMavlinkVersion() {
        return mavlinkVersion;
    }

    public long getFlightStartTime() {
        return flightStartTime;
    }

    public void setFlightStartTime(long flightStartTime) {
        this.flightStartTime = flightStartTime;
    }

    @Override
    public int describeContents() {
        return 0;
    }

    @Override
    public void writeToParcel(Parcel dest, int flags) {
        dest.writeByte(isConnected ? (byte) 1: (byte) 0);
        dest.writeByte(armed ? (byte) 1 : (byte) 0);
        dest.writeByte(isFlying ? (byte) 1 : (byte) 0);
        dest.writeString(this.calibrationStatus);
        dest.writeParcelable(this.vehicleMode, 0);
        dest.writeString(this.failsafeWarning);
        dest.writeInt(this.mavlinkVersion);
        dest.writeLong(this.flightStartTime);
    }

    private State(Parcel in) {
        this.isConnected = in.readByte() != 0;
        this.armed = in.readByte() != 0;
        this.isFlying = in.readByte() != 0;
        this.calibrationStatus = in.readString();
        this.vehicleMode = in.readParcelable(VehicleMode.class.getClassLoader());
        this.failsafeWarning = in.readString();
        this.mavlinkVersion = in.readInt();
        this.flightStartTime = in.readLong();
    }

    public static final Creator<State> CREATOR = new Creator<State>() {
        public State createFromParcel(Parcel source) {
            return new State(source);
        }

        public State[] newArray(int size) {
            return new State[size];
        }
    };
}
