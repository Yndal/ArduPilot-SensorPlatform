package org.droidplanner.android.tracking.sensorBased;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Binder;
import android.os.IBinder;

import com.o3dr.android.client.Drone;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.property.FloatSensor;
import com.o3dr.services.android.lib.drone.property.IntSensor;

import org.droidplanner.android.DroidPlannerApp;
import org.droidplanner.android.proxy.mission.MissionProxy;

import java.util.LinkedList;
import java.util.List;
import java.util.Queue;

public class SensorTrackingService extends Service {
    private enum SensorType{
        IntSensor,
        FloatSensor
    }

    private static final int MAX_SENSORS_IN_LIST = 100;
    private SensorType trackedSensor = null; //Which sensor is being tracked - null if none
    private Queue<IntSensor> intSensors = new LinkedList<>();
    private Queue<FloatSensor> floatSensors = new LinkedList<>();
    private static final IntentFilter superIntentFilter = new IntentFilter();

    static {
        superIntentFilter.addAction(AttributeEvent.INT_SENSOR_UPDATED); //TODO Sensor stuff
        superIntentFilter.addAction(AttributeEvent.FLOAT_SENSOR_UPDATED); //TODO Sensor stuff
    }

    private final BroadcastReceiver receiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            switch (action) {
                //TODO Added to support sensor stuff
                case AttributeEvent.INT_SENSOR_UPDATED:
                    onIntSensorUpdated();
                    break;
                case AttributeEvent.FLOAT_SENSOR_UPDATED:
                    onFloatSensorUpdated();
                    break;
            }
        }
    };


    private final IBinder mBinder = new SensorTrackingServiceBinder();
    private DroidPlannerApp dpApp;
    private Drone drone;
    private boolean isTrackingInt = false;
    private boolean isTrackingFloat = false;

    @Override
    public IBinder onBind(Intent intent) {
        //Toast.makeText(getApplicationContext(), "OnBind called", Toast.LENGTH_SHORT).show();
        //addFakeSquareWaypoints();

        return mBinder;
    }

    @Override
    public void onCreate(){
        super.onCreate();
        registerReceiver(receiver, superIntentFilter);
        onStart();
    }

    @Override
    public int onStartCommand(Intent intent, int flags, int startId) {
        onStart();
        return super.onStartCommand(intent, flags, startId);
    }

    private void onStart(){
        dpApp = (DroidPlannerApp) getApplication();
        drone = dpApp.getDrone();
    }

    @Override
    public void onDestroy(){
        super.onDestroy();
        unregisterReceiver(receiver);
    }

    public void startTracking(SensorType type){
        trackedSensor = type;
    }

    //TODO Remove before launch
    /*private void addFakeSquareWaypoints(){
        MissionProxy missionProxy = dpApp.getMissionProxy();

        Gps gps = drone.getAttribute(AttributeType.GPS);

        LatLong latLong = gps.getPosition();
        if(latLong == null || latLong.getLatitude() == 0.0 || latLong.getLongitude() == 0.0) {
            Toast.makeText(getApplicationContext(), "Unable to get current position - Fake square is NOT set", Toast.LENGTH_SHORT).show();
            return;
        }
        double defaultAltitude = Takeoff.DEFAULT_TAKEOFF_ALTITUDE; //10
        double squareSize = 0.0001; //Test safety distance. (~5 meters)
        LatLongAlt sw = new LatLongAlt(latLong.getLatitude()-squareSize/2, latLong.getLongitude()-squareSize/2, defaultAltitude);
        LatLongAlt nw = new LatLongAlt(latLong.getLatitude()+squareSize/2, latLong.getLongitude()-squareSize/2, defaultAltitude);
        LatLongAlt ne = new LatLongAlt(latLong.getLatitude()+squareSize/2, latLong.getLongitude()+squareSize/2, defaultAltitude);
        LatLongAlt se = new LatLongAlt(latLong.getLatitude()-squareSize/2, latLong.getLongitude()+squareSize/2, defaultAltitude);

        missionProxy.clear();
        missionProxy.addWaypoint(sw);
        missionProxy.addWaypoint(nw);
        missionProxy.addWaypoint(ne);
        missionProxy.addWaypoint(se);
        missionProxy.addWaypoint(sw); //Back again
        missionProxy.addTakeOffAndRTL();

        missionProxy.sendMissionToAPM(drone);
    }*/


    /**
     * Class used for the client Binder.  Because we know this service always
     * runs in the same process as its clients, we don't need to deal with IPC.
     */
    public class SensorTrackingServiceBinder extends Binder {
        public SensorTrackingService getService() {
            // Return this instance of LocalService so clients can call public methods
            return SensorTrackingService.this;
        }
    }

    private void onIntSensorUpdated(){
        IntSensor sensor = dpApp.getDrone().getAttribute(AttributeType.INT_SENSOR);
                     //                               sensor.getGlobalMax();
        //Only use the last MAX_SENSORS_IN_LIST
        if(MAX_SENSORS_IN_LIST < intSensors.size())
            intSensors.remove();

        intSensors.add(sensor);

        if(trackedSensor == SensorType.IntSensor)
        //TODO
        //TODO Calculate new route
        //TODO
            ;


    }

    private void onFloatSensorUpdated(){
        FloatSensor sensor = dpApp.getDrone().getAttribute(AttributeType.FLOAT_SENSOR);

        //Only use the last MAX_SENSORS_IN_LIST
        if(MAX_SENSORS_IN_LIST < floatSensors.size())
            floatSensors.remove();

       // sensor.getGlobalMin();

        floatSensors.add(sensor);

        if(trackedSensor == SensorType.FloatSensor)
            //TODO
            //TODO Calculate new route
            //TODO
            ;




    }

    private void setNewDirection(LatLong latLong){
        MissionProxy missionProxy = dpApp.getMissionProxy();

       /* double alt = missionProxy.getLastAltitude();
        LatLongAlt latLongAlt = new LatLongAlt(latLong.getLatitude(), latLong.getLongitude(), alt);
        Waypoint waypoint= createWaypoint(latLongAlt);

        MissionItemProxy newItem = new MissionItemProxy(missionProxy, waypoint);
        */
        missionProxy.clear(); //TODO This might fail because of complete empty queue when on a mission
        missionProxy.addWaypoint(latLong);
        missionProxy.sendMissionToAPM(drone);
        /*
        getAllItems
        if item.size > 1 then remove current item

       */



    }


    private void uploadWaypoints(List<LatLong> waypoints){
        dpApp.getMissionProxy().addWaypoints(waypoints);

        dpApp.getMissionProxy().sendMissionToAPM(drone);

    }
}
