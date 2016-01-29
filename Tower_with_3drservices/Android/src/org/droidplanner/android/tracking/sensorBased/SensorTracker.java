package org.droidplanner.android.tracking.sensorBased;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;

import com.o3dr.android.client.Drone;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.property.FloatSensor;
import com.o3dr.services.android.lib.drone.property.IntSensor;

import org.droidplanner.android.utils.Utils;

import java.util.Queue;
import java.util.concurrent.ConcurrentLinkedQueue;

/**
 * Created by lynd on 05/04/15.
 */
public class SensorTracker/* extends Service*/ /*implements DroidPlannerApp.ApiListener,*/ /*DroneListener */{
    public static final double DELTA_FOR_REACTION = 5.00; //TODO Change


    private final Queue<IntSensor> intReadings;
    private final Queue<FloatSensor> floatReadings;
    private final Drone drone;

    private static final IntentFilter superIntentFilter = new IntentFilter();

    static {
        superIntentFilter.addAction(AttributeEvent.INT_SENSOR_UPDATED); //TODO Sensor stuff
        superIntentFilter.addAction(AttributeEvent.FLOAT_SENSOR_UPDATED); //TODO Sensor stuff
        superIntentFilter.addAction(AttributeEvent.STATE_CONNECTED);
        superIntentFilter.addAction(AttributeEvent.STATE_DISCONNECTED);
        superIntentFilter.addAction(Utils.ACTION_UPDATE_OPTIONS_MENU);
    }

    private final BroadcastReceiver superReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            switch (action) {
                //TODO Added to support sensor stuff
                case AttributeEvent.INT_SENSOR_UPDATED:
                    //onFloatSensor();
                    break;
                case AttributeEvent.FLOAT_SENSOR_UPDATED:
                    //onIntSensor();
                    break;
                case AttributeEvent.STATE_CONNECTED:
                    //onDroneConnected();
                    break;

                case AttributeEvent.STATE_DISCONNECTED:
                    //onDroneDisconnected();
                    break;

                case Utils.ACTION_UPDATE_OPTIONS_MENU:
                    //invalidateOptionsMenu();
                    break;
            }
        }
    };



    public SensorTracker(Drone drone, String eventType){
        this.drone = drone;
        this.intReadings = new ConcurrentLinkedQueue<IntSensor>();
        this.floatReadings = new ConcurrentLinkedQueue<FloatSensor>();
     //   drone.registerDroneListener(this);


    }


   // @Override
    public void onDroneConnectionFailed(com.o3dr.services.android.lib.drone.connection.ConnectionResult connectionResult){

    }

 /*   @Override
    public void onDroneEvent(java.lang.String s, android.os.Bundle bundle) {
        final String action = intent.getAction();
        switch (action) {
            //TODO Added to support sensor stuff
            case AttributeEvent.SENSOR_UPDATED:
                onSensor();

        }
    }

    @Override
    public void onDroneServiceInterrupted(java.lang.String s){

    }



    @Override
    public void onApiConnected() {

        getBroadcastManager().registerReceiver(superReceiver, superIntentFilter);
        if (dpApp.getDrone().isConnected())
            onDroneConnected();
        else
            onDroneDisconnected();

        lbm.sendBroadcast(new Intent(MissionProxy.ACTION_MISSION_PROXY_UPDATE));
    }

    @Override
    public void onApiDisconnected() {
        getBroadcastManager().unregisterReceiver(superReceiver);
        onDroneDisconnected();
    }
*/


}
