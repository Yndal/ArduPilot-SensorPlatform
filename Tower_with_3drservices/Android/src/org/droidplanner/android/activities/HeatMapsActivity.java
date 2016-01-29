package org.droidplanner.android.activities;

import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Handler;
import android.os.IBinder;
import android.support.v4.app.FragmentManager;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.Circle;
import com.google.android.gms.maps.model.CircleOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.o3dr.android.client.Drone;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.mission.Mission;
import com.o3dr.services.android.lib.drone.mission.item.command.ChangeSpeed;
import com.o3dr.services.android.lib.drone.mission.item.command.ReturnToLaunch;
import com.o3dr.services.android.lib.drone.mission.item.command.Takeoff;
import com.o3dr.services.android.lib.drone.mission.item.spatial.Waypoint;
import com.o3dr.services.android.lib.drone.property.FloatSensor;
import com.o3dr.services.android.lib.drone.property.IntSensor;
import com.sothree.slidinguppanel.SlidingUpPanelLayout;

import org.droidplanner.android.R;
import org.droidplanner.android.fragments.FlightActionsFragment;
import org.droidplanner.android.tracking.sensorBased.SensorTrackingService;
import org.droidplanner.android.tracking.sensorBased.TrackerHelper;
import org.droidplanner.android.utils.prefs.DroidPlannerPrefs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Stack;
import java.util.concurrent.atomic.AtomicBoolean;

public class HeatMapsActivity extends DrawerNavigationUI {
    private enum ShowedSensor {
        IntSensor,
        FloatSensor
    }

    private ShowedSensor showedSensor;

    /**
     * Determines how long the failsafe view is visible for.
     */
    private static final long WARNING_VIEW_DISPLAY_TIMEOUT = 10000l; //ms


    private static final IntentFilter eventFilter = new IntentFilter();

    static {
        eventFilter.addAction(AttributeEvent.INT_SENSOR_UPDATED);
        eventFilter.addAction(AttributeEvent.FLOAT_SENSOR_UPDATED);
        eventFilter.addAction(AttributeEvent.AUTOPILOT_FAILSAFE);
        eventFilter.addAction(AttributeEvent.STATE_ARMING);
        eventFilter.addAction(AttributeEvent.STATE_CONNECTED);
        eventFilter.addAction(AttributeEvent.STATE_DISCONNECTED);
        eventFilter.addAction(AttributeEvent.STATE_UPDATED);
        eventFilter.addAction(AttributeEvent.FOLLOW_START);
        eventFilter.addAction(AttributeEvent.MISSION_DRONIE_CREATED);
        eventFilter.addAction(AttributeEvent.MISSION_RECEIVED);
    }

    private final BroadcastReceiver eventReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            switch (action) {
                case AttributeEvent.INT_SENSOR_UPDATED:
                    IntSensor intSensor = dpApp.getDrone().getAttribute(AttributeType.INT_SENSOR);
                    onIntSensorUpdated(intSensor);
                    //TODO Just for debugging
                    TextView textView = ((TextView) findViewById(R.id.dummy));
                    if(textView != null)
                        textView.setText(intSensor.getSensorValue() + "");

                    break;
                case AttributeEvent.FLOAT_SENSOR_UPDATED:
                    FloatSensor floatSensor = dpApp.getDrone().getAttribute(AttributeType.FLOAT_SENSOR);
                    onFloatSensorUpdated(floatSensor);
                    break;

                case AttributeEvent.AUTOPILOT_FAILSAFE:
                    String warning = intent.getStringExtra(AttributeEventExtra
                            .EXTRA_AUTOPILOT_FAILSAFE_MESSAGE);
                    final int logLevel = intent.getIntExtra(AttributeEventExtra
                            .EXTRA_AUTOPILOT_FAILSAFE_MESSAGE_LEVEL, Log.VERBOSE);
                    onWarningChanged(warning, logLevel);
                    break;

                case AttributeEvent.STATE_ARMING:
                case AttributeEvent.STATE_CONNECTED:
                case AttributeEvent.STATE_DISCONNECTED:
                case AttributeEvent.STATE_UPDATED:
                    enableSlidingUpPanel(dpApp.getDrone());
                    break;

                case AttributeEvent.FOLLOW_START:
                    //Extend the sliding drawer if collapsed.
                    if (!mSlidingPanelCollapsing.get() && mSlidingPanel.isSlidingEnabled() &&
                            !mSlidingPanel.isPanelExpanded()) {
                        mSlidingPanel.expandPanel();
                    }
                    break;

                case AttributeEvent.MISSION_DRONIE_CREATED:
    /*                float dronieBearing = intent.getFloatExtra(AttributeEventExtra.EXTRA_MISSION_DRONIE_BEARING, -1);
                    if (dronieBearing != -1)
                        updateMapBearing(dronieBearing);
    */                break;
                case AttributeEvent.MISSION_RECEIVED:
                    onMissionReceived();
                    break;
            }
        }
    };

    private final SlidingUpPanelLayout.PanelSlideListener mDisablePanelSliding = new
            SlidingUpPanelLayout.PanelSlideListener() {
                @Override
                public void onPanelSlide(View view, float v) {
                }

                @Override
                public void onPanelCollapsed(View view) {
                    mSlidingPanel.setSlidingEnabled(false);
                    mSlidingPanel.setPanelHeight(mFlightActionsView.getHeight());
                    mSlidingPanelCollapsing.set(false);

                    //Remove the panel slide listener
                    mSlidingPanel.setPanelSlideListener(null);
                }

                @Override
                public void onPanelExpanded(View view) {
                }

                @Override
                public void onPanelAnchored(View view) {
                }

                @Override
                public void onPanelHidden(View view) {
                }
            };

    private final Runnable hideWarningView = new Runnable() {
        @Override
        public void run() {
            handler.removeCallbacks(this);

            if (warningView != null && warningView.getVisibility() != View.GONE)
                warningView.setVisibility(View.GONE);
        }
    };

    private QueueWithLimit<IntSensor> sensorIntQueue;
    private QueueWithLimit<FloatSensor> sensorFloatQueue;
    private int minInt = 0;
    private int maxInt = 0;
    private double minFloat = 0;
    private double maxFloat = 0;
    private List<Circle> circles;

    private SensorTrackingService sensorTrackingService;
    private boolean mBound = false;

    private FragmentManager fragmentManager;


    private final AtomicBoolean mSlidingPanelCollapsing = new AtomicBoolean(false);

    private FlightActionsFragment flightActions;

    private final Handler handler = new Handler();
    private TextView warningView;

    private SlidingUpPanelLayout mSlidingPanel;
    private View mFlightActionsView;

    private volatile GoogleMap mMap; // Might be null if Google Play services APK is not available.
    private LinkedList<Marker> markers;
    private Polygon polygon;

    private static Thread sensorThread;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        setContentView(org.droidplanner.android.R.layout.activity_heat_maps);

        fragmentManager = getSupportFragmentManager();


        mSlidingPanel = (SlidingUpPanelLayout) findViewById(R.id.slidingPanelContainer);
        warningView = (TextView) findViewById(R.id.failsafeTextView);


        flightActions = (FlightActionsFragment) fragmentManager.findFragmentById(R.id.flightActionsFragment);
        if (flightActions == null) {
            flightActions = new FlightActionsFragment();
            fragmentManager.beginTransaction().add(R.id.flightActionsFragment, flightActions).commit();
        }


        sensorIntQueue = new QueueWithLimit<>();
        sensorFloatQueue = new QueueWithLimit<>();
        circles = new ArrayList<>();

        setUpMapIfNeeded();

        markers = new LinkedList<>();

        addClickListeners();
        if(sensorThread == null) {
            sensorThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    //ITU
                    double latitude = 55.6597021E7;
                    double longitude = 12.5918181E7;

                    double deltaLong = 0.001E7;
                    int deltaValue = 1;
                    int counter = 0;

                    while (true) {
                        IntSensor sensor = new IntSensor(latitude, longitude + counter * deltaLong, 20, 10, 10 + counter * deltaValue);
                        counter++;
                        Log.d("Thread thread thread", "Added sensor #" + counter + "(" + sensor.getLatitude() / 1E7 + "; " + sensor.getLongitude() / 1E7 + ")");
                        onIntSensorUpdated(sensor);

                        try {
                            Thread.sleep(1000);
                        } catch (InterruptedException ie) {
                            ie.printStackTrace();
                        }
                    }
                }
            });
        }
        //if(!sensorThread.isAlive())
        //    sensorThread.start();
    }

    @Override
    public void onResume() {
        super.onResume();
        setUpMapIfNeeded();
    }


    @Override
    public void onStart(){
        super.onStart();

        // Bind to LocalService
        Intent intent = new Intent(this, SensorTrackingService.class);
        bindService(intent, mConnection, Context.BIND_AUTO_CREATE);
        updateMarkerLines();
    }

    @Override
    public void onStop(){
        super.onStop();

        unbindService(mConnection);
    }


    @Override
    protected int getToolbarId() {
        return R.id.actionbar_toolbar;
    }

    @Override
    public void onApiConnected() {
        super.onApiConnected();
        enableSlidingUpPanel(dpApp.getDrone());
        getBroadcastManager().registerReceiver(eventReceiver, eventFilter);
    }

    @Override
    public void onApiDisconnected() {
        super.onApiDisconnected();
        enableSlidingUpPanel(dpApp.getDrone());
        getBroadcastManager().unregisterReceiver(eventReceiver);
    }

    @Override
    protected int getNavigationDrawerEntryId() {
        return R.id.navigation_flight_data;
    }

    public void onWarningChanged(String warning, int logLevel) {
        if (!TextUtils.isEmpty(warning)) {
            if (logLevel == Log.INFO) {
                Toast.makeText(getApplicationContext(), warning, Toast.LENGTH_SHORT).show();
            } else if (logLevel == Log.WARN || logLevel == Log.ERROR) {
                handler.removeCallbacks(hideWarningView);

                warningView.setText(warning);
                warningView.setVisibility(View.VISIBLE);
                handler.postDelayed(hideWarningView, WARNING_VIEW_DISPLAY_TIMEOUT);
            }
        }
    }

    @Override
    public void onIntSensorUpdated(final IntSensor sensor){
        //Invalid sensor location => Sensor is discarded
        if(sensor.getLatitude() == 0 && sensor.getLongitude() == 0)
            return;

        if(sensor.getSensorValue() == 2816)
            return; //Ignore: Bad I2C connection...

     /*   runOnUiThread(new Runnable() {
            @Override
            public void run() {
                Toast.makeText(getApplicationContext(), "Location (" + sensor.getLatitude() + ", " + sensor.getLongitude() + ")", Toast.LENGTH_SHORT).show();
            }
        });*/

        if(sensorIntQueue.size() == 0){
            minInt = sensor.getSensorValue();
            maxInt = sensor.getSensorValue();
        }

        sensorIntQueue.enqueue(sensor);

        //Find global min and max
        if(sensor.getSensorValue() < minInt)
            minInt = sensor.getSensorValue();
        if(maxInt < sensor.getSensorValue())
            maxInt = sensor.getSensorValue();

        updateCircles();
    }

    @Override
    public void onFloatSensorUpdated(FloatSensor sensor){
        //Invalid sensor location => Sensor is discarded
        if(sensor.getLatitude() == 0 && sensor.getLongitude() == 0)
            return;

        sensorFloatQueue.enqueue(sensor);

        if(sensorFloatQueue.size() == 0){
            minFloat = sensor.getSensorValue();
            maxFloat = sensor.getSensorValue();
        }

        //Find global min and max
        if(sensor.getSensorValue() < minFloat)
            minFloat = sensor.getSensorValue();
        if(maxFloat < sensor.getSensorValue())
            maxFloat = sensor.getSensorValue();

        if(showedSensor == ShowedSensor.FloatSensor)
            updateCircles();
    }


    private void onMissionReceived(){
        List<LatLong> coords = dpApp.getMissionProxy().getVisibleCoords();

        for(LatLong ll : coords)
            addMarker(new LatLng(ll.getLatitude(), ll.getLongitude()));
    }


    /**
     * Sets up the map if it is possible to do so (i.e., the Google Play services APK is correctly
     * installed) and the map has not already been instantiated.. This will ensure that we only ever
     * call {@link #setUpMap()} once when {@link #mMap} is not null.
     * <p/>
     * If it isn't installed {@link SupportMapFragment} (and
     * {@link com.google.android.gms.maps.MapView MapView}) will show a prompt for the user to
     * install/update the Google Play services APK on their device.
     * <p/>
     * A user can return to this FragmentActivity after following the prompt and correctly
     * installing/updating/enabling the Google Play services. Since the FragmentActivity may not
     * have been completely destroyed during this process (it is likely that it would only be
     * stopped or paused), {@link #onCreate(Bundle)} may not be called again so we should call this
     * method in {@link #onResume()} to guarantee that it will be called.
     */
    private void setUpMapIfNeeded() {
        showedSensor = ShowedSensor.IntSensor;

        // Do a null check to confirm that we have not already instantiated the map.
        if (mMap == null) {
            // Try to obtain the map from the SupportMapFragment.
            mMap = ((SupportMapFragment) getSupportFragmentManager().findFragmentById(R.id.heat_map))
                    .getMap();
            mMap.setMyLocationEnabled(true);
            // Check if we were successful in obtaining the map.
            if (mMap != null) {
                setUpMap();
            }
        }
    }

    /** Defines callbacks for service binding, passed to bindService() */
    private ServiceConnection mConnection = new ServiceConnection() {

        @Override
        public void onServiceConnected(ComponentName className,
                                       IBinder service) {
            // We've bound to SensorTrackingService, cast the IBinder and get SensorTrackingService instance
            SensorTrackingService.SensorTrackingServiceBinder binder = (SensorTrackingService.SensorTrackingServiceBinder) service;
            sensorTrackingService = binder.getService();
            mBound = true;
        }

        @Override
        public void onServiceDisconnected(ComponentName arg0) {
            mBound = false;
        }
    };

    @Override
    protected boolean enableMissionMenus() {
        return true;
    }
    /**
     * This is where we can add markers or lines, add listeners or move the camera. In this case, we
     * just add a marker near Africa.
     * <p/>
     * This should only be called once and when we are sure that {@link #mMap} is not null.
     */
    private void setUpMap() {
        //mMap.addMarker(new MarkerOptions().position(new LatLng(0, 0)).title("Marker"));
    }

    private static int undisplayedCirclesCounter = 0;
    private void updateCircles() {
        undisplayedCirclesCounter++;
        //Only update every 10th time, to avoid resource exhaustion....
        if (10 < undisplayedCirclesCounter) {
            undisplayedCirclesCounter = 0;

            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    //Clear all previous data
                    for (Circle c : circles)
                        c.remove(); //Remove from map
                    circles.clear(); //Remove from list

                    int low = Color.YELLOW;
                    int high = Color.RED;

                    int deltaR = Color.red(high) - Color.red(low);
                    int deltaG = Color.green(high) - Color.green(low);
                    int deltaB = Color.blue(high) - Color.blue(low);

                    switch (showedSensor) {
                        case IntSensor:

                            double deltaInt = maxInt - minInt;

                            QueueWithLimit<IntSensor> cloneInt = sensorIntQueue.clone();

                            while (cloneInt.hasNext()) {
                                IntSensor intSensor = cloneInt.next();
                                double weight = deltaInt == 0 ? 0 : (double) (intSensor.getSensorValue() - minInt) / deltaInt;
                                int r = Color.red(low) + (int) (deltaR * weight);
                                int g = Color.green(low) + (int) (deltaG * weight);
                                int b = Color.blue(low) + (int) (deltaB * weight);
                                int color = Color.argb(40, r, g, b);


                                Circle circleInt = mMap.addCircle(new CircleOptions()
                                        .center(new LatLng(intSensor.getLatitude()/1E7, intSensor.getLongitude()/1E7)) //TODO This a last minute hack...
                                        .radius(5)
                                        .fillColor(color)
                                        .strokeColor(color)
                                        .strokeWidth(0f));
                                circles.add(circleInt);


                            }
                            break;
                        case FloatSensor:
                            double deltaFloat = maxFloat - minFloat;

                            QueueWithLimit<FloatSensor> cloneFloat = sensorFloatQueue.clone();
                            while (cloneFloat.hasNext()) {
                                FloatSensor floatSensor = cloneFloat.next();
                                double weight = deltaFloat == 0 ? 0 : (double) (floatSensor.getSensorValue() - minInt) / deltaFloat;
                                int r = Color.red(low) + (int) (deltaR * weight);
                                int g = Color.green(low) + (int) (deltaG * weight);
                                int b = Color.blue(low) + (int) (deltaB * weight);
                                int color = Color.argb(40, r, g, b);

                                Circle circleFloat = mMap.addCircle(new CircleOptions()
                                        .center(new LatLng(floatSensor.getLatitude(), floatSensor.getLongitude()))
                                        .radius(50)
                                        .fillColor(color)
                                        .strokeColor(color));
                                circles.add(circleFloat);
                            }
                            break;
                        default:
                            //Unknown show option => don't show anything
                    }
                }
            });

        }
    }

    private void addClickListeners(){
        mMap.setOnMapLongClickListener(new GoogleMap.OnMapLongClickListener() {
            @Override
            public void onMapLongClick(LatLng latLng) {
                addMarker(latLng);
            }
        });
    }

    private Marker addMarker(LatLng latLng){
        MarkerOptions markerOpt = new MarkerOptions().
                position(latLng).
                draggable(true).
                icon(BitmapDescriptorFactory.defaultMarker(BitmapDescriptorFactory.HUE_GREEN));


        Marker marker = mMap.addMarker(markerOpt);
        markers.add(marker);

        mMap.setOnMarkerClickListener(new GoogleMap.OnMarkerClickListener() {
            @Override
            public boolean onMarkerClick(Marker marker) {
                marker.remove();
                markers.remove(marker);

                updateMarkerLines();

                return false;
            }
        });

        mMap.setOnMarkerDragListener(new GoogleMap.OnMarkerDragListener() {
            @Override
            public void onMarkerDragStart(Marker marker) {
            }

            @Override
            public void onMarkerDrag(Marker marker) {
                updateMarkerLines();
            }

            @Override
            public void onMarkerDragEnd(Marker marker) {
            }
        });
        updateMarkerLines();

        return marker;
    }

    private void updateMarkerLines(){
        List<LatLng> points = new LinkedList<>();
        for(Marker m : markers)
            points.add(m.getPosition());

        DroidPlannerPrefs dpPrefs = new DroidPlannerPrefs(this);
        double granularityInMeters = dpPrefs.getHeatMapGranularity();


        List<LatLng> route = TrackerHelper.computeRoute(this, points, granularityInMeters);
        addRouteToMissionProxy(route);

        PolygonOptions polyOpt = new PolygonOptions();
        for(LatLng ll : route)
            polyOpt.add(ll);//.strokeColor(Color.BLACK);

        if(polygon != null) {
            polygon.remove();
        }
        if (polyOpt.getPoints() != null && 0 < polyOpt.getPoints().size())
            polygon = mMap.addPolygon(polyOpt);

    }



    private void enableSlidingUpPanel(Drone api) {
        if (mSlidingPanel == null || api == null) {
            return;
        }

        final boolean isEnabled = flightActions != null && flightActions.isSlidingUpPanelEnabled
                (api);

        if (isEnabled) {
            mSlidingPanel.setSlidingEnabled(true);
        } else {
            if (!mSlidingPanelCollapsing.get()) {
                if (mSlidingPanel.isPanelExpanded()) {
                    mSlidingPanel.setPanelSlideListener(mDisablePanelSliding);
                    mSlidingPanel.collapsePanel();
                    mSlidingPanelCollapsing.set(true);
                } else {
                    mSlidingPanel.setSlidingEnabled(false);
                    mSlidingPanelCollapsing.set(false);
                }
            }
        }
    }


    private void addRouteToMissionProxy(List<LatLng> route){
        Drone drone = dpApp.getDrone();
        if(drone == null)
            return;


        Mission mission = new Mission();

        //Get preferences
        DroidPlannerPrefs dpPrefs = new DroidPlannerPrefs(this);

        //Create Take Off and set altitude
        double altitude = dpPrefs.setHeatMapAltitude();
        Takeoff takeoff = new Takeoff();
        takeoff.setTakeoffAltitude(altitude);
        mission.addMissionItem(takeoff);

        //Set speed - m/s
        double speed = dpPrefs.getHeatMapSpeedMetersPerSecond();
        ChangeSpeed changeSpeed = new ChangeSpeed();
        changeSpeed.setSpeed(speed);
        mission.addMissionItem(changeSpeed);

        //Add waypoints
        for(LatLng ll : route){
            Waypoint wp = new Waypoint();
            wp.setCoordinate(new LatLongAlt(ll.latitude, ll.longitude, altitude));
            mission.addMissionItem(wp);
        }

        //Add Return-to-launch
        ReturnToLaunch rtl = new ReturnToLaunch();
        mission.addMissionItem(rtl);

/*        Parameter updateParameter = new Parameter("WPNAV_SPEED", speedCmS, MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32);
        List<Parameter> params = new ArrayList<>(1);
        params.add(updateParameter);
        drone.writeParameters(new Parameters(params));
*/
        //Set new waypoints
        dpApp.getMissionProxy().load(mission);
    }

    public void showSensorType(ShowedSensor sensor){
        this.showedSensor = sensor;
    }
}

class QueueWithLimit<E> implements Iterator<E>, Cloneable{
    private final int LIMIT;
    private Stack<E> stack = new Stack<E>();

    public QueueWithLimit(){
        this(300);
    }

    public QueueWithLimit(int limit){
        LIMIT = limit;
    }

    public synchronized boolean enqueue(E e){
        if(stack.size() < LIMIT) {
            stack.add(e);
            return true;
        } else {
            stack.remove(stack.firstElement());
            stack.add(e);
            return false;
        }
    }

    public synchronized boolean enqueue(Collection<E> collection){
        Iterator<E> it = collection.iterator();
        boolean val = false;
        while(it.hasNext()) {
            val = enqueue(it.next());
        }

        return val;
    }

    public synchronized E dequeue(){
        return stack.remove(0);
    }

    public synchronized boolean isEmpty(){
        return stack.size() <= 0;
    }

    public synchronized int size(){
        return stack.size();
    }

    public synchronized int getLimit(){
        return LIMIT;
    }

    @Override
    public synchronized E next(){
        return dequeue();
    }

    @Override
    public synchronized boolean hasNext(){
        return 0 < stack.size();
    }

    @Override
    public synchronized void remove(){
        stack.firstElement();
    }

    public synchronized QueueWithLimit<E> clone() {
        QueueWithLimit<E> clone = new QueueWithLimit<>(LIMIT);
        for (int i = 0; i < stack.size(); i++) {
            E e = stack.get(i);

            if (e instanceof IntSensor) {
                IntSensor intSensor = (IntSensor) e;
                IntSensor intSensor1 = new IntSensor(intSensor.getLatitude(),
                        intSensor.getLongitude(),
                        intSensor.getAltitudeInCm(),
                        intSensor.getTime(),
                        intSensor.getSensorValue());

                clone.enqueue((E) intSensor1);
            } else if (e instanceof FloatSensor) {
                FloatSensor floatSensor = (FloatSensor) e;
                FloatSensor floatSensor1 = new FloatSensor(floatSensor.getLatitude(),
                        floatSensor.getLongitude(),
                        floatSensor.getAltitudeInCm(),
                        floatSensor.getTime(),
                        floatSensor.getSensorValue());

                clone.enqueue((E) floatSensor1);
            }

        }

        return clone;
    }
}

