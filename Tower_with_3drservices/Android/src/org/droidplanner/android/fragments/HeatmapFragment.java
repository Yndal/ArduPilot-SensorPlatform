package org.droidplanner.android.fragments;

import android.app.Activity;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.content.SharedPreferences;
import android.graphics.Bitmap;
import android.graphics.Color;
import android.location.Location;
import android.os.Bundle;
import android.os.IBinder;
import android.support.v4.content.LocalBroadcastManager;
import android.util.Log;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.maps.CameraUpdate;
import com.google.android.gms.maps.CameraUpdateFactory;
import com.google.android.gms.maps.GoogleMap;
import com.google.android.gms.maps.OnMapReadyCallback;
import com.google.android.gms.maps.SupportMapFragment;
import com.google.android.gms.maps.model.BitmapDescriptorFactory;
import com.google.android.gms.maps.model.CameraPosition;
import com.google.android.gms.maps.model.Circle;
import com.google.android.gms.maps.model.CircleOptions;
import com.google.android.gms.maps.model.LatLng;
import com.google.android.gms.maps.model.LatLngBounds;
import com.google.android.gms.maps.model.Marker;
import com.google.android.gms.maps.model.MarkerOptions;
import com.google.android.gms.maps.model.Polygon;
import com.google.android.gms.maps.model.PolygonOptions;
import com.o3dr.android.client.Drone;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.coordinate.LatLongAlt;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.mission.Mission;
import com.o3dr.services.android.lib.drone.mission.item.command.ChangeSpeed;
import com.o3dr.services.android.lib.drone.mission.item.command.ReturnToLaunch;
import com.o3dr.services.android.lib.drone.mission.item.command.Takeoff;
import com.o3dr.services.android.lib.drone.mission.item.spatial.Waypoint;
import com.o3dr.services.android.lib.drone.property.FloatSensor;
import com.o3dr.services.android.lib.drone.property.Gps;
import com.o3dr.services.android.lib.drone.property.IntSensor;
import com.o3dr.services.android.lib.drone.property.State;

import org.droidplanner.android.DroidPlannerApp;
import org.droidplanner.android.R;
import org.droidplanner.android.graphic.map.GraphicDrone;
import org.droidplanner.android.maps.DPMap;
import org.droidplanner.android.tracking.sensorBased.SensorTrackingService;
import org.droidplanner.android.tracking.sensorBased.TrackerHelper;
import org.droidplanner.android.utils.DroneHelper;
import org.droidplanner.android.utils.prefs.DroidPlannerPrefs;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Date;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Stack;

public class HeatmapFragment extends SupportMapFragment  {
    private static final String PREF_MARKERS = "pref_coords";

    /**
     * The map should zoom on the user location the first time it's acquired. This flag helps
     * enable the behavior.
     */
    private static boolean didZoomOnUserLocation = false;


    private static final IntentFilter eventFilter = new IntentFilter(AttributeEvent.STATE_ARMING);
    static {
        eventFilter.addAction(AttributeEvent.INT_SENSOR_UPDATED);
        eventFilter.addAction(AttributeEvent.FLOAT_SENSOR_UPDATED);
        eventFilter.addAction(AttributeEvent.HEARTBEAT_FIRST);
        eventFilter.addAction(AttributeEvent.HEARTBEAT_RESTORED);
        eventFilter.addAction(AttributeEvent.HEARTBEAT_TIMEOUT);
        eventFilter.addAction(AttributeEvent.STATE_CONNECTED);
        eventFilter.addAction(AttributeEvent.STATE_DISCONNECTED);
        eventFilter.addAction(AttributeEvent.GPS_POSITION);
        eventFilter.addAction(AttributeEvent.MISSION_RECEIVED);
    }

    private final BroadcastReceiver eventReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            switch(action) {
                case AttributeEvent.INT_SENSOR_UPDATED:
                    IntSensor intSensor = dpApp.getDrone().getAttribute(AttributeType.INT_SENSOR);
                    onIntSensorUpdated(intSensor);
                    break;

                case AttributeEvent.FLOAT_SENSOR_UPDATED:
                    FloatSensor floatSensor = dpApp.getDrone().getAttribute(AttributeType.FLOAT_SENSOR);
                    onFloatSensorUpdated(floatSensor);
                    break;

                case AttributeEvent.HEARTBEAT_FIRST:
                case AttributeEvent.HEARTBEAT_RESTORED:
                case AttributeEvent.STATE_CONNECTED:
                case AttributeEvent.STATE_DISCONNECTED:
                case AttributeEvent.GPS_POSITION:
                case AttributeEvent.HEARTBEAT_TIMEOUT:
                    updateDroneIcon();
                    break;

                case AttributeEvent.MISSION_RECEIVED:
                    onMissionReceived();
                    break;
            }
            if (AttributeEvent.STATE_ARMING.equals(action)) {
                final State droneState = dpApp.getDrone().getAttribute(AttributeType.STATE);
                if (droneState.isArmed()) {
                    mMap.clear();
                }

            }
        }
    };


    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        dpApp = (DroidPlannerApp) activity.getApplication();
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup viewGroup, Bundle bundle) {
        View view = super.onCreateView(inflater, viewGroup, bundle);

        mAppPrefs = new DroidPlannerPrefs(getActivity());
        initHeatmapStuff();

        return view;
    }

    @Override
    public void onViewCreated(View view, Bundle savedInstanceState){
        super.onViewCreated(view, savedInstanceState);
    }

    @Override
    public void onDestroyView() {
        super.onDestroyView();
    }

    @Override
    public void onResume() {
        super.onResume();

     /*
        if (!didZoomOnUserLocation) {
            super.goToMyLocation();
            didZoomOnUserLocation = true;
        }*/
        LocalBroadcastManager.getInstance(getActivity()).registerReceiver(eventReceiver, eventFilter);

        setUpMapIfNeeded();
    }

    @Override
    public void onStart(){
        super.onStart();

        // Bind to LocalService
        Intent intent = new Intent(getActivity(), SensorTrackingService.class);
        getActivity().bindService(intent, mConnection, Context.BIND_AUTO_CREATE);


        //Tell the user about the click possibilities
        //TODO informUserOfClickOption();

        updateMarkerLines();
        updateDroneIcon();
    }


    private void informUserOfClickOption(){
        Toast.makeText(getActivity(), "Path options:\n" +
                "Long click: Add point\n" +
                "Short click on point: Remove\n" +
                "Long click on point: Drag", Toast.LENGTH_LONG).show();
        Toast.makeText(getActivity(), "Altitude, speed, sensor settings and data extraction can be set in Settings -> Sensor platform", Toast.LENGTH_LONG).show();
        Toast.makeText(getActivity(), "When path is done, remember to upload to drone.", Toast.LENGTH_LONG).show();
    }

    @Override
    public void onStop(){
        super.onStop();

        saveCameraPosition();
        getActivity().unbindService(mConnection);
        dpApp.getMissionProxy().clear(); //TODO Very important: This is done to prevent app from crashing when going to Editor and mission proxy is not empty. Crash appears when EditorMapFragment:149 public void zoomToFit(List<LatLong> itemsToFit) is called

    }

    @Override
    public void onPause() {
        super.onPause();
        LocalBroadcastManager.getInstance(getActivity()).unregisterReceiver(eventReceiver);
    }

    public void saveCameraPosition() {
        CameraPosition camera = getMap().getCameraPosition();
        mAppPrefs.prefs.edit()
                .putFloat(DPMap.PREF_LAT, (float) camera.target.latitude)
                .putFloat(DPMap.PREF_LNG, (float) camera.target.longitude)
                .putFloat(DPMap.PREF_BEA, camera.bearing)
                .putFloat(DPMap.PREF_TILT, camera.tilt)
                .putFloat(DPMap.PREF_ZOOM, camera.zoom).apply();
    }

    public void loadCameraPosition() {
        final SharedPreferences settings = mAppPrefs.prefs;

        CameraPosition.Builder camera = new CameraPosition.Builder();
        camera.bearing(settings.getFloat(DPMap.PREF_BEA, DPMap.DEFAULT_BEARING));
        camera.tilt(settings.getFloat(DPMap.PREF_TILT, DPMap.DEFAULT_TILT));
        camera.zoom(settings.getFloat(DPMap.PREF_ZOOM, DPMap.DEFAULT_ZOOM_LEVEL));
        camera.target(new LatLng(settings.getFloat(DPMap.PREF_LAT, DPMap.DEFAULT_LATITUDE),
                settings.getFloat(DPMap.PREF_LNG, DPMap.DEFAULT_LONGITUDE)));

        getMap().moveCamera(CameraUpdateFactory.newCameraPosition(camera.build()));
    }

    public void saveMarkers(){
       /* Gson gson = new Gson();
        String markersStr = gson.toJson(markers); //TODO This takes waaayyyy too long time

        mAppPrefs.prefs.edit().putString(PREF_MARKERS, markersStr).apply();*/
    }

    public void loadMarkers(){
        /*final SharedPreferences settings = mAppPrefs.prefs;

        Gson gson = new Gson();
        List<LatLng> gsonMarkers = gson.fromJson(settings.getString(PREF_MARKERS,""), new TypeToken<ArrayList<LatLng>>(){}.getType());
        if(gsonMarkers != null && !gsonMarkers.isEmpty()) {
            markers.clear();
            //addPolygonToMap(gsonMarkers);
            for (LatLng latLng : gsonMarkers)
                addMarker(latLng);
        }*/
    }


    /**********************
     * Stuff below this point has been added
     *********************/

    public enum ShowedSensor {
        IntSensor,
        FloatSensor
    }

    private ShowedSensor showedSensor;

    private DroidPlannerApp dpApp;
    private Marker droneMarker;
    private DroidPlannerPrefs mAppPrefs;

    private QueueWithLimit<IntSensor> sensorIntQueue;
    private QueueWithLimit<FloatSensor> sensorFloatQueue;
    private List<LatLng> coords;
    private int minInt = 0;
    private int maxInt = 0;
    private double minFloat = 0;
    private double maxFloat = 0;
    private List<Circle> circles;

    private SensorTrackingService sensorTrackingService;
    private boolean mBound = false;

    private volatile GoogleMap mMap; // Might be null if Google Play services APK is not available.
    private LinkedList<Marker> markers;
    private Polygon polygon;

    private static Thread sensorThread;


    private void initHeatmapStuff(){
        sensorIntQueue = new QueueWithLimit<>(300);
        sensorFloatQueue = new QueueWithLimit<>(300);
        coords = new ArrayList<>();
        circles = new ArrayList<>();

        setUpMapIfNeeded();

        markers = new LinkedList<>();

        addClickListeners();
        if(sensorThread == null) {
            sensorThread = new Thread(new Runnable() {
                @Override
                public void run() {
                    //ITU (55.659459, 12.591262)
                    //Koege (55.480338, 12.178821)
                    double latitude = 55.480338;//E7;
                    double longitude = 12.178821;//E7;



                    double deltaLong = 0.0001;//0.001E7;
                    int deltaValue = 1;
                    int counter = 0;

                    while (true) {
                        IntSensor sensor = new IntSensor(latitude, longitude + counter * deltaLong, 20, 10, 10 + counter * deltaValue);
                        counter++;
                        Log.d("Thread thread thread", "Added sensor #" + counter + "(" + sensor.getLatitude() /*/ 1E7*/ + "; " + sensor.getLongitude() /*/ 1E7*/ + ")");
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
       // if(!sensorThread.isAlive())
         //   sensorThread.start();
    }

    public ShowedSensor getShowedSensors(){
        return showedSensor;
    }

    public void setShowedSensors(ShowedSensor sensor){
        showedSensor = sensor;
        undisplayedCirclesCounter = -2; //Will clear all current circles at the map, when updateCircles() is called :)
        updateCircles();
    }

    public void onIntSensorUpdated(final IntSensor sensor){
        //Toast.makeText(getActivity(), "Date: " + new Date(sensor.getTime()/1000).toString() + ", Value: " + sensor.getSensorValue() + "\nLat: " + sensor.getLatitude() + ", Long: " + sensor.getLongitude(), Toast.LENGTH_SHORT).show();

        //Got measurement, but with no valid time (which includes the location since it is GPS based) => Discard this
        if(sensor.getTime() == 0) //sensor.getLatitude() == 0 && sensor.getLongitude() == 0)
            return;

       /* if(sensor.getSensorValue() == 2816)
            return; //TODO Ignore: Bad I2C connection (Fix this!!)
*/
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

        if(showedSensor == ShowedSensor.IntSensor) {
            updateCircles();
            showSensorValue(sensor.getSensorValue());
        }

    }

    public void onFloatSensorUpdated(FloatSensor sensor){
        //Got measurement, but with no valid location => Discard this
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

        if(showedSensor == ShowedSensor.FloatSensor) {
            updateCircles();
            showSensorValue(sensor.getSensorValue());
        }
    }

    public void clearMarkers(){
        for (Marker m : markers)
            m.remove();
        markers.clear();

        if(polygon != null)
            polygon.remove();
    }

    private void showSensorValue(final int value){
        showSensorValue(Integer.toString(value));
    }

    private void showSensorValue(final double value){
        showSensorValue(Double.toString(value));
    }

    private void showSensorValue(final String value){
        getActivity().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                TextView textView = ((TextView) getActivity().findViewById(R.id.dummy));
                textView.setText("Sensor: " + value);
            }
        });
    }

    private void onMissionReceived(){
        List<LatLong> coords = dpApp.getMissionProxy().getVisibleCoords();
        if(coords.isEmpty())
            return;
        mMap.clear();

        List<LatLng> polyPoints = new ArrayList<>();
        for(LatLong ll : coords)
            polyPoints.add(new LatLng(ll.getLatitude(), ll.getLongitude()));

        addPolygonToMap(polyPoints);

        /*PolygonOptions polygonOptions = new PolygonOptions();
        for(LatLong ll : coords) //TODO Somethings wrong here: Should only be added as a polyline and not all as markers!!!
            polygonOptions.add(new LatLng(ll.getLatitude(), ll.getLongitude()));
*/
    }


    private void updateDroneIcon(){
        GraphicDrone graphicDrone = new GraphicDrone(dpApp.getDrone());
        LatLong position = graphicDrone.getPosition();

        if(position == null) //Will be null if isn't connected to drone and reaching this point
            return;

        LatLng latLng = new LatLng(position.getLatitude(), position.getLongitude());
        final MarkerOptions markerOptions = new MarkerOptions()
                .position(latLng)
                .draggable(false)
                .alpha(graphicDrone.getAlpha())
                .anchor(graphicDrone.getAnchorU(), graphicDrone.getAnchorV())
                .infoWindowAnchor(graphicDrone.getInfoWindowAnchorU(),
                        graphicDrone.getInfoWindowAnchorV()).rotation(graphicDrone.getRotation())
                .snippet(graphicDrone.getSnippet()).title(graphicDrone.getTitle())
                .flat(graphicDrone.isFlat()).visible(graphicDrone.isVisible());

        final Bitmap markerIcon = graphicDrone.getIcon(getResources());
        if (markerIcon != null) {
            markerOptions.icon(BitmapDescriptorFactory.fromBitmap(markerIcon));
        }

        if(droneMarker != null)
            droneMarker.remove();
        droneMarker = mMap.addMarker(markerOptions);
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
        setShowedSensors(ShowedSensor.IntSensor);

        // Do a null check to confirm that we have not already instantiated the map.
        if (mMap == null) {
            // Try to obtain the map from the SupportMapFragment.

            mMap = ((SupportMapFragment) getFragmentManager().findFragmentById(R.id.heatmapFragment)).getMap();
            // Check if we were successful in obtaining the map.
            if (mMap != null) {
                setUpMap();
            }
        }
    }

    private void setUpMap() {
        mMap.setMyLocationEnabled(true);
        mMap.setMapType(GoogleMap.MAP_TYPE_HYBRID);
        //mMap.addMarker(new MarkerOptions().position(new LatLng(0, 0)).title("Marker"));
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

    private static int undisplayedCirclesCounter = 0;
    private void updateCircles() {
        undisplayedCirclesCounter++;
        //Only update every 10th time, to avoid resource exhaustion....
        if (10 < undisplayedCirclesCounter || undisplayedCirclesCounter < 0) {
            undisplayedCirclesCounter = 0;

            getActivity().runOnUiThread(new Runnable() {
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

                    int alpha = 130;

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
                                int color = Color.argb(alpha, r, g, b);


                                Circle circleInt = mMap.addCircle(new CircleOptions()
                                        .center(new LatLng(intSensor.getLatitude(), intSensor.getLongitude()))
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
                                int color = Color.argb(alpha, r, g, b);

                                Circle circleFloat = mMap.addCircle(new CircleOptions()
                                        .center(new LatLng(floatSensor.getLatitude(), floatSensor.getLongitude()))
                                        .radius(5)
                                        .fillColor(color)
                                        .strokeColor(color)
                                .strokeWidth(0f));
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
                if (markers.isEmpty())
                    dpApp.getMissionProxy().load(new Mission());//.clear();
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

        DroidPlannerPrefs dpPrefs = new DroidPlannerPrefs(getActivity());
        double granularityInMeters = dpPrefs.getHeatMapGranularity();


        List<LatLng> route = TrackerHelper.computeRoute(getActivity(), points, granularityInMeters);
        addRouteToMissionProxy(route);

        addPolygonToMap(route);

    }

    private void addPolygonToMap(List<LatLng> points){
        PolygonOptions polyOpt = new PolygonOptions();
        for(LatLng ll : points)
            polyOpt.add(ll);//.strokeColor(Color.BLACK);

        if(polygon != null) {
            polygon.remove();
        }
        if (polyOpt.getPoints() != null && 0 < polyOpt.getPoints().size())
            polygon = mMap.addPolygon(polyOpt);
    }

    private void addRouteToMissionProxy(List<LatLng> route){
        if(route.isEmpty())
            return;

        Drone drone = dpApp.getDrone();
        if(drone == null)
            return;


        Mission mission = new Mission();

        //Get preferences
        DroidPlannerPrefs dpPrefs = new DroidPlannerPrefs(getActivity());

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

        //Return to launch
        ReturnToLaunch rtl = new ReturnToLaunch();
        mission.addMissionItem(rtl);
/*        //Return to first waypoint again
        LatLng firstLl = route.get(0);
        LatLongAlt firstLla = new LatLongAlt(firstLl.latitude, firstLl.longitude, altitude);
        Waypoint wp = new Waypoint();
        wp.setCoordinate(firstLla);
        mission.addMissionItem(wp);

        //Add Landing
        Land land = new Land();
        land.setCoordinate(firstLla);
        mission.addMissionItem(land);
*/
        //Set mission
        dpApp.getMissionProxy().load(mission);
    }

    public void zoomToFit(){
        List<LatLong> coords = dpApp.getMissionProxy().getVisibleCoords();

        if(!coords.isEmpty()){
            if (!coords.isEmpty()) {
                final List<LatLng> points = new ArrayList<>();
                for (LatLong coord : coords)
                    points.add(DroneHelper.CoordToLatLang(coord));

                final LatLngBounds bounds = getBounds(points);
                getMapAsync(new OnMapReadyCallback() {
                    @Override
                    public void onMapReady(GoogleMap googleMap) {
                        CameraUpdate animation = CameraUpdateFactory.newLatLngBounds(bounds, 100);
                        googleMap.animateCamera(animation);
                    }
                });
            }
        }
    }

    private LatLngBounds getBounds(List<LatLng> pointsList) {
        LatLngBounds.Builder builder = new LatLngBounds.Builder();
        for (LatLng point : pointsList) {
            builder.include(point);
        }
        return builder.build();
    }

    public void goToDroneLocation(){
        Drone dpApi = dpApp.getDrone();
        if (!dpApi.isConnected())
            return;

        Gps gps = dpApi.getAttribute(AttributeType.GPS);
        if (!gps.isValid()) {
            Toast.makeText(getActivity().getApplicationContext(), R.string.drone_no_location, Toast.LENGTH_SHORT).show();
            return;
        }

        final float currentZoomLevel = getMap().getCameraPosition().zoom;
        final LatLong droneLocation = gps.getPosition();
        updateCamera(droneLocation, (int) currentZoomLevel);
    }

    public void goToMyLocation(){
        final Location myLocation =  mMap.getMyLocation();//LocationServices.FusedLocationApi.getLastLocation(getGoogleApiClient());
        if (myLocation != null) {
            updateCamera(DroneHelper.LocationToCoord(myLocation), 19f);

          /*  if (mLocationListener != null)
                mLocationListener.onLocationChanged(myLocation);*/
        }

    }

    public void updateCamera(final LatLong coord, final float zoomLevel) {
        if (coord != null) {
            getMapAsync(new OnMapReadyCallback() {
                @Override
                public void onMapReady(GoogleMap googleMap) {
                    googleMap.animateCamera(CameraUpdateFactory.newLatLngZoom(
                            DroneHelper.CoordToLatLang(coord), zoomLevel));
                }
            });
        }
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