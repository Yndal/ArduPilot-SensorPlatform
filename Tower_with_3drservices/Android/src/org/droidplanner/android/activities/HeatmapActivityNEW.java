package org.droidplanner.android.activities;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Bundle;
import android.os.Handler;
import android.support.v4.app.FragmentManager;
import android.text.TextUtils;
import android.util.Log;
import android.util.Pair;
import android.util.TypedValue;
import android.view.Gravity;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.View.OnClickListener;
import android.view.View.OnLongClickListener;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.LinearLayout;
import android.widget.TextView;
import android.widget.Toast;

import com.google.android.gms.analytics.HitBuilders;
import com.o3dr.services.android.lib.coordinate.LatLong;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra;
import com.o3dr.services.android.lib.drone.mission.MissionItemType;

import org.droidplanner.android.R;
import org.droidplanner.android.activities.interfaces.OnEditorInteraction;
import org.droidplanner.android.dialogs.EditInputDialog;
import org.droidplanner.android.dialogs.openfile.OpenFileDialog;
import org.droidplanner.android.dialogs.openfile.OpenMissionDialog;
import org.droidplanner.android.fragments.FlightActionsFragment;
import org.droidplanner.android.fragments.HeatmapFragment;
import org.droidplanner.android.fragments.HeatmapFragment.ShowedSensor;
import org.droidplanner.android.fragments.helpers.GestureMapFragment.OnPathFinishedListener;
import org.droidplanner.android.proxy.mission.MissionProxy;
import org.droidplanner.android.proxy.mission.MissionSelection;
import org.droidplanner.android.proxy.mission.item.MissionItemProxy;
import org.droidplanner.android.proxy.mission.item.fragments.MissionDetailFragment;
import org.droidplanner.android.utils.analytics.GAUtils;
import org.droidplanner.android.utils.file.FileStream;
import org.droidplanner.android.utils.file.IO.MissionReader;

import java.util.List;

/**
 * This implements the map editor activity. The map editor activity allows the
 * user to create and/or modify autonomous missions for the drone.
 */
public class HeatmapActivityNEW extends DrawerNavigationUI implements OnPathFinishedListener,
        MissionDetailFragment.OnMissionDetailListener, OnEditorInteraction, MissionSelection.OnSelectionUpdateListener, OnClickListener, OnLongClickListener {

    /**
     * Used to retrieve the item detail window when the activity is destroyed,
     * and recreated.
     */
    private static final String ITEM_DETAIL_TAG = "Item Detail Window";

    private static final String EXTRA_OPENED_MISSION_FILENAME = "extra_opened_mission_filename";

    private static final IntentFilter eventFilter = new IntentFilter();

    static {
        eventFilter.addAction(AttributeEvent.AUTOPILOT_FAILSAFE);

    }

    private final BroadcastReceiver eventReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final String action = intent.getAction();
            switch (action) {
                case AttributeEvent.AUTOPILOT_FAILSAFE:
                    String warning = intent.getStringExtra(AttributeEventExtra
                            .EXTRA_AUTOPILOT_FAILSAFE_MESSAGE);
                    final int logLevel = intent.getIntExtra(AttributeEventExtra
                            .EXTRA_AUTOPILOT_FAILSAFE_MESSAGE_LEVEL, Log.VERBOSE);
                    onWarningChanged(warning, logLevel);
                    break;
            }
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


    View.OnLayoutChangeListener removeDronieButton = new View.OnLayoutChangeListener() {
        @Override
        public void onLayoutChange(View v, int left, int top, int right, int bottom, int oldLeft, int oldTop, int oldRight, int oldBottom) {
            //TODO Shouldn't be enabled during the use of Heatmap
            Button dronieBtn = (Button) findViewById(R.id.mc_dronieBtn);
            if (dronieBtn != null && dronieBtn.getVisibility() != View.GONE){
                dronieBtn.setEnabled(false);
                dronieBtn.setVisibility(View.GONE);
                dronieBtn.setActivated(false);
                dronieBtn.getParent().requestLayout();
            }
        }
    };
    View.OnLayoutChangeListener addShowAndClearButtons = new View.OnLayoutChangeListener(){
        @Override
        public void onLayoutChange(View v, int left, int top, int right, int bottom, int oldLeft, int oldTop, int oldRight, int oldBottom) {
            LinearLayout disconnectedButtons = (LinearLayout) findViewById(R.id.mc_disconnected_buttons_layout);
            LinearLayout disarmedButtons = (LinearLayout) findViewById(R.id.mc_disarmed_buttons_layout);
            LinearLayout armedButtons = (LinearLayout) findViewById(R.id.mc_armed_buttons_layout);
            LinearLayout inFlightButtons = (LinearLayout) findViewById(R.id.mc_in_flight_buttons_layout);

            Button showSensorButton = (Button) findViewById(R.id.showedSensorButton);
            Button clearMarkersButton = (Button) findViewById(R.id.clearMarkersButton);
            if(showSensorButton != null || clearMarkersButton != null)
                return;

            if(disconnectedButtons != null){//addedButtonsToDisconnectedLayout.getAndSet(true) == false) {
                disconnectedButtons.setGravity(Gravity.CENTER);
                disconnectedButtons.addView(createShowedSensorButton());
                disconnectedButtons.addView(createClearMarkersButton());
                disconnectedButtons.requestLayout();
            }

            if(disarmedButtons != null){//addedButtonsToDisarmedLayout.getAndSet(true) == false) {
                disarmedButtons.setGravity(Gravity.CENTER);
                disarmedButtons.addView(createShowedSensorButton());
                disarmedButtons.addView(createClearMarkersButton());
                disarmedButtons.requestLayout();
            }
            if(armedButtons != null){//addedButtonsToArmedLayout.getAndSet(true) == false) {
                armedButtons.setGravity(Gravity.CENTER);
                armedButtons.addView(createShowedSensorButton());
                armedButtons.addView(createClearMarkersButton());
                armedButtons.requestLayout();
            }
            if(inFlightButtons != null){//addedButtonsToInFlightLayout.getAndSet(true) == false) {
                inFlightButtons.setGravity(Gravity.CENTER);
                inFlightButtons.addView(createShowedSensorButton());
                inFlightButtons.addView(createClearMarkersButton());
                inFlightButtons.requestLayout();
            }


        }
    };
    private Button createShowedSensorButton(){
        /*
         * Create "Show Int/Float values" button
         */
        //Create button and set style
        final Button sensorButton = new Button(getApplicationContext());
        sensorButton.setId(R.id.showedSensorButton);

        //Set drawableTop
        //sensorButton.setCompoundDrawables(null, getResources().getDrawable(R.drawable.ic_heatmap), null, null);

        //Set text
        sensorButton.setText("Show\nfloat\nvalues");

        sensorButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                ShowedSensor showedSensor = heatmapFragment.getShowedSensors();
                switch (showedSensor) {
                    case IntSensor:
                        heatmapFragment.setShowedSensors(HeatmapFragment.ShowedSensor.FloatSensor);
                        sensorButton.setText("Show\nInt\n values");
                        break;

                    case FloatSensor:
                        heatmapFragment.setShowedSensors(HeatmapFragment.ShowedSensor.IntSensor);
                        sensorButton.setText("Show\nfloat\nvalues");
                        break;
                }
            }
        });

        return sensorButton;
    }
    private Button createClearMarkersButton(){
        /*
         * Create "Clear markers" button
         */
        //Create button and set style
        final Button clearButton = new Button(getApplicationContext());
        clearButton.setId(R.id.clearMarkersButton);

        //Set drawableTop
        //sensorButton.setCompoundDrawables(null, getResources().getDrawable(R.drawable.ic_heatmap), null, null);

        //Set text
        clearButton.setText("Clear\nmarkers");
        clearButton.setGravity(Gravity.CENTER);

        clearButton.setOnClickListener(new OnClickListener() {
            @Override
            public void onClick(View v) {
                heatmapFragment.clearMarkers();
            }
        });

        return clearButton;
    }

    private final Handler handler = new Handler();


    /**
     * Used to provide access and interact with the
     * {@link org.droidplanner.android.proxy.mission.MissionProxy} object on the Android
     * layer.
     */
    private MissionProxy missionProxy;

    /*
     * View widgets.
     */
    private HeatmapFragment heatmapFragment;
    private FragmentManager fragmentManager;


    /**
     * If the mission was loaded from a file, the filename is stored here.
     */
    private String openedMissionFilename;



    private View mLocationButtonsContainer;
    private ImageButton itemDetailToggle;
    private TextView warningView;

    //private EditorListFragment editorListFragment;


    //private final AtomicBoolean mSlidingPanelCollapsing = new AtomicBoolean(false);

    private FlightActionsFragment flightActions;


    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_heatmap_new);

        fragmentManager = getSupportFragmentManager();

        heatmapFragment = ((HeatmapFragment) fragmentManager.findFragmentById(R.id.heatmapFragment));

      //  infoView = (TextView) findViewById(R.id.infoWindow);

        mLocationButtonsContainer = findViewById(R.id.location_button_container);
        final ImageButton zoomToFit = (ImageButton) findViewById(R.id.zoom_to_fit_button);
        zoomToFit.setVisibility(View.VISIBLE);
        zoomToFit.setOnClickListener(this);

        final ImageButton mGoToMyLocation = (ImageButton) findViewById(R.id.my_location_button);
        mGoToMyLocation.setOnClickListener(this);
        mGoToMyLocation.setOnLongClickListener(this);

        final ImageButton mGoToDroneLocation = (ImageButton) findViewById(R.id.drone_location_button);
        mGoToDroneLocation.setOnClickListener(this);
        mGoToDroneLocation.setOnLongClickListener(this);

        itemDetailToggle = (ImageButton) findViewById(R.id.toggle_action_drawer);
        itemDetailToggle.setOnClickListener(this);

        warningView = (TextView) findViewById(R.id.failsafeTextView);

        flightActions = (FlightActionsFragment) fragmentManager.findFragmentById(R.id.flightActionsFragment);
        if (flightActions == null) {
            flightActions = new FlightActionsFragment();
            fragmentManager.beginTransaction().add(R.id.flightActionsFragment, flightActions).commit();
        }

        if (savedInstanceState != null) {
            openedMissionFilename = savedInstanceState.getString(EXTRA_OPENED_MISSION_FILENAME);
        }

       // heatmapFragment.setOnPathFinishedListener(this);

        openActionDrawer();
    }

    @Override
    protected float getActionDrawerTopMargin() {
        return TypedValue.applyDimension(TypedValue.COMPLEX_UNIT_DIP, 24, getResources().getDisplayMetrics());
    }

    /**
     * Account for the various ui elements and update the map padding so that it
     * remains 'visible'.
     */
    private void updateLocationButtonsMargin(boolean isOpened) {
        final View actionDrawer = getActionDrawer();
        if (actionDrawer == null)
            return;

        itemDetailToggle.setActivated(isOpened);

        // Update the right margin for the my location button
        final ViewGroup.MarginLayoutParams marginLp = (ViewGroup.MarginLayoutParams) mLocationButtonsContainer
                .getLayoutParams();
        final int rightMargin = isOpened ? marginLp.leftMargin + actionDrawer.getWidth() : marginLp.leftMargin;
        marginLp.setMargins(marginLp.leftMargin, marginLp.topMargin, rightMargin, marginLp.bottomMargin);
        mLocationButtonsContainer.requestLayout();
    }

    @Override
    public void onApiConnected() {
        super.onApiConnected();


        missionProxy = dpApp.getMissionProxy();
        if (missionProxy != null) {
            missionProxy.selection.addSelectionUpdateListener(this);
            itemDetailToggle.setVisibility(missionProxy.selection.getSelected().isEmpty() ? View.GONE : View.VISIBLE);
        }

        getBroadcastManager().registerReceiver(eventReceiver, eventFilter);
    }

    @Override
    public void onApiDisconnected() {
        super.onApiDisconnected();

        if (missionProxy != null)
            missionProxy.selection.removeSelectionUpdateListener(this);

        getBroadcastManager().unregisterReceiver(eventReceiver);
    }

    @Override
    public void onClick(View v) {
        //final EditorMapFragment planningMapFragment = heatmapFragment.getMapFragment();

        switch (v.getId()) {
            case R.id.zoom_to_fit_button:
                if (heatmapFragment != null) {
                    heatmapFragment.zoomToFit();
                }
                break;
            case R.id.drone_location_button:
                heatmapFragment.goToDroneLocation();
                break;
            case R.id.my_location_button:
                heatmapFragment.goToMyLocation();
                break;
            default:
                break;
        }
    }

    @Override
    public boolean onLongClick(View view) {
        //final EditorMapFragment planningMapFragment = heatmapFragment.getMapFragment();

        switch (view.getId()) {
            case R.id.drone_location_button:

          //      planningMapFragment.setAutoPanMode(AutoPanMode.DRONE);
                return true;
            case R.id.my_location_button:
          //      planningMapFragment.setAutoPanMode(AutoPanMode.USER);
                return true;
            default:
                return false;
        }
    }

    @Override
    public void onStart(){
        super.onStart();


    }

    @Override
    public void onResume() {
        super.onResume();
        flightActions.getView().addOnLayoutChangeListener(removeDronieButton);
        flightActions.getView().addOnLayoutChangeListener(addShowAndClearButtons);

        heatmapFragment.loadCameraPosition();
        heatmapFragment.loadMarkers();
        setupTool();
    }

    @Override
    public void onPause(){
        super.onPause();
        heatmapFragment.saveCameraPosition();
        heatmapFragment.saveMarkers();
        flightActions.getView().removeOnLayoutChangeListener(removeDronieButton);
        flightActions.getView().removeOnLayoutChangeListener(addShowAndClearButtons);

    }

    @Override
    public void onStop(){
        super.onStop();

    }
    @Override
    protected int getToolbarId() {
        return R.id.actionbar_container;
    }

    @Override
    public void onSaveInstanceState(Bundle outState) {
        super.onSaveInstanceState(outState);
        outState.putString(EXTRA_OPENED_MISSION_FILENAME, openedMissionFilename);
    }

    @Override
    protected int getNavigationDrawerEntryId() {
        return R.id.navigation_heatmap;
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        super.onCreateOptionsMenu(menu);

        getMenuInflater().inflate(R.menu.menu_mission, menu);

        //TODO Currently is messes up, when downloading a mission
        final MenuItem loadMission = menu.findItem(R.id.menu_download_mission);
        loadMission.setEnabled(false);
        loadMission.setVisible(false);

        return true;
    }


    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch (item.getItemId()) {
            case R.id.menu_open_mission:
                openMissionFile();
                return true;

            case R.id.menu_save_mission:
                saveMissionFile();
                return true;

            default:
                return super.onOptionsItemSelected(item);
        }
    }

    private void openMissionFile() {
        OpenFileDialog missionDialog = new OpenMissionDialog() {
            @Override
            public void waypointFileLoaded(MissionReader reader) {
                openedMissionFilename = getSelectedFilename();
                missionProxy.readMissionFromFile(reader);
         //       heatmapFragment.getMapFragment().zoomToFit();
            }
        };
        missionDialog.openDialog(this);
    }

    private void saveMissionFile() {
        final Context context = getApplicationContext();
        final String defaultFilename = TextUtils.isEmpty(openedMissionFilename)
                ? FileStream.getWaypointFilename("waypoints")
                : openedMissionFilename;

        final EditInputDialog dialog = EditInputDialog.newInstance(context, getString(R.string.label_enter_filename),
                defaultFilename, new EditInputDialog.Listener() {
                    @Override
                    public void onOk(CharSequence input) {
                        if (missionProxy.writeMissionToFile(input.toString())) {
                            Toast.makeText(context, R.string.file_saved_success, Toast.LENGTH_SHORT)
                                    .show();

                            final HitBuilders.EventBuilder eventBuilder = new HitBuilders.EventBuilder()
                                    .setCategory(GAUtils.Category.MISSION_PLANNING)
                                    .setAction("Mission saved to file")
                                    .setLabel("Mission items count");
                            GAUtils.sendEvent(eventBuilder);

                            return;
                        }

                        Toast.makeText(context, R.string.file_saved_error, Toast.LENGTH_SHORT)
                                .show();
                    }

                    @Override
                    public void onCancel() {
                    }
                });

        dialog.show(getSupportFragmentManager(), "Mission filename");
    }

    @Override
    public void onBackPressed() {
        super.onBackPressed();
        heatmapFragment.saveCameraPosition();
    }

    @Override
    public void onMapClick(LatLong point) {
       // EditorToolsFragment.EditorToolsImpl toolImpl = getToolImpl();
       // toolImpl.onMapClick(point);
    }


    private void setupTool() {
        //final EditorToolsFragment.EditorToolsImpl toolImpl = getToolImpl();
        //toolImpl.setup();
        //editorListFragment.enableDeleteMode(toolImpl.getEditorTools() == EditorTools.TRASH);
    }


    @Override
    public void onPathFinished(List<LatLong> path) {
      //  final EditorMapFragment planningMapFragment = heatmapFragment.getMapFragment();
      //  List<LatLong> points = planningMapFragment.projectPathIntoMap(path);
       // EditorToolsFragment.EditorToolsImpl toolImpl = getToolImpl();
       // toolImpl.onPathFinished(points);
    }

    @Override
    public void onDetailDialogDismissed(List<MissionItemProxy> itemList) {
        if (missionProxy != null) missionProxy.selection.removeItemsFromSelection(itemList);
    }

    @Override
    public void onWaypointTypeChanged(MissionItemType newType, List<Pair<MissionItemProxy,
            List<MissionItemProxy>>> oldNewItemsList) {
        missionProxy.replaceAll(oldNewItemsList);
    }

    private MissionDetailFragment selectMissionDetailType(List<MissionItemProxy> proxies) {
        if (proxies == null || proxies.isEmpty())
            return null;

        MissionItemType referenceType = null;
        for (MissionItemProxy proxy : proxies) {
            final MissionItemType proxyType = proxy.getMissionItem().getType();
            if (referenceType == null) {
                referenceType = proxyType;
            } else if (referenceType != proxyType
                    || MissionDetailFragment.typeWithNoMultiEditSupport.contains(referenceType)) {
                //Return a generic mission detail.
                return new MissionDetailFragment();
            }
        }

        return MissionDetailFragment.newInstance(referenceType);
    }

    @Override
    public void onItemClick(MissionItemProxy item, boolean zoomToFit) {
    }

    @Override
    public void onListVisibilityChanged() {
    }

    @Override
    protected boolean enableMissionMenus() {
        return true;
    }

    @Override
    public void onSelectionUpdate(List<MissionItemProxy> selected) {
        final boolean isEmpty = selected.isEmpty();

        if (isEmpty) {
            itemDetailToggle.setVisibility(View.GONE);
        } else {
            itemDetailToggle.setVisibility(View.VISIBLE);
            /*if (getTool() == EditorTools.SELECTOR)
                removeItemDetail();
            else {
                showItemDetail(selectMissionDetailType(selected));
            }*/
        }

 /*       final EditorMapFragment planningMapFragment = heatmapFragment.getMapFragment();
        if (planningMapFragment != null)
            planningMapFragment.postUpdate();
   */ }

    public void onWarningChanged(String warning, int logLevel) {
        if (!TextUtils.isEmpty(warning)) {
            if (logLevel == Log.INFO) {
                Toast.makeText(getApplicationContext(), warning, Toast.LENGTH_SHORT).show();
            } else if (logLevel == Log.WARN || logLevel == Log.ERROR) {
                handler.removeCallbacks(hideWarningView);

                warningView.setText(warning);
                warningView.setVisibility(View.VISIBLE);
                handler.postDelayed(hideWarningView, FlightActivity.WARNING_VIEW_DISPLAY_TIMEOUT);
            }
        }
    }
}
