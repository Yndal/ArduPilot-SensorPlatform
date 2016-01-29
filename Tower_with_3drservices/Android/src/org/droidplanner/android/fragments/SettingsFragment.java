package org.droidplanner.android.fragments;

import android.app.Activity;
import android.app.AlertDialog;
import android.app.ProgressDialog;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.SharedPreferences;
import android.content.SharedPreferences.OnSharedPreferenceChangeListener;
import android.content.pm.PackageManager.NameNotFoundException;
import android.os.AsyncTask;
import android.os.Bundle;
import android.os.Handler;
import android.preference.CheckBoxPreference;
import android.preference.EditTextPreference;
import android.preference.ListPreference;
import android.preference.Preference;
import android.preference.PreferenceCategory;
import android.preference.PreferenceFragment;
import android.support.v4.content.LocalBroadcastManager;
import android.text.TextUtils;
import android.util.Log;
import android.widget.Toast;

import com.MAVLink.enums.MAV_PARAM_TYPE;
import com.google.android.gms.analytics.GoogleAnalytics;
import com.google.android.gms.analytics.HitBuilders;
import com.o3dr.android.client.Drone;
import com.o3dr.services.android.lib.drone.attribute.AttributeEvent;
import com.o3dr.services.android.lib.drone.attribute.AttributeEventExtra;
import com.o3dr.services.android.lib.drone.attribute.AttributeType;
import com.o3dr.services.android.lib.drone.connection.ConnectionType;
import com.o3dr.services.android.lib.drone.property.Parameter;
import com.o3dr.services.android.lib.drone.property.Parameters;
import com.o3dr.services.android.lib.drone.property.State;
import com.o3dr.services.android.lib.drone.property.Type;

import org.droidplanner.android.DroidPlannerApp;
import org.droidplanner.android.R;
import org.droidplanner.android.activities.helpers.MapPreferencesActivity;
import org.droidplanner.android.dataManager.collecting.DBAdapter;
import org.droidplanner.android.dialogs.ClearBTDialogPreference;
import org.droidplanner.android.maps.providers.DPMapProvider;
import org.droidplanner.android.utils.Utils;
import org.droidplanner.android.utils.analytics.GAUtils;
import org.droidplanner.android.utils.file.DirectoryPath;
import org.droidplanner.android.utils.prefs.DroidPlannerPrefs;

import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Locale;
import java.util.Properties;

import javax.mail.Message;
import javax.mail.MessagingException;
import javax.mail.PasswordAuthentication;
import javax.mail.Session;
import javax.mail.Transport;
import javax.mail.internet.AddressException;
import javax.mail.internet.InternetAddress;
import javax.mail.internet.MimeMessage;

/**
 * Implements the application settings screen.
 */
public class SettingsFragment extends PreferenceFragment implements OnSharedPreferenceChangeListener,
        DroidPlannerApp.ApiListener {

    /**
     * Used as tag for logging.
     */
    private final static String TAG = SettingsFragment.class.getSimpleName();

    private static final String PACKAGE_NAME = SettingsFragment.class.getPackage().getName();

    /**
     * Action used to broadcast updates to the period for the spoken status
     * summary.
     */
    public static final String ACTION_UPDATED_STATUS_PERIOD = PACKAGE_NAME + ".ACTION_UPDATED_STATUS_PERIOD";

    /**
     * Used to retrieve the new period for the spoken status summary.
     */
    public static final String EXTRA_UPDATED_STATUS_PERIOD = "extra_updated_status_period";

    private static final IntentFilter intentFilter = new IntentFilter();

    static {
        intentFilter.addAction(AttributeEvent.STATE_DISCONNECTED);
        intentFilter.addAction(AttributeEvent.STATE_UPDATED);
        intentFilter.addAction(AttributeEvent.HEARTBEAT_FIRST);
        intentFilter.addAction(AttributeEvent.HEARTBEAT_RESTORED);
        intentFilter.addAction(AttributeEvent.TYPE_UPDATED);
    }

    private final BroadcastReceiver broadcastReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context context, Intent intent) {
            final Activity activity = getActivity();
            if (activity == null)
                return;

            final String action = intent.getAction();
            switch (action) {
                case AttributeEvent.STATE_DISCONNECTED:
                    updateMavlinkVersionPreference(null);
                    updateFirmwareVersionPreference(null);
                    break;

                case AttributeEvent.HEARTBEAT_FIRST:
                case AttributeEvent.HEARTBEAT_RESTORED:
                    int mavlinkVersion = intent.getIntExtra(AttributeEventExtra.EXTRA_MAVLINK_VERSION, -1);
                    if (mavlinkVersion == -1)
                        updateMavlinkVersionPreference(null);
                    else
                        updateMavlinkVersionPreference(String.valueOf(mavlinkVersion));
                    break;

                case AttributeEvent.TYPE_UPDATED:
                    Drone drone = dpApp.getDrone();
                    if (drone.isConnected()) {
                        Type droneType = drone.getAttribute(AttributeType.TYPE);
                        updateFirmwareVersionPreference(droneType.getFirmwareVersion());
                    } else
                        updateFirmwareVersionPreference(null);
                    break;
            }
        }
    };

    /**
     * Keep track of which preferences' summary need to be updated.
     */
    private final HashSet<String> mDefaultSummaryPrefs = new HashSet<String>();

    private final Handler mHandler = new Handler();

    private DroidPlannerApp dpApp;
    private DroidPlannerPrefs dpPrefs;
    private LocalBroadcastManager lbm;
    private int lastSensorMeasuringTime;
    private int lastSensorUpdateInterval;

    @Override
    public void onAttach(Activity activity) {
        super.onAttach(activity);
        dpApp = (DroidPlannerApp) activity.getApplication();
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        addPreferencesFromResource(R.xml.preferences);

        initSummaryPerPrefs();

        final Context context = getActivity().getApplicationContext();
        dpPrefs = new DroidPlannerPrefs(context);
        lbm = LocalBroadcastManager.getInstance(context);
        final SharedPreferences sharedPref = dpPrefs.prefs;

        setupPeriodicControls();

        // Populate the map preference category
        final String mapsProvidersPrefKey = getString(R.string.pref_maps_providers_key);
        final ListPreference mapsProvidersPref = (ListPreference) findPreference(mapsProvidersPrefKey);
        if (mapsProvidersPref != null) {
            final DPMapProvider[] providers = DPMapProvider.values();
            final int providersCount = providers.length;

            final CharSequence[] providersNames = new CharSequence[providersCount];
            final CharSequence[] providersNamesValues = new CharSequence[providersCount];
            for (int i = 0; i < providersCount; i++) {
                final String providerName = providers[i].name();
                providersNamesValues[i] = providerName;
                providersNames[i] = providerName.toLowerCase(Locale.ENGLISH).replace('_', ' ');
            }

            final String defaultProviderName = sharedPref.getString(mapsProvidersPrefKey,
                    DPMapProvider.DEFAULT_MAP_PROVIDER.name());

            mapsProvidersPref.setEntries(providersNames);
            mapsProvidersPref.setEntryValues(providersNamesValues);
            mapsProvidersPref.setValue(defaultProviderName);
            mapsProvidersPref
                    .setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {

                        @Override
                        public boolean onPreferenceChange(Preference preference, Object newValue) {
                            // Update the map provider settings preference.
                            final String mapProviderName = newValue.toString();
                            return updateMapSettingsPreference(mapProviderName);
                        }
                    });

            updateMapSettingsPreference(defaultProviderName);
        }

        // update the summary for the preferences in the mDefaultSummaryPrefs hash table.
        for (String prefKey : mDefaultSummaryPrefs) {
            final Preference pref = findPreference(prefKey);
            if (pref != null) {
                pref.setSummary(sharedPref.getString(prefKey, ""));
            }
        }

        // Set the usage statistics preference
        final String usageStatKey = getString(R.string.pref_usage_statistics_key);
        final CheckBoxPreference usageStatPref = (CheckBoxPreference) findPreference(usageStatKey);
        if (usageStatPref != null) {
            usageStatPref
                    .setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                        @Override
                        public boolean onPreferenceChange(Preference preference, Object newValue) {
                            // Update the google analytics singleton.
                            final boolean optIn = (Boolean) newValue;
                            final GoogleAnalytics analytics = GoogleAnalytics.getInstance(context);
                            analytics.setAppOptOut(!optIn);
                            return true;
                        }
                    });
        }

        final Preference storagePref = findPreference(getString(R.string.pref_storage_key));
        if (storagePref != null) {
            storagePref.setSummary(DirectoryPath.getPublicDataPath());
        }

        try {
            Preference versionPref = findPreference("pref_version");
            if (versionPref != null) {
                String version = context.getPackageManager().getPackageInfo(
                        context.getPackageName(), 0).versionName;
                versionPref.setSummary(version);
            }
        } catch (NameNotFoundException e) {
            Log.e(TAG, "Unable to retrieve version name.", e);
        }

        updateMavlinkVersionPreference(null);
        setupConnectionPreferences();
        setupAdvancedMenuToggle();
        setupUnitSystemPreferences();
        setupBluetoothDevicePreferences();
        setupImminentGroundCollisionWarningPreference();

        //TODO Sensor stuff settings
        setupSensorUpdateInterval();
        setupSensorMeasuringTime();
        setupHeatMapAltitude();
        setupHeatMapSpeed();
        setupHeatMapGranularity();

        lastSensorMeasuringTime = dpPrefs.getSensorMeasuringTime();
        lastSensorUpdateInterval = dpPrefs.getSensorUpdateInterval();

        //final EditTextPreference exportMailPreference = (EditTextPreference) findPreference(getString(R.string.pref_export_mail));
        //exportMailPreference.setSummary(((EditTextPreference) findPreference(getString(R.string.pref_export_mail))).getText());

        final Preference exportPreference = (Preference) findPreference(getString(R.string.pref_export_csv));
        exportPreference.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
            @Override
            public boolean onPreferenceClick(Preference preference) {
                //Collect data from local DB
                DBAdapter db = new DBAdapter(getActivity());
                String intData = db.getAllIntSensors_CSV();
                String floatData = db.getAllFloatSensors_CSV();
                db.close();

                //Get receiver mail
                String receiver = ((EditTextPreference) findPreference(getString(R.string.pref_export_mail))).getText();

                //Valid mail address?
                if(!validEmailAddress(receiver)) {
                    Toast.makeText(getActivity(), "Invalid mail address. Unable to send!", Toast.LENGTH_LONG).show();
                    return false;
                }

                //Send email
                sendMail(receiver, "oWasp: Int data", intData);
                sendMail(receiver, "oWasp: Float data", floatData);

                return false;
            }
        });

        final Preference clearDataPreference = findPreference(getString(R.string.pref_clear_data));
        clearDataPreference.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
            @Override
            public boolean onPreferenceClick(Preference preference) {
                AlertDialog.Builder builder = new AlertDialog.Builder(getActivity());
                builder.setMessage("This will delete all previously collected data. Do you really want to delete this?")
                        .setPositiveButton("Delete", new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                DBAdapter db = new DBAdapter(getActivity());
                                db.clearAllTables();
                            }
                        })
                        .setNegativeButton("No", new DialogInterface.OnClickListener() {
                            public void onClick(DialogInterface dialog, int id) {
                                // User cancelled the dialog
                            }
                        });
                // Create the AlertDialog object and show it
                builder.create().show();

                return false;
            }
        });


    }

    private boolean validEmailAddress(String mail){
        return mail != null && android.util.Patterns.EMAIL_ADDRESS.matcher(mail).matches();
    }

    private void setupAdvancedMenuToggle() {
        CheckBoxPreference togglePref = (CheckBoxPreference) findPreference(getString(R.string
                .pref_advanced_menu_toggle_key));
        if (togglePref != null) {
            togglePref.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    lbm.sendBroadcast(new Intent(Utils.ACTION_UPDATE_OPTIONS_MENU));
                    return true;
                }
            });
        }
    }

    //TODO Sensor stuff settings
    private void setupSensorUpdateInterval() {
        ListPreference sensorUpdateIntervalPref = (ListPreference) findPreference(getString(R.string.sensor_update_interval_key));
        if (sensorUpdateIntervalPref != null) {
            int intervalS = dpPrefs.getSensorUpdateInterval();
            updateSensorUpdateInterval(sensorUpdateIntervalPref, intervalS);
            sensorUpdateIntervalPref.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    int intervalS = Integer.parseInt((String) newValue);
                    return updateSensorUpdateInterval(preference, intervalS);
                    //return true;
                }
            });
        }
    }
    private void setupSensorMeasuringTime() {
        ListPreference sensorMeasuringTimePref = (ListPreference) findPreference(getString(R.string.sensor_measuring_time_key));
        if (sensorMeasuringTimePref != null) {
            int time = dpPrefs.getSensorMeasuringTime();
            updateSensorMeasuringTime(sensorMeasuringTimePref, time);
            sensorMeasuringTimePref.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    int time = Integer.parseInt((String) newValue);
                    return updateSensorMeasuringTime(preference, time);
                    //return true;
                }
            });
        }
    }

    private void setupHeatMapAltitude() {
        ListPreference altitudePref = (ListPreference) findPreference(getString(R.string.pref_sensor_platform_heat_map_altitude_key));
        if (altitudePref != null) {
            double altitude = dpPrefs.setHeatMapAltitude();
            updateHeatMapAltitude(altitudePref, altitude);
            altitudePref.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    double altitude = Double.parseDouble((String) newValue);
                    updateHeatMapAltitude(preference, altitude);
                    return true;
                }
            });
        }
    }

    private void setupHeatMapSpeed() {
        ListPreference speedPref = (ListPreference) findPreference(getString(R.string.pref_sensor_platform_heat_map_speed_key));
        if (speedPref != null) {
            double speedCmS = dpPrefs.getHeatMapSpeedMetersPerSecond();
            updateHeatMapSpeed(speedPref, speedCmS);
            speedPref.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    double speed = Double.parseDouble((String) newValue);
                    updateHeatMapAltitude(preference, speed);
                    return true;
                }
            });
        }
    }

    private void setupHeatMapGranularity() {
        ListPreference granPrefs = (ListPreference) findPreference(getString(R.string.pref_sensor_platform_heat_map_granularity_key));
        if (granPrefs != null) {
            int granularity = dpPrefs.getHeatMapGranularity();
            updateHeatMapGranularity(granPrefs, granularity);
            granPrefs.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    int granularity = Integer.parseInt((String) newValue);
                    updateHeatMapGranularity(preference, granularity);
                    return true;
                }
            });
        }
    }

    private void setupUnitSystemPreferences() {
        ListPreference unitSystemPref = (ListPreference) findPreference(getString(R.string.pref_unit_system_key));
        if (unitSystemPref != null) {
            int defaultUnitSystem = dpPrefs.getUnitSystemType();
            updateUnitSystemSummary(unitSystemPref, defaultUnitSystem);
            unitSystemPref.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    int unitSystem = Integer.parseInt((String) newValue);
                    updateUnitSystemSummary(preference, unitSystem);
                    return true;
                }
            });
        }
    }

    private void setupImminentGroundCollisionWarningPreference() {
        final CheckBoxPreference collisionWarn = (CheckBoxPreference) findPreference(getString(R.string
                .pref_ground_collision_warning_key));
        if (collisionWarn != null) {
            collisionWarn.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                @Override
                public boolean onPreferenceChange(Preference preference, Object newValue) {
                    final boolean isEnabled = (Boolean) newValue;
                    if (!isEnabled) {
                        lbm.sendBroadcast(new Intent(Drone.ACTION_GROUND_COLLISION_IMMINENT)
                                .putExtra(Drone.EXTRA_IS_GROUND_COLLISION_IMMINENT, false));
                    }
                    return true;
                }
            });
        }
    }

    private void updateUnitSystemSummary(Preference preference, int unitSystemType) {
        final int summaryResId;
        switch (unitSystemType) {
            case 0:
            default:
                summaryResId = R.string.unit_system_entry_auto;
                break;

            case 1:
                summaryResId = R.string.unit_system_entry_metric;
                break;

            case 2:
                summaryResId = R.string.unit_system_entry_imperial;
                break;
        }

        preference.setSummary(summaryResId);
    }

    //TODO Sensor stuff
    private boolean updateSensorUpdateInterval(Preference preference, int inter) {
        final Drone drone = dpApp.getDrone();
        if (!drone.isConnected()) {
            preference.setSummary("--");
            return false;
        }

        int meas = dpPrefs.getSensorMeasuringTime();

        if (inter < meas) {
            Toast.makeText(getActivity(), "Measuring interval can't be less than estimated measuring time.", Toast.LENGTH_LONG).show();
//            Toast.makeText(getActivity(), "Measuring time(" + meas + " s) can't be greater than measuring interval (" + inter + " s). Interval time is set to " + meas + ".", Toast.LENGTH_LONG).show();

            return false;
        }

        dpPrefs.setSensorUpdateInterval(inter);
        lastSensorUpdateInterval = inter;

        Parameter updateParameter = new Parameter("TEENSY_SEN_F", inter, MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
        List<Parameter> params = new ArrayList<>(1);
        params.add(updateParameter);
        drone.writeParameters(new Parameters(params));

        int index = ((ListPreference) preference).findIndexOfValue(String.valueOf(inter));
        preference.setSummary(((ListPreference) preference).getEntries()[index]);

        //preference.setSummary("Every " + inter + " second(s)");

        return true;
    }

    private boolean updateSensorMeasuringTime(Preference preference, int meas) {
        final Drone drone = dpApp.getDrone();
        if (!drone.isConnected()) {
            preference.setSummary("--");
            return false;
        }

        int inter = dpPrefs.getSensorUpdateInterval();
        if (inter < meas) {
            Toast.makeText(getActivity(), "Measuring interval can't be less than estimated measuring time.", Toast.LENGTH_LONG).show();

            return false;
        }

        dpPrefs.setSensorMeasuringTime(meas);
        lastSensorMeasuringTime = meas; //TODO Considering the line just above - is this one really needed

        Parameter updateParameter = new Parameter("TEENSY_SEN_T", meas, MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT16);
        List<Parameter> params = new ArrayList<>(1);
        params.add(updateParameter);
        drone.writeParameters(new Parameters(params));

        int index = ((ListPreference) preference).findIndexOfValue(String.valueOf(meas));
        preference.setSummary(((ListPreference) preference).getEntries()[index]);

        //preference.setSummary(meas + " second(s)");

        return true;
    }

    private void updateHeatMapAltitude(Preference preference, double altitude) {
        dpPrefs.setHeatMapAltitude(altitude);
        preference.setSummary(altitude + " meters");
    }

    private void updateHeatMapGranularity(Preference preference, int granularity) {
        dpPrefs.setHeatMapGranularity(granularity);
        preference.setSummary(granularity + " meters");
    }

    private void updateHeatMapSpeed(Preference preference, double speedMS) {
        dpPrefs.setHeatMapSpeedMetersPerSecond(speedMS);
        preference.setSummary(speedMS + " m/s");
    }


    private void setupConnectionPreferences() {
        ListPreference connectionTypePref = (ListPreference) findPreference(getString(R.string.pref_connection_type_key));
        if (connectionTypePref != null) {
            int defaultConnectionType = dpPrefs.getConnectionParameterType();
            updateConnectionPreferenceSummary(connectionTypePref, defaultConnectionType);
            connectionTypePref
                    .setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
                        @Override
                        public boolean onPreferenceChange(Preference preference, Object newValue) {
                            int connectionType = Integer.parseInt((String) newValue);
                            updateConnectionPreferenceSummary(preference, connectionType);
                            return true;
                        }
                    });
        }
    }

    private void setupBluetoothDevicePreferences() {
        final ClearBTDialogPreference preference = (ClearBTDialogPreference) findPreference(getString(R.string
                .pref_bluetooth_device_address_key));
        if (preference != null) {
            updateBluetoothDevicePreference(preference, dpPrefs.getBluetoothDeviceAddress());
            preference.setOnResultListener(new ClearBTDialogPreference.OnResultListener() {
                @Override
                public void onResult(boolean result) {
                    if (result) {
                        updateBluetoothDevicePreference(preference, dpPrefs.getBluetoothDeviceAddress());
                    }
                }
            });
        }
    }

    private void updateBluetoothDevicePreference(Preference preference, String deviceAddress) {
        if (TextUtils.isEmpty(deviceAddress)) {
            preference.setEnabled(false);
            preference.setTitle(R.string.pref_no_saved_bluetooth_device_title);
            preference.setSummary("");
        } else {
            preference.setEnabled(true);
            preference.setSummary(deviceAddress);

            final String deviceName = dpPrefs.getBluetoothDeviceName();
            if (deviceName != null) {
                preference.setTitle(getString(R.string.pref_forget_bluetooth_device_title, deviceName));
            } else
                preference.setTitle(getString(R.string.pref_forget_bluetooth_device_address));
        }
    }

    private void updateConnectionPreferenceSummary(Preference preference, int connectionType) {
        String connectionName;
        switch (connectionType) {
            case ConnectionType.TYPE_USB:
                connectionName = "USB";
                break;

            case ConnectionType.TYPE_UDP:
                connectionName = "UDP";
                break;

            case ConnectionType.TYPE_TCP:
                connectionName = "TCP";
                break;

            case ConnectionType.TYPE_BLUETOOTH:
                connectionName = "BLUETOOTH";
                break;

            default:
                connectionName = null;
                break;
        }

        if (connectionName != null)
            preference.setSummary(connectionName);
    }

    private void initSummaryPerPrefs() {
        mDefaultSummaryPrefs.clear();

        mDefaultSummaryPrefs.add(getString(R.string.pref_baud_type_key));
        mDefaultSummaryPrefs.add(getString(R.string.pref_server_port_key));
        mDefaultSummaryPrefs.add(getString(R.string.pref_server_ip_key));
        mDefaultSummaryPrefs.add(getString(R.string.pref_udp_server_port_key));
        mDefaultSummaryPrefs.add(getString(R.string.pref_rc_quickmode_left_key));
        mDefaultSummaryPrefs.add(getString(R.string.pref_rc_quickmode_right_key));
        mDefaultSummaryPrefs.add(getString(R.string.pref_export_mail));
        //  mDefaultSummaryPrefs.add("pref_sensor_platform_update_interval"); //TODO Sensor stuff settings
    }

    /**
     * This is used to update the mavlink version preference.
     *
     * @param version mavlink version
     */
    private void updateMavlinkVersionPreference(String version) {
        final Preference mavlinkVersionPref = findPreference(getString(R.string.pref_mavlink_version_key));
        if (mavlinkVersionPref != null) {
            final HitBuilders.EventBuilder mavlinkEvent = new HitBuilders.EventBuilder()
                    .setCategory(GAUtils.Category.MAVLINK_CONNECTION);

            if (version == null) {
                mavlinkVersionPref.setSummary(getString(R.string.empty_content));
                mavlinkEvent.setAction("Mavlink version unset");
            } else {
                mavlinkVersionPref.setSummary('v' + version);
                mavlinkEvent.setAction("Mavlink version set").setLabel(version);
            }

            // Record the mavlink version
            GAUtils.sendEvent(mavlinkEvent);
        }
    }

    private void updateFirmwareVersionPreference(String firmwareVersion) {
        final Preference firmwareVersionPref = findPreference(getString(R.string.pref_firmware_version_key));
        if (firmwareVersionPref != null) {
            final HitBuilders.EventBuilder firmwareEvent = new HitBuilders.EventBuilder()
                    .setCategory(GAUtils.Category.MAVLINK_CONNECTION);

            if (firmwareVersion == null) {
                firmwareVersionPref.setSummary(getString(R.string.empty_content));
                firmwareEvent.setAction("Firmware version unset");
            } else {
                firmwareVersionPref.setSummary(firmwareVersion);
                firmwareEvent.setAction("Firmware version set").setLabel(firmwareVersion);
            }

            // Record the firmware version.
            GAUtils.sendEvent(firmwareEvent);
        }
    }

    private boolean updateMapSettingsPreference(final String mapProviderName) {
        final DPMapProvider mapProvider = DPMapProvider.getMapProvider(mapProviderName);
        if (mapProvider == null)
            return false;

        final Preference providerPrefs = findPreference(getText(R.string.pref_map_provider_settings_key));
        if (providerPrefs != null) {
            providerPrefs.setOnPreferenceClickListener(new Preference.OnPreferenceClickListener() {
                @Override
                public boolean onPreferenceClick(Preference preference) {
                    startActivity(new Intent(getActivity(), MapPreferencesActivity.class).putExtra(
                            MapPreferencesActivity.EXTRA_MAP_PROVIDER_NAME, mapProviderName));
                    return true;
                }
            });
        }
        return true;
    }


    @Override
    public void onSharedPreferenceChanged(SharedPreferences sharedPreferences, String key) {
        final Preference preference = findPreference(key);
        if (preference == null) {
            return;
        }

        if (mDefaultSummaryPrefs.contains(key)) {
            preference.setSummary(sharedPreferences.getString(key, ""));
        }
    }

    private void setupPeriodicControls() {
        final PreferenceCategory periodicSpeechPrefs = (PreferenceCategory) findPreference(getString(R.string.pref_tts_periodic_key));
        ListPreference periodic = ((ListPreference) periodicSpeechPrefs.getPreference(0));
        periodic.setOnPreferenceChangeListener(new Preference.OnPreferenceChangeListener() {
            @Override
            public boolean onPreferenceChange(Preference preference, final Object newValue) {
                // Broadcast the event locally on update.
                // A handler is used to that the current action has the time to
                // return, and store the value in the preferences.
                mHandler.post(new Runnable() {
                    @Override
                    public void run() {
                        lbm.sendBroadcast(new Intent(ACTION_UPDATED_STATUS_PERIOD)
                                .putExtra(EXTRA_UPDATED_STATUS_PERIOD, (String) newValue));

                        setupPeriodicControls();
                    }
                });
                return true;
            }
        });

        int val = Integer.parseInt(periodic.getValue());

        final boolean isEnabled = val != 0;
        if (isEnabled) {
            periodic.setSummary(getString(R.string.pref_tts_status_every) + " " + val + " "
                    + getString(R.string.pref_tts_seconds));
        } else {
            periodic.setSummary(R.string.pref_tts_periodic_status_disabled);
        }

        for (int i = 1; i < periodicSpeechPrefs.getPreferenceCount(); i++) {
            periodicSpeechPrefs.getPreference(i).setEnabled(isEnabled);
        }
    }

    @Override
    public void onStart() {
        super.onStart();
        dpApp.addApiListener(this);
    }

    @Override
    public void onStop() {
        super.onStop();
        dpApp.removeApiListener(this);
    }

    @Override
    public void onResume() {
        super.onResume();
        getPreferenceScreen().getSharedPreferences().registerOnSharedPreferenceChangeListener(this);
    }

    @Override
    public void onPause() {
        super.onPause();
        getPreferenceScreen().getSharedPreferences().unregisterOnSharedPreferenceChangeListener(
                this);
    }

    @Override
    public void onApiConnected() {
        Drone drone = dpApp.getDrone();
        State droneState = drone.getAttribute(AttributeType.STATE);
        Type droneType = drone.getAttribute(AttributeType.TYPE);
        final int mavlinkVersion = droneState == null
                ? State.INVALID_MAVLINK_VERSION
                : droneState.getMavlinkVersion();

        if (mavlinkVersion != State.INVALID_MAVLINK_VERSION) {
            updateMavlinkVersionPreference(String.valueOf(mavlinkVersion));
        } else {
            updateMavlinkVersionPreference(null);
        }

        String firmwareVersion = droneType == null ? null : droneType.getFirmwareVersion();
        updateFirmwareVersionPreference(firmwareVersion);

        lbm.registerReceiver(broadcastReceiver, intentFilter);
    }

    @Override
    public void onApiDisconnected() {
        lbm.unregisterReceiver(broadcastReceiver);
    }





    private void sendMail(String email, String subject, String messageBody) {
        Session session = createSessionObject();
        try {
            Message message = createMessage(email, subject, messageBody, session);
            new SendMailTask().execute(message);
        } catch (AddressException e) {
            e.printStackTrace();
        } catch (MessagingException e) {
            e.printStackTrace();
        } catch (UnsupportedEncodingException e) {
            e.printStackTrace();
        }
    }

    private Session createSessionObject() {
        Properties properties = new Properties();
        properties.put("mail.smtp.auth", "true");
        properties.put("mail.smtp.starttls.enable", "true");
        properties.put("mail.smtp.host", "smtp.gmail.com");
        properties.put("mail.smtp.port", "587");

        return Session.getInstance(properties, new javax.mail.Authenticator() {
            protected PasswordAuthentication getPasswordAuthentication() {
                return new PasswordAuthentication("owasp.data.provider@gmail.com", "owasp2300");
            }
        });
    }

    private Message createMessage(String email, String subject, String messageBody, Session session) throws MessagingException, UnsupportedEncodingException {
        Message message = new MimeMessage(session);
        message.setFrom(new InternetAddress("owasp.data.provider@gmail.com", "Drone Data Provider"));
        message.addRecipient(Message.RecipientType.TO, new InternetAddress(email, email));
        message.setSubject(subject);
        message.setText(messageBody);
        return message;
    }

    private class SendMailTask extends AsyncTask<Message, Void, Void> {
        private ProgressDialog progressDialog;

        @Override
        protected void onPreExecute() {
            super.onPreExecute();
            progressDialog = ProgressDialog.show(getActivity(), "Please wait", "Sending data", true, false);
        }

        protected void onPostExecute(Void aVoid) {
            super.onPostExecute(aVoid);
            progressDialog.dismiss();

        }

        protected Void doInBackground(Message... messages) {
            try {
                Transport.send(messages[0]);
            } catch (MessagingException e) {
                e.printStackTrace();
            }
            return null;
        }
    }

}
