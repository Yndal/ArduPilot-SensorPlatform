package com.o3dr.services.android.lib.drone.attribute;

/**
 * Stores the set of attribute types.
 */
public class AttributeType {

    private static final String PACKAGE_NAME = "com.o3dr.services.android.lib.attribute";

    //TODO Sensor stuff
    public static final String INT_SENSOR = PACKAGE_NAME + ".INT_SENSOR";
    public static final String FLOAT_SENSOR = PACKAGE_NAME + ".FLOAT_SENSOR";

    /**
     * Used to access the vehicle's altitude state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Altitude}
     */
    public static final String ALTITUDE = PACKAGE_NAME + ".ALTITUDE";

    /**
     * Used to access the vehicle's attitude state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Attitude}
     */
    public static final String ATTITUDE = PACKAGE_NAME + ".ATTITUDE";

    /**
     * Used to access the vehicle's battery state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Speed}
     */
    public static final String BATTERY = PACKAGE_NAME + ".BATTERY";

    /**
     * Used to access the set of camera information available for the connected drone.
     * @see {@link com.o3dr.services.android.lib.drone.property.CameraProxy}
     */
    public static final String CAMERA = PACKAGE_NAME + ".CAMERA";

    /**
     * Used to acces the vehicle's follow state.
     * @see {@link com.o3dr.services.android.lib.gcs.follow.FollowState}
     */
    public static final String FOLLOW_STATE = PACKAGE_NAME + ".FOLLOW_STATE";

    /**
     * Used to access the vehicle's guided state.
     * @see {@link com.o3dr.services.android.lib.drone.property.GuidedState}
     */
    public static final String GUIDED_STATE = PACKAGE_NAME + ".GUIDED_STATE";

    /**
     * Used to access the vehicle's gps state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Gps} object.
     */
    public static final String GPS = PACKAGE_NAME + ".GPS";

    /**
     * Used to access the vehicle's home state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Home}
     */
    public static final String HOME = PACKAGE_NAME + ".HOME";

    /**
     * Used to access the vehicle's mission state.
     * @see {@link com.o3dr.services.android.lib.drone.mission.Mission}
     */
    public static final String MISSION = PACKAGE_NAME + ".MISSION";

    /**
     * Used to access the vehicle's parameters.
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameters}
     * @see {@link com.o3dr.services.android.lib.drone.property.Parameter}
     */
    public static final String PARAMETERS = PACKAGE_NAME + ".PARAMETERS";

    /**
     * Used to access the vehicle's signal state.
     * @see {@link com.o3dr.services.android.lib.drone.property.Signal}
     */
    public static final String SIGNAL = PACKAGE_NAME + ".SIGNAL";

    /**
     * Used to access the vehicle's speed info.
     * @see {@link com.o3dr.services.android.lib.drone.property.Speed}
     */
    public static final String SPEED = PACKAGE_NAME + ".SPEED";

    /**
     * Used to access the vehicle state.
     * @see {@link com.o3dr.services.android.lib.drone.property.State} object.
     */
    public static final String STATE = PACKAGE_NAME + ".STATE";

    /**
     * Used to access the vehicle type.
     * @see {@link com.o3dr.services.android.lib.drone.property.Type}
     */
    public static final String TYPE = PACKAGE_NAME + ".TYPE";
}
