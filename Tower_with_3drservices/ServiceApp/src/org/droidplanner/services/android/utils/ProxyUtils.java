package org.droidplanner.services.android.utils;

import android.util.Log;

import com.o3dr.services.android.lib.drone.mission.item.MissionItem;
import com.o3dr.services.android.lib.drone.mission.item.command.CameraTrigger;
import com.o3dr.services.android.lib.drone.mission.item.command.ChangeSpeed;
import com.o3dr.services.android.lib.drone.mission.item.command.EpmGripper;
import com.o3dr.services.android.lib.drone.mission.item.command.ReturnToLaunch;
import com.o3dr.services.android.lib.drone.mission.item.command.SetRelay;
import com.o3dr.services.android.lib.drone.mission.item.command.SetServo;
import com.o3dr.services.android.lib.drone.mission.item.command.Takeoff;
import com.o3dr.services.android.lib.drone.mission.item.command.YawCondition;
import com.o3dr.services.android.lib.drone.mission.item.complex.CameraDetail;
import com.o3dr.services.android.lib.drone.mission.item.complex.StructureScanner;
import com.o3dr.services.android.lib.drone.mission.item.complex.Survey;
import com.o3dr.services.android.lib.drone.mission.item.complex.SurveyDetail;
import com.o3dr.services.android.lib.drone.mission.item.spatial.Circle;
import com.o3dr.services.android.lib.drone.mission.item.spatial.Land;
import com.o3dr.services.android.lib.drone.mission.item.spatial.RegionOfInterest;
import com.o3dr.services.android.lib.drone.mission.item.spatial.SplineWaypoint;
import com.o3dr.services.android.lib.drone.mission.item.spatial.Waypoint;

import org.droidplanner.core.helpers.coordinates.Coord2D;
import org.droidplanner.core.helpers.units.Altitude;
import org.droidplanner.core.helpers.units.Length;
import org.droidplanner.core.helpers.units.Speed;
import org.droidplanner.core.mission.Mission;
import org.droidplanner.core.mission.commands.ConditionYaw;
import org.droidplanner.core.mission.commands.ReturnToHome;
import org.droidplanner.core.mission.commands.SetRelayImpl;
import org.droidplanner.core.survey.CameraInfo;
import org.droidplanner.core.survey.SurveyData;

import java.util.List;

/**
 * Created by fhuya on 11/10/14.
 */
public class ProxyUtils {

    private static final String TAG = ProxyUtils.class.getSimpleName();

    public static CameraDetail getCameraDetail(CameraInfo camInfo) {
        if(camInfo == null) return null;
        return new CameraDetail(camInfo.name, camInfo.sensorWidth,
                camInfo.sensorHeight, camInfo.sensorResolution, camInfo.focalLength,
                camInfo.overlap, camInfo.sidelap, camInfo.isInLandscapeOrientation);
    }

    public static CameraInfo getCameraInfo(CameraDetail camDetail){
        if(camDetail == null) return null;

        CameraInfo camInfo = new CameraInfo();
        camInfo.name = camDetail.getName();
        camInfo.sensorWidth = camDetail.getSensorWidth();
        camInfo.sensorHeight = camDetail.getSensorHeight();
        camInfo.sensorResolution = camDetail.getSensorResolution();
        camInfo.focalLength = camDetail.getFocalLength();
        camInfo.overlap = camDetail.getOverlap();
        camInfo.sidelap = camDetail.getSidelap();
        camInfo.isInLandscapeOrientation = camDetail.isInLandscapeOrientation();

        return camInfo;
    }

    public static SurveyDetail getSurveyDetail(SurveyData surveyData) {
        SurveyDetail surveyDetail = new SurveyDetail();
        surveyDetail.setCameraDetail(getCameraDetail(surveyData.getCameraInfo()));
        surveyDetail.setSidelap(surveyData.getSidelap());
        surveyDetail.setOverlap(surveyData.getOverlap());
        surveyDetail.setAngle(surveyData.getAngle());
        surveyDetail.setAltitude(surveyData.getAltitude().valueInMeters());
        return surveyDetail;
    }

    public static org.droidplanner.core.mission.MissionItem getMissionItemImpl(Mission mission, MissionItem proxyItem) {
        if (proxyItem == null)
            return null;

        org.droidplanner.core.mission.MissionItem missionItemImpl;
        switch (proxyItem.getType()) {

            case CAMERA_TRIGGER: {
                CameraTrigger proxy = (CameraTrigger) proxyItem;

                org.droidplanner.core.mission.commands.CameraTrigger temp = new org.droidplanner
                        .core.mission.commands.CameraTrigger(mission,
                        new Length(proxy.getTriggerDistance()));

                missionItemImpl = temp;
                break;
            }
            case CHANGE_SPEED: {
                ChangeSpeed proxy = (ChangeSpeed) proxyItem;

                org.droidplanner.core.mission.commands.ChangeSpeed temp = new org.droidplanner
                        .core.mission.commands.ChangeSpeed(mission, new Speed(proxy.getSpeed()));

                missionItemImpl = temp;
                break;
            }
            case EPM_GRIPPER: {
                EpmGripper proxy = (EpmGripper) proxyItem;

                org.droidplanner.core.mission.commands.EpmGripper temp = new org.droidplanner
                        .core.mission.commands.EpmGripper(mission, proxy.isRelease());

                missionItemImpl = temp;
                break;
            }
            case RETURN_TO_LAUNCH: {
                ReturnToLaunch proxy = (ReturnToLaunch) proxyItem;

                ReturnToHome temp = new ReturnToHome(mission);
                temp.setHeight(new Altitude(proxy.getReturnAltitude()));

                missionItemImpl = temp;
                break;
            }
            case SET_SERVO: {
                SetServo proxy = (SetServo) proxyItem;

                org.droidplanner.core.mission.commands.SetServo temp = new org.droidplanner.core
                        .mission.commands.SetServo(mission, proxy.getChannel(), proxy.getPwm());

                missionItemImpl = temp;
                break;
            }
            case TAKEOFF: {
                Takeoff proxy = (Takeoff) proxyItem;

                org.droidplanner.core.mission.commands.Takeoff temp = new org.droidplanner.core
                        .mission.commands.Takeoff(mission, new Altitude(proxy.getTakeoffAltitude()));

                missionItemImpl = temp;
                break;
            }
            case CIRCLE: {
                Circle proxy = (Circle) proxyItem;

                org.droidplanner.core.mission.waypoints.Circle temp = new org.droidplanner.core
                        .mission.waypoints.Circle(mission, MathUtils.latLongAltToCoord3D(proxy
                        .getCoordinate()));
                temp.setRadius(proxy.getRadius());
                temp.setTurns(proxy.getTurns());

                missionItemImpl = temp;
                break;
            }
            case LAND: {
                Land proxy = (Land) proxyItem;

                org.droidplanner.core.mission.waypoints.Land temp = new org.droidplanner.core
                        .mission.waypoints.Land(mission, MathUtils.latLongToCoord2D(proxy
                        .getCoordinate()));

                missionItemImpl = temp;
                break;
            }
            case REGION_OF_INTEREST: {
                RegionOfInterest proxy = (RegionOfInterest) proxyItem;

                org.droidplanner.core.mission.waypoints.RegionOfInterest temp = new org
                        .droidplanner.core.mission.waypoints.RegionOfInterest(mission,
                        MathUtils.latLongAltToCoord3D(proxy.getCoordinate()));

                missionItemImpl = temp;
                break;
            }
            case SPLINE_WAYPOINT: {
                SplineWaypoint proxy = (SplineWaypoint) proxyItem;

                org.droidplanner.core.mission.waypoints.SplineWaypoint temp = new org
                        .droidplanner.core.mission.waypoints.SplineWaypoint(mission,
                        MathUtils.latLongAltToCoord3D(proxy.getCoordinate()));
                temp.setDelay(proxy.getDelay());

                missionItemImpl = temp;
                break;
            }
            case STRUCTURE_SCANNER: {
                StructureScanner proxy = (StructureScanner) proxyItem;

                org.droidplanner.core.mission.waypoints.StructureScanner temp = new org
                        .droidplanner.core.mission.waypoints.StructureScanner(mission,
                        MathUtils.latLongAltToCoord3D(proxy.getCoordinate()));
                temp.setRadius((int) proxy.getRadius());
                temp.setNumberOfSteps(proxy.getStepsCount());
                temp.setAltitudeStep((int) proxy.getHeightStep());
                temp.enableCrossHatch(proxy.isCrossHatch());

                CameraDetail camDetail = proxy.getSurveyDetail().getCameraDetail();
                if(camDetail != null)
                    temp.setCamera(getCameraInfo(camDetail));

                missionItemImpl = temp;
                break;
            }
            case WAYPOINT: {
                Waypoint proxy = (Waypoint) proxyItem;

                org.droidplanner.core.mission.waypoints.Waypoint temp = new org.droidplanner.core
                        .mission.waypoints.Waypoint(mission, MathUtils.latLongAltToCoord3D(proxy
                        .getCoordinate()));
                temp.setAcceptanceRadius(proxy.getAcceptanceRadius());
                temp.setDelay(proxy.getDelay());
                temp.setOrbitCCW(proxy.isOrbitCCW());
                temp.setOrbitalRadius(proxy.getOrbitalRadius());
                temp.setYawAngle(proxy.getYawAngle());

                missionItemImpl = temp;
                break;
            }
            case SURVEY: {
                Survey proxy = (Survey) proxyItem;
                SurveyDetail surveyDetail = proxy.getSurveyDetail();
                List<Coord2D> polygonPoints = MathUtils.latLongToCoord2D(proxy.getPolygonPoints());

                org.droidplanner.core.mission.survey.Survey temp = new org.droidplanner.core
                        .mission.survey.Survey(mission, polygonPoints);

                if(surveyDetail != null) {
                    CameraDetail cameraDetail = surveyDetail.getCameraDetail();
                    if(cameraDetail != null)
                        temp.setCameraInfo(getCameraInfo(cameraDetail));

                    temp.update(surveyDetail.getAngle(), new Altitude(surveyDetail.getAltitude()),
                            surveyDetail.getOverlap(), surveyDetail.getSidelap());
                }

                try {
                    temp.build();
                } catch (Exception e) {
                    Log.e(TAG, e.getMessage(), e);
                }

                missionItemImpl = temp;
                break;
            }
            case YAW_CONDITION: {
                YawCondition proxy = (YawCondition) proxyItem;

                ConditionYaw temp = new ConditionYaw(mission, proxy.getAngle(), proxy.isRelative());
                temp.setAngularSpeed(proxy.getAngularSpeed());

                missionItemImpl = temp;
                break;
            }

            case SET_RELAY:
                SetRelay proxy = (SetRelay) proxyItem;
                missionItemImpl = new SetRelayImpl(mission, proxy.getRelayNumber(), proxy.isEnabled());
                break;

            default:
                missionItemImpl = null;
                break;
        }

        return missionItemImpl;
    }

    public static MissionItem getProxyMissionItem(org.droidplanner.core.mission.MissionItem itemImpl) {
        if (itemImpl == null)
            return null;

        MissionItem proxyMissionItem;
        switch (itemImpl.getType()) {
            case WAYPOINT: {
                org.droidplanner.core.mission.waypoints.Waypoint source = (org.droidplanner.core.mission.waypoints.Waypoint) itemImpl;

                Waypoint temp = new Waypoint();
                temp.setCoordinate(MathUtils.coord3DToLatLongAlt(source.getCoordinate()));
                temp.setAcceptanceRadius(source.getAcceptanceRadius());
                temp.setDelay(source.getDelay());
                temp.setOrbitalRadius(source.getOrbitalRadius());
                temp.setOrbitCCW(source.isOrbitCCW());
                temp.setYawAngle(source.getYawAngle());

                proxyMissionItem = temp;
                break;
            }

            case SPLINE_WAYPOINT: {
                org.droidplanner.core.mission.waypoints.SplineWaypoint source = (org.droidplanner.core.mission.waypoints.SplineWaypoint) itemImpl;

                SplineWaypoint temp = new SplineWaypoint();
                temp.setCoordinate(MathUtils.coord3DToLatLongAlt(source.getCoordinate()));
                temp.setDelay(source.getDelay());

                proxyMissionItem = temp;
                break;
            }

            case TAKEOFF: {
                org.droidplanner.core.mission.commands.Takeoff source = (org.droidplanner.core.mission.commands.Takeoff) itemImpl;

                Takeoff temp = new Takeoff();
                temp.setTakeoffAltitude(source.getFinishedAlt().valueInMeters());

                proxyMissionItem = temp;
                break;
            }
            case RTL: {
                ReturnToHome source = (ReturnToHome) itemImpl;

                ReturnToLaunch temp = new ReturnToLaunch();
                temp.setReturnAltitude(source.getHeight().valueInMeters());

                proxyMissionItem = temp;
                break;
            }
            case LAND: {
                org.droidplanner.core.mission.waypoints.Land source = (org.droidplanner.core.mission.waypoints.Land) itemImpl;

                Land temp = new Land();
                temp.setCoordinate(MathUtils.coord3DToLatLongAlt(source.getCoordinate()));

                proxyMissionItem = temp;
                break;
            }

            case CIRCLE: {
                org.droidplanner.core.mission.waypoints.Circle source = (org.droidplanner.core.mission.waypoints.Circle) itemImpl;

                Circle temp = new Circle();
                temp.setCoordinate(MathUtils.coord3DToLatLongAlt(source.getCoordinate()));
                temp.setRadius(source.getRadius());
                temp.setTurns(source.getNumberOfTurns());

                proxyMissionItem = temp;
                break;
            }

            case ROI: {
                org.droidplanner.core.mission.waypoints.RegionOfInterest source = (org.droidplanner.core.mission.waypoints.RegionOfInterest) itemImpl;

                RegionOfInterest temp = new RegionOfInterest();
                temp.setCoordinate(MathUtils.coord3DToLatLongAlt(source.getCoordinate()));

                proxyMissionItem = temp;
                break;
            }

            case SURVEY: {
                org.droidplanner.core.mission.survey.Survey source = (org.droidplanner.core.mission.survey.Survey) itemImpl;

                boolean isValid = true;
                try {
                    source.build();
                } catch (Exception e) {
                    isValid = false;
                }

                Survey temp = new Survey();
                temp.setValid(isValid);
                temp.setSurveyDetail(getSurveyDetail(source.surveyData));
                temp.setPolygonPoints(MathUtils.coord2DToLatLong(source.polygon.getPoints()));

                if(source.grid != null) {
                    temp.setGridPoints(MathUtils.coord2DToLatLong(source.grid.gridPoints));
                    temp.setCameraLocations(MathUtils.coord2DToLatLong(source.grid.getCameraLocations()));
                }

                temp.setPolygonArea(source.polygon.getArea().valueInSqMeters());

                proxyMissionItem = temp;
                break;
            }

            case CYLINDRICAL_SURVEY: {
                org.droidplanner.core.mission.waypoints.StructureScanner source = (org.droidplanner.core.mission.waypoints.StructureScanner) itemImpl;

                StructureScanner temp = new StructureScanner();
                temp.setSurveyDetail(getSurveyDetail(source.getSurveyData()));
                temp.setCoordinate(MathUtils.coord3DToLatLongAlt(source.getCoordinate()));
                temp.setRadius(source.getRadius().valueInMeters());
                temp.setCrossHatch(source.isCrossHatchEnabled());
                temp.setHeightStep(source.getEndAltitude().valueInMeters());
                temp.setStepsCount(source.getNumberOfSteps());
                temp.setPath(MathUtils.coord2DToLatLong(source.getPath()));

                proxyMissionItem = temp;
                break;
            }
            case CHANGE_SPEED: {
                org.droidplanner.core.mission.commands.ChangeSpeed source = (org.droidplanner.core.mission.commands.ChangeSpeed) itemImpl;

                ChangeSpeed temp = new ChangeSpeed();
                temp.setSpeed(source.getSpeed().valueInMetersPerSecond());

                proxyMissionItem = temp;
                break;
            }

            case CAMERA_TRIGGER: {
                org.droidplanner.core.mission.commands.CameraTrigger source = (org.droidplanner.core.mission.commands.CameraTrigger) itemImpl;

                CameraTrigger temp = new CameraTrigger();
                temp.setTriggerDistance(source.getTriggerDistance().valueInMeters());

                proxyMissionItem = temp;
                break;
            }
            case EPM_GRIPPER: {
                org.droidplanner.core.mission.commands.EpmGripper source = (org.droidplanner.core.mission.commands.EpmGripper) itemImpl;

                EpmGripper temp = new EpmGripper();
                temp.setRelease(source.isRelease());

                proxyMissionItem = temp;
                break;
            }

            case SET_SERVO: {
                org.droidplanner.core.mission.commands.SetServo source = (org.droidplanner.core.mission.commands.SetServo) itemImpl;

                SetServo temp = new SetServo();
                temp.setChannel(source.getChannel());
                temp.setPwm(source.getPwm());

                proxyMissionItem = temp;
                break;
            }
            case CONDITION_YAW: {
                ConditionYaw source = (ConditionYaw) itemImpl;

                YawCondition temp = new YawCondition();
                temp.setAngle(source.getAngle());
                temp.setAngularSpeed(source.getAngularSpeed());
                temp.setRelative(source.isRelative());

                proxyMissionItem = temp;
                break;
            }

            case SET_RELAY:{
                SetRelayImpl impl = (SetRelayImpl) itemImpl;

                SetRelay proxy = new SetRelay();
                proxy.setRelayNumber(impl.getRelayNumber());
                proxy.setEnabled(impl.isEnabled());

                proxyMissionItem = proxy;
                break;
            }

            default:
                proxyMissionItem = null;
                break;
        }

        return proxyMissionItem;
    }

}
