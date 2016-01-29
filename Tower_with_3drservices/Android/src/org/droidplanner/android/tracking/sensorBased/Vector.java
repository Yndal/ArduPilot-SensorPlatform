package org.droidplanner.android.tracking.sensorBased;

import com.google.android.gms.maps.model.LatLng;
import com.google.maps.android.geometry.Point;

/**
 * Created by lynd on 27/04/15.
 */
public class Vector {
    private Point p1;
    private Point p2;

    public Vector(Point p1, Point p2){
        this.p1 = p1;
        this.p2 = p2;
    }

    public Point getP1() {
        return p1;
    }

    public Point getP2() {
        return p2;
    }

    public Point getCenterPoint(){
        double lat = p1.y + (p2.y- p1.y)/2;
        double lng = p1.x + (p2.x- p1.x)/2;
        return new Point(lng, lat);
    }

    public Vector getOrthogonalVector(){
        //This for aligning the vectors correctly later
        Point center = getCenterPoint();

        /**********************************
         * Extracting coordinates from vector
         * called x in this context
         *********************************/
        double xLat = p2.y- p1.y; //n
        double xLng = p2.x- p1.x; //m

        /**********************************
         * Create orthogonal vector (but misplaced in the coordinate system)
         *
         * Normal vector (m,n) => orthogonal vector (-n,m)
         *********************************/
        double yLat = xLng;
        double yLng = -xLat;
        Vector orthMisplaced = new Vector(new Point(0,0), new Point(yLng,yLat));


        /**********************************
         * Align the two centerpoints of the vectors
         *********************************/
        Point origCenter = getCenterPoint();
        Point orthCenter = orthMisplaced.getCenterPoint();
        double latOffset = origCenter.y-orthCenter.y;
        double lngOffset = origCenter.x-orthCenter.x;


        /**********************************
         * Create actual orthogonal vector
         *********************************/
        Point orth1 = new Point(orthMisplaced.getP1().x+lngOffset, orthMisplaced.getP1().y+latOffset);
        Point orth2 = new Point(orthMisplaced.getP2().x+lngOffset, orthMisplaced.getP2().y+latOffset);
        Vector orthogonal = new Vector(orth1, orth2);

        return orthogonal;

    }

    public Vector getOppositeVector(){
        return new Vector(p2, p1);
    }
    public double length(){
        return Math.sqrt(Math.pow(p2.x- p1.x,2)+
                Math.pow(p2.y- p1.y,2));
    }

    /**
     * Scaled this vector around the center point
     * @param factor The factor of which the vector shall be scaled
     */
    public void scaleAroundCenter(double factor){
        Point center = getCenterPoint();

        double lat1 = center.y + (p1.y - center.y)*factor;
        double lng1 = center.x + (p1.x - center.x)*factor;

        double lat2 = center.y + (p2.y - center.y)*factor;
        double lng2 = center.x + (p2.x - center.x)*factor;

        p1 = new Point(lng1, lat1);
        p2 = new Point(lng2, lat2);

        /*
        latitude	The point's latitude. This will be clamped to between -90 degrees and +90 degrees inclusive.
        longitude	The point's longitude. This will be normalized to be within -180 degrees inclusive and +180 degrees exclusive.
         */
    }


    public void setLengthAroundCenter(double newLength){
        Point center = getCenterPoint();

        /**********************************************************
         * Compute new length
         *********************************************************/
        double oldLength = length();
        double factor = newLength / oldLength;

        double x = (p2.x - p1.x) * factor;
        double y = (p2.y - p1.y) * factor;

        /**********************************************************
         * Create new point with respect of both centerpoint and new length
         *********************************************************/

        p1 = new Point(center.x - x/2, center.y - y/2);
        p2 = new Point(center.x + x/2, center.y + y/2);
    }

    public void setLengthInMetersAroundCenter(double newLength){
        Point center = getCenterPoint();

        /**********************************************************
         * Compute new length
         *********************************************************/
        double oldLength = TrackerHelper.haversineDistance(new LatLng(p1.y, p1.x), new LatLng(p2.y, p2.x));
        double factor = newLength / oldLength;

        double x = (p2.x - p1.x) * factor;
        double y = (p2.y - p1.y) * factor;

        /**********************************************************
         * Create new point with respect of both centerpoint and new length
         *********************************************************/

        p1 = new Point(center.x - x/2, center.y - y/2);
        p2 = new Point(center.x + x/2, center.y + y/2);
    }

    public void setLengthFromP1(double newLength){
        Point p1 = this.p1;

        /**********************************************************
         * Compute new length
         *********************************************************/
        double oldLength = length();
        double factor = newLength / oldLength;

        double x = (p2.x - p1.x) * factor;
        double y = (p2.y - p1.y) * factor;

        /**********************************************************
         * Create new point with respect of both centerpoint and new length
         *********************************************************/

        Point p2 = new Point(p1.x + x, p1.y + y);

        this.p1 = p1;
        this.p2 = p2;
    }

    public void setLengthInMeterFromP1(double meters){
        double curLength = TrackerHelper.haversineDistance(new LatLng(p1.y, p1.x), new LatLng(p2.y, p2.x));
        Point p1 = this.p1;

        /**********************************************************
         * Compute new length
         *********************************************************/
        double factor = meters / curLength;

        double x = (p2.x - p1.x) * factor;
        double y = (p2.y - p1.y) * factor;

        /**********************************************************
         * Create new point with respect of both centerpoint and new length
         *********************************************************/

        Point p2 = new Point(p1.x + x, p1.y + y);

        this.p1 = p1;
        this.p2 = p2;
    }
}
