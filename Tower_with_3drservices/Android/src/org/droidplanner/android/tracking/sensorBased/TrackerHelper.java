package org.droidplanner.android.tracking.sensorBased;

import android.content.Context;
import android.util.Log;
import android.widget.Toast;

import com.google.android.gms.maps.model.LatLng;
import com.google.maps.android.geometry.Point;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by lynd on 26/04/15.
 */
public class TrackerHelper {
    public static final String TAG = "TrackerHelper";
    public final static double AVERAGE_RADIUS_OF_EARTH = 6372.8;
    private static final int MAX_POINTS = 100;
    private static Toast maxPointsToast;

    public static List<LatLng> computeRoute(Context context, List<LatLng> polygon, double granularityInMeters){//}, int granularityMeteres, double speedM_per_S){
        //Preconditions
        if(polygon.size() < 3)
            return polygon;

        //Create toast here and avoid that it is being displayed a million times within 5 seconds :)
        if(maxPointsToast == null && polygon.size() > MAX_POINTS)
            maxPointsToast = Toast.makeText(context, "Area is too big: Maximum waypoints is " + MAX_POINTS, Toast.LENGTH_SHORT);

        MyQueue<Point> result = new MyQueue<>();
        MyQueue<Point> points = new MyQueue<>();

        for(int i=0; i<polygon.size(); i++) {
            result.enqueue(new Point(polygon.get(i).longitude, polygon.get(i).latitude));
            points.enqueue(new Point(polygon.get(i).longitude, polygon.get(i).latitude));
        }

        Point first = result.getLastElement();
        Point second = points.get(0);//.dequeue();
        Point third = points.get(1);//.dequeue();//peek();

        Vector lastVector = new Vector(first, second);
        Vector currentVector = new Vector(second, third);

        while(!points.isEmpty()){
            if(points.size() < 3)
                break;
            //Preconditions
            /*if(points.size() < 3)
                break;
*/
            //third = points.dequeue();

            /********************************************************************************************
             * Create orthogonal vector and figure out which way it shall go (includes ROUGH scaling)   *
             ********************************************************************************************/
            Vector orthogonal = currentVector.getOrthogonalVector(); //Orthogonal validated!

            boolean foundDirection = false;
            int counter = 0;
            double divider = 2.0d;
            Point point = null;
            //Will figure out which way the vector shall point
            while(!foundDirection) {
                orthogonal = orthogonal.getOppositeVector();

                orthogonal.scaleAroundCenter(1 / divider); //Scale is validated!!
                point = orthogonal.getP2();

                foundDirection = isInsideOfPolygon(points, point);
                if(20 < counter++) {
                    //To avoid infinite loops
                    Log.d(TAG, "Unable to find direction for orthogonal");
                    break;
                }
            }


            /********************************************************************************************************
             * Scale the orthogonal vector to be of same length as the specified distance between paths (parameter) *
             ********************************************************************************************************/
            double length = getLengthFromLatLng_Meters_And_Direction(orthogonal.getCenterPoint(), granularityInMeters, orthogonal);
            orthogonal.setLengthInMetersAroundCenter(length);

            /***************************************************************************************
             * Create projected vector                                                             *
             ***************************************************************************************/
            Vector projectedVector = getParallelVectorProjection(currentVector, orthogonal);

            /***************************************************************************************
             * Find cross point                                                                    *
             ***************************************************************************************/
            Point thisIntersection = findIntersection(lastVector, projectedVector, true); //TODO What if this is null (Parallel)
            Point nextIntersection = findIntersection(new Vector(third, points.get(2)), projectedVector, true); //TODO What if this is null (Parallel)

            double havDist = haversineDistance(new LatLng(thisIntersection.y, thisIntersection.x), new LatLng(nextIntersection.y, nextIntersection.x));//thisIntersection, nextIntersection)
            if(length <= havDist) {
                result.enqueue(thisIntersection);
                points.enqueue(thisIntersection);
            }

            first = thisIntersection;//points.getLastElement();
            Point oldThird = third;
            second = nextIntersection;//points.dequeue();
            third = points.get(2);//points.dequeue();

            lastVector = new Vector(first, second);// "Intersection of projectedVector and next vector") projectedVector;
            currentVector = new Vector(oldThird, third);//("New intersection and next point");

            //while(distanceBetween() < distanceBetweenPaths/2)
            //


            if(MAX_POINTS < result.size()) {
                //To avoid the use of too much RAM and make the app block/crash
                if(maxPointsToast != null && maxPointsToast.getView().getWindowToken() == null)//.getWindowVisibility() == View.GONE)
                    maxPointsToast.show();
                break;
            }
            points.dequeue();
        }


        //Add last points as a part of route
        result.enqueue(first);
        result.enqueue(second);

        List<LatLng> respond = new ArrayList<>();
        while(!result.isEmpty()){
            Point p = result.dequeue();
            respond.add(new LatLng(p.y, p.x));
        }
        return respond;
    }

    private static double distanceBetween(Point p1, Point p2){
        double res =  Math.sqrt(Math.pow(p2.x-p1.x,2)+
                Math.pow(p2.y-p1.y,2));
        return res;
    }


    private static Vector getParallelVectorProjection(Vector vector, Vector direction){
        double latMove = direction.getP2().y -direction.getP1().y;
        double lonMove = direction.getP2().x-direction.getP1().x;

        Vector pVector = new Vector(new Point(vector.getP1().x + lonMove, vector.getP1().y + latMove),
                new Point(vector.getP2().x + lonMove, vector.getP2().y + latMove));

        return pVector;
    }

    private static Point findIntersection(Vector vector1, Vector vector2) {
        return findIntersection(vector1, vector2, false);
    }

    private static Point findIntersection(Vector vector1, Vector vector2, boolean allowExtendingVectors) {
        if (allowExtendingVectors) {
            vector1.setLengthAroundCenter(200);
            vector2.setLengthAroundCenter(200);
        }

        /**********************************
         * Vector 1 function: f(x)=ax+b
         *********************************/
        boolean isVertical1 = false;
        boolean isHorizontal1 = false;
        double a1;
        double b1;
        double x1 = 0;
        double y1 = 0;
        if (vector1.getP2().x == vector1.getP1().x) {
            //It's an vertical line
            a1 = Double.POSITIVE_INFINITY;
            isVertical1 = true;
            x1 = vector1.getP1().x;
        } else if (vector1.getP2().y == vector1.getP1().y) {
            //Horizontal line
            a1 = 0;
            isHorizontal1 = true;
            y1 = vector1.getP1().y;
        } else {
            a1 = (vector1.getP2().y - vector1.getP1().y) / (vector1.getP2().x - vector1.getP1().x);
        }
        b1 = vector1.getP1().y - a1 * vector1.getP1().x; //b = y-ax

        String debugging1 = "f1(x)=ax+b => f(x)=" + a1 + "x + " + b1;

        //f(x) = 2; Horizontal
        //f(x) =  ; Vertical

        /**********************************
         * Vector 2 function: f(x)=ax+b
         *********************************/
        boolean isVertical2 = false;
        boolean isHorizontal2 = false;
        double a2;
        double b2;
        double x2 = 0;
        double y2 = 0;
        if (vector2.getP2().x == vector2.getP1().x) {
            //It's a vertical line
            a2 = Double.POSITIVE_INFINITY;
            isVertical2 = true;
            x2 = vector2.getP1().x;
        } else if (vector2.getP2().y == vector2.getP1().y) {
            //Horizontal line
            a2 = 0;
            isHorizontal2 = true;
            y2 = vector2.getP1().y;
        } else {
            a2 = (vector2.getP2().y - vector2.getP1().y) / (vector2.getP2().x - vector2.getP1().x);
        }
        b2 = vector2.getP1().y - a2 * vector2.getP1().x; //b = y-ax
        String debugging2 = "f2(x)=ax+b => f(x)=" + a2 + "x + " + b2;


        /**********************************
         * Intersection - computation
         *********************************/
        if (isVertical1) {
            if (isVertical2) {
                //parallel
                return null;
            } else {
                if (isHorizontal2) {
                    //Orthogonal
                    return new Point(x1, y2);
                } else {
                    //1 is vertical
                    //2 is ?
                    //a1*x+b1 == a2*x+b2
                    //b1-b2 == a2*x-a1*x
                    //b1-b2 == (a2-a1)*x
                    //(b1-b2)/(a2-a1)=x

                    //f(x) = ax + b
                    //x = x1
                    double y = a2*x1+b2;

                    return new Point(x1, y);

                }
            }
        } else if (isHorizontal1){
            if (isVertical2) {
                //Orthogonal
                return new Point(x2, y1);
            } else {
                if (isHorizontal2) {
                    //Parallel
                    return null;
                } else {
                    //1 is horizontal
                    //2 is ?
                    //f(x) = ax + b
                    //y = y1
                    //y1 = a2*x-b
                    double x = (y1-b2)/a2;

                    return new Point(x, y1);
                }
            }
        } else {
            if (isVertical2) {
                //1 is ?
                //2 is vertical
                double y = a1*x2+b1;

                return new Point(x2,y);
            } else {
                if (isHorizontal2) {
                    //1 is ?
                    //2 is horizontal
                    double x = (y2-b1)/a1;

                    return new Point(x, y2);
                } else {
                    //Neither is vertical or horizontal
                    //a1*x+b1 == a2*x+b2
                    //b1-b2 == a2*x-a1*x
                    //b1-b2 == (a2-a1)*x
                    //(b1-b2)/(a2-a1)=x

                    double lng = (b1-b2)/(a2-a1); //x
                    double lat = a1 * lng + b1;

                    String debugging3 = "Intersection: (" + lng + "," + lat + ")";

                    return new Point(lng, lat);
                }
            }

        }
     }


    private static boolean isInsideOfPolygon(MyQueue<Point> polygon, Point latLng){
        // y = lat
        // x = lng
        int i;
        int j;
        boolean result = false;
        for (i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++) {
            if ((polygon.get(i).y > latLng.y) != (polygon.get(j).y > latLng.y) &&
                    (latLng.x < (polygon.get(j).x - polygon.get(i).x) * (latLng.y - polygon.get(i).y) / (polygon.get(j).y-polygon.get(i).y) + polygon.get(i).x)) {
                result = !result;
            }
        }
        return result;
    }

    //NB As these distances will be very small, there is not used a high-end algorithm
/*    public static double convertFromMeters2LatLng(double meters, Point latLng){

    }*/

    //TODO This method is copied from: http://reflectvicky.blogspot.dk/2013/04/calculate-haversine-distance-in-java.html
    public static double haversineDistance(LatLng l1, LatLng l2){//double lat1,double lat2,double lon1,double lon2) {
        double deltaLat = Math.toRadians(l1.latitude - l2.latitude);//lat1 - lat2);
        double deltaLong = Math.toRadians(l1.longitude - l2.longitude);//lon1 - lon2);
        double lat1R = Math.toRadians(l1.latitude);//lat1);
        double lat2R = Math.toRadians(l2.latitude);//lat2);
        double a = (Math.sin(deltaLat/2.0) * Math.sin(deltaLat/2.0)) +
                (Math.sin(deltaLong/2.0) * Math.sin(deltaLong/2.0) *
                        Math.cos(lat1R) * Math.cos(lat2R));
        double c = 2.0*Math.atan2(Math.sqrt(a), Math.sqrt(1.0 - a));
        double d = 3959.0*c;
        return d * 1000; //Added 1000 to go from km to m
    }

    private static double getLengthFromLatLng_Meters_And_Direction(Point startingPoint, double distance, Vector direction){
       // throw new RuntimeException("Distance error is within this method!! :)");
        Point pointDir = new Point(direction.getP2().x-direction.getP1().x, direction.getP2().y-direction.getP1().y);

        Vector vector = new Vector(startingPoint, new Point(startingPoint.x+pointDir.x, startingPoint.y+pointDir.y));
        vector.setLengthInMeterFromP1(distance);

        LatLng l1 = new LatLng(vector.getP1().y, vector.getP1().x);
        LatLng l2 = new LatLng(vector.getP2().y, vector.getP2().x);

        return haversineDistance(l1, l2);
    }

    private static double convertSensorUpdateInterval2MetersPerSecond(double invervalSeconds, double granularityInMeteres){
        //TODO
        //TODO
        //TODO

        return 5.0d;
    }

    private LatLng getCenterOf(List<LatLng> polygon){
        double lat = 0;
        double lng = 0;

        for(LatLng l : polygon) {
            lat += l.latitude;
            lng += l.longitude;
        }
        lat /= polygon.size();
        lng /= polygon.size();

        return new LatLng(lat, lng);
    }
}

class MyQueue<E> {
    private ArrayList<E> array = new ArrayList<>();

    public E peek(){
        if(array.size() <= 0)
            return null;
        return array.get(0);
    }

    public E dequeue(){
        return  array.remove(0);
    }

    public void enqueue(E e){
        array.add(e);
    }

    public int size(){
        return array.size();
    }

    public E dequeueLastElement(){
        return array.remove(array.size() - 1);
    }

    public boolean isEmpty(){
        return array.size() <= 0;
    }

    public void clear(){
        array.clear();
    }

    public E get(int location){
        return array.get(location);
    }

    public E getLastElement(){
        return array.get(array.size()-1);
    }
}