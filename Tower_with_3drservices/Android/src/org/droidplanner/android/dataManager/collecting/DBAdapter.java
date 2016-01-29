package org.droidplanner.android.dataManager.collecting;

import android.content.ContentValues;
import android.content.Context;
import android.database.Cursor;
import android.database.sqlite.SQLiteDatabase;
import android.util.Log;

import com.o3dr.services.android.lib.drone.property.FloatSensor;
import com.o3dr.services.android.lib.drone.property.IntSensor;

import java.util.Date;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by Yndal on 02-03-2015.
 */
public class DBAdapter {
    private static final String TAG = "DBAdapter";

    private DBHelper helper;
    private SQLiteDatabase db;

    public DBAdapter(Context context) {
        helper = new DBHelper(context);
        open();
    }

    public void open() {
        db = helper.getWritableDatabase();
    }

    /*
     * Closes the DB
     */
    public void close() {
        db.close();
    }


    public List<IntSensorFromDB> getAllIntSensors(){
        String[] FROM = null;
        String WHERE = null;
        String[] WHERE_ARGS = null;

        Cursor query = db.query(DbConstants.IntTable.TABLE_NAME, FROM, WHERE, WHERE_ARGS, null, null,  null);

        Log.d(TAG, "getAllIntSensors() returned: " + query.getCount());

        query.moveToFirst();
        List<IntSensorFromDB> list = new LinkedList<>();

        while(query.moveToNext()) {
            int id = query.getInt(0);
            double latitude = query.getDouble(1);
            double longitude = query.getDouble(2);
            double altitude = query.getDouble(3);
            long time = query.getLong(4);
            int value = query.getInt(5);

            list.add(new IntSensorFromDB(id, latitude, longitude, altitude, time, value));
        }
        query.close();
        return list;
    }

    public List<FloatSensorFromDB> getAllFloatSensors(){
        String[] FROM = null;
        String WHERE = null;
        String[] WHERE_ARGS = null;

        Cursor query = db.query(DbConstants.FloatTable.TABLE_NAME, FROM, WHERE, WHERE_ARGS, null, null,  null);


        Log.d(TAG, "getAllFloatSensors() returned: " + query.getCount());

        query.moveToFirst();
        List<FloatSensorFromDB> list = new LinkedList<>();

        while(query.moveToNext()) {
            int id = query.getInt(0);
            double latitude = query.getDouble(1);
            double longitude = query.getDouble(2);
            double altitude = query.getDouble(3);
            long time = query.getLong(4);
            float value = query.getFloat(5);

            list.add(new FloatSensorFromDB(id, latitude, longitude, altitude, time, value));
        }
        query.close();
        return list;
    }

    public String getAllIntSensors_CSV(){
        return getAllSensors_CSV(DbConstants.IntTable.TABLE_NAME);
    }

    public String getAllFloatSensors_CSV(){
        return getAllSensors_CSV(DbConstants.FloatTable.TABLE_NAME);
    }

    private String getAllSensors_CSV(String table){
        final String COMMA = ",";
        StringBuilder sb = new StringBuilder();

        Cursor query = db.query(table, null, null, null, null, null, null);
        String[] columnNames = query.getColumnNames();
        for(String col : columnNames)
            sb.append(col + COMMA);

        sb.append("Time (human readable)" + COMMA);
        sb.append('\n');
        
        //query.moveToFirst();
        while(query.moveToNext()){
            for(int i=0; i<query.getColumnCount(); i++) {
                String s = query.getString(i);
                sb.append(s + COMMA);
            }

            String dateStr = new Date(query.getLong(4)/1000).toString();
            sb.append(dateStr + COMMA);
            sb.append('\n');
        }

        //Remove the very last comma
        sb.replace(sb.length() - 2, sb.length(), "");

        return sb.toString();
    }


    public long save(IntSensor sensor){
        if(sensor == null)
            return -1;

        ContentValues values = new ContentValues();
        values.put(DbConstants.IntTable.LATITUDE, sensor.getLatitude());
        values.put(DbConstants.IntTable.LONGITUDE, sensor.getLongitude());
        values.put(DbConstants.IntTable.ALTITUDE, sensor.getAltitudeInCm());
        values.put(DbConstants.IntTable.TIME, sensor.getTime());
        values.put(DbConstants.IntTable.VALUE, sensor.getSensorValue());

        return db.insert(DbConstants.IntTable.TABLE_NAME, null, values);
    }

    public long save(FloatSensor sensor){
        if(sensor == null)
            return -1;

        ContentValues values = new ContentValues();
        values.put(DbConstants.FloatTable.LATITUDE, sensor.getLatitude());
        values.put(DbConstants.FloatTable.LONGITUDE, sensor.getLongitude());
        values.put(DbConstants.FloatTable.ALTITUDE, sensor.getAltitudeInCm());
        values.put(DbConstants.FloatTable.TIME, sensor.getTime());
        values.put(DbConstants.FloatTable.VALUE, sensor.getSensorValue());

        return db.insert(DbConstants.FloatTable.TABLE_NAME, null, values);
    }

    /*
     * Delete all data
     */
    public void clearAllTables() {
        db.delete(DbConstants.IntTable.TABLE_NAME, null, null);
        db.delete(DbConstants.FloatTable.TABLE_NAME, null, null);
    }
}
