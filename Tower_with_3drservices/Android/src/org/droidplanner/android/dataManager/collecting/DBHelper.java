package org.droidplanner.android.dataManager.collecting;

import android.content.Context;
import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.util.Log;

import org.droidplanner.android.R;

import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;


/**
 * Created by Yndal on 02-03-2015.
 */
public class DBHelper extends SQLiteOpenHelper {
    //The DB's version
    public static final int DB_VERSION = 8;

    //DB name
    public static final String DB_NAME = "droneSensorData.db";

    //The Apps name - used for debugging
    private static final String APP_NAME = "droneSensorDrone";

    private Context context;

    /*
     * The constructor
     */
    public DBHelper(Context context) {
        super(context, DB_NAME, null, DB_VERSION);
        this.context = context;
    }

    @Override
    public void onCreate(SQLiteDatabase db) {
        runSQL(db, R.raw.createdb);
    }

    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        runSQL(db, R.raw.upgradedb);
        onCreate(db);
    }

    /*
     * Execute a queries to the DB
     */
    private void runSQL(SQLiteDatabase db, int resource) {
        InputStream in = context.getResources().openRawResource(resource);
        InputStreamReader reader = new InputStreamReader(in);
        BufferedReader br = new BufferedReader(reader);
        String sql;
        try {
            while((sql = br.readLine()) != null) {
                db.execSQL(sql);
            }
        } catch (Exception e) {
            Log.e(APP_NAME, e.getMessage());
        }
    }
}


