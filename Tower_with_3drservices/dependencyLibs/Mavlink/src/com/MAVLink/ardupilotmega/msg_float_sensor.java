        // MESSAGE FLOAT_SENSOR PACKING
package com.MAVLink.ardupilotmega;
import com.MAVLink.MAVLinkPacket;
import com.MAVLink.Messages.MAVLinkMessage;
import com.MAVLink.Messages.MAVLinkPayload;
        //import android.util.Log;
        
        /**
        * Onboard sensor message.
        */
        public class msg_float_sensor extends MAVLinkMessage{
        
        public static final int MAVLINK_MSG_ID_FLOAT_SENSOR = 231;
        public static final int MAVLINK_MSG_LENGTH = 24;
        private static final long serialVersionUID = MAVLINK_MSG_ID_FLOAT_SENSOR;
        
        
         	/**
        * Unix epoch time
        */
        public long time_usec;
         	/**
        * measured value
        */
        public float value;
         	/**
        * Altitude
        */
        public int altitude;
         	/**
        * Longitude
        */
        public int longitude;
         	/**
        * Latitude
        */
        public int latitude;
        
        
        /**
        * Generates the payload for a mavlink message for a message of this type
        * @return
        */
        public MAVLinkPacket pack(){
		MAVLinkPacket packet = new MAVLinkPacket();
		packet.len = MAVLINK_MSG_LENGTH;
		packet.sysid = 255;
		packet.compid = 190;
		packet.msgid = MAVLINK_MSG_ID_FLOAT_SENSOR;
        		packet.payload.putLong(time_usec);
        		packet.payload.putFloat(value);
        		packet.payload.putInt(altitude);
        		packet.payload.putInt(longitude);
        		packet.payload.putInt(latitude);
        
		return packet;
        }
        
        /**
        * Decode a float_sensor message into this class fields
        *
        * @param payload The message to decode
        */
        public void unpack(MAVLinkPayload payload) {
        payload.resetIndex();
        	    this.time_usec = payload.getLong();
        	    this.value = payload.getFloat();
        	    this.altitude = payload.getInt();
        	    this.longitude = payload.getInt();
        	    this.latitude = payload.getInt();
        
        }
        
        /**
        * Constructor for a new message, just initializes the msgid
        */
        public msg_float_sensor(){
    	msgid = MAVLINK_MSG_ID_FLOAT_SENSOR;
        }
        
        /**
        * Constructor for a new message, initializes the message with the payload
        * from a mavlink packet
        *
        */
        public msg_float_sensor(MAVLinkPacket mavLinkPacket){
        this.sysid = mavLinkPacket.sysid;
        this.compid = mavLinkPacket.compid;
        this.msgid = MAVLINK_MSG_ID_FLOAT_SENSOR;
        unpack(mavLinkPacket.payload);
        //Log.d("MAVLink", "FLOAT_SENSOR");
        //Log.d("MAVLINK_MSG_ID_FLOAT_SENSOR", toString());
        }
        
                  
        /**
        * Returns a string with the MSG name and data
        */
        public String toString(){
    	return "MAVLINK_MSG_ID_FLOAT_SENSOR -"+" time_usec:"+time_usec+" value:"+value+" altitude:"+altitude+" longitude:"+longitude+" latitude:"+latitude+"";
        }
        }
        