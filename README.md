ArduPilot-SensorPlatform
==========================
Based on the ArduPilot project, we created a sensor platform for off-the-shelf drones, that easily can be used by people with only sparse coding skills.

The project has been based on the following community contributed repositories:
https://github.com/ArduPilot/ardupilot

https://github.com/PX4/Firmware

https://github.com/DroidPlanner/Tower

https://github.com/ne0fhyk/3DRServices

For convenience, the Tower and 3DRServices has been merged into a single project in this repository. The additions to the Android code have been made in Android Studio 1.2 (https://sites.google.com/a/android.com/tools/download/studio/stable).

The Arduino sketch in the repository can be used to honor the I2C protocol that is implemented in the solution. If I2C device is to be used, the protocol can be seen in this matrix.

|   Register (HEX)  |   Semantics    |
| :---------------: | -------------- |
| 0x44              | Device address |
| 0x10              | MEASURE. Initiate sensor measurement and conversion. Returns nothing.
| 0x11              | COLLECT. Retrieve measure- ment. After this register is ad- dressed the master expects to read 2-4 bytes from the sensor module (slave) |
| 0x20              | TYPE. Send the enumerated type of the sensor module. Used to identify the sensor. After a write to the register, a single byte must be read from the slave |
| 0x30              | SETRATE. Configure the I2C data rate. Defaults to the Pix- hawk 100 kHz setting. Af- ter writing the register address the master will write a single byte representing an enumer- ated rate |
| 0x40              | TEST. The device returns a constant response - 0x42 read after register write. Used for driver initialization on the flight computer |



