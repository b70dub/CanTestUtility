using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using System.Diagnostics;

/*
 * This example shows how to :
 *  1) Connect to Mavlink UDP stream
 *  2) Decode UDP packets into messages
 *  3) Decode messages into their component pieces
 * 
 *  For a complete list of messages see:
 *  https://pixhawk.ethz.ch/mavlink/
 *  Please note the description of the messages fields above are NOT in the order they are transmitted.
 *  To see the actual message structure you need to download the Mavlink 1.0 C library and peruse the headers there:
 *  https://github.com/mavlink/mavlink/
 *  I've converted two of the structures -- mavlink_global_position_int_t and mavlink_param_value_t -- and two of the messages types
 *  -- MAVLINK_MSG_ID_GLOBAL_POSITION_INT and MAVLINK_MSG_ID_PARAM_VALUE -- as an example. If you search for those strings in that library you can see what I did.
 *  
 */
namespace MavlinkComms
{
    public enum MAV_PARAM_TYPE
    {
        MAV_PARAM_TYPE_UINT8 = 1, /* 8-bit unsigned integer | */
        MAV_PARAM_TYPE_INT8 = 2, /* 8-bit signed integer | */
        MAV_PARAM_TYPE_UINT16 = 3, /* 16-bit unsigned integer | */
        MAV_PARAM_TYPE_INT16 = 4, /* 16-bit signed integer | */
        MAV_PARAM_TYPE_UINT32 = 5, /* 32-bit unsigned integer | */
        MAV_PARAM_TYPE_INT32 = 6, /* 32-bit signed integer | */
        MAV_PARAM_TYPE_UINT64 = 7, /* 64-bit unsigned integer | */
        MAV_PARAM_TYPE_INT64 = 8, /* 64-bit signed integer | */
        MAV_PARAM_TYPE_REAL32 = 9, /* 32-bit floating-point | */
        MAV_PARAM_TYPE_REAL64 = 10, /* 64-bit floating-point | */
        MAV_PARAM_TYPE_ENUM_END = 11, /*  | */
    };
    public struct mavlink_global_position_int_t
    {
        public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
        public float lat; ///< Latitude, expressed as int * 1E7
        public float lon; ///< Longitude, expressed as int * 1E7
        public float alt; ///< Altitude in meters, expressed as int * 1000 (millimeters), above MSL
        public float relative_alt; ///< Altitude above ground in meters, expressed as int * 1000 (millimeters)
        public float vx; ///< Ground X Speed (Latitude), expressed as m/s, short * 100
        public float vy; ///< Ground Y Speed (Longitude), expressed as m/s, short * 100
        public float vz; ///< Ground Z Speed (Altitude), expressed as m/s, short * 100
        public float hdg; ///< Compass heading in degrees, ushort * 100, 0.0..359.99 degrees. If unknown, set to: 65535
    } ;


    public struct mavlink_heartbeat_t
    {
        public uint type; ///< Type of the MAV (quadrotor, helicopter, etc., up to 15 types, defined in MAV_TYPE ENUM)
        public uint autopilot; ///< 	Autopilot type / class. defined in MAV_AUTOPILOT ENUM
        public uint base_mode; ///< 	System mode bitfield, see MAV_MODE_FLAG ENUM in mavlink/include/mavlink_types.h
        public uint custom_mode; ///< A bitfield for use for autopilot-specific flags.
        public uint system_status; ///< System status flag, see MAV_STATE ENUM
        public uint mavlink_version; ///< 	MAVLink version, not writable by user, gets added by protocol because of magic data type: uint8_t_mavlink_version
        public DateTime previous_heartbeat_time;
    } ;

    public struct mavlink_system_status_t
    {
        public uint onboard_control_sensors_present; ///< Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        public uint onboard_control_sensors_enabled; ///< 	Bitmask showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        public uint onboard_control_sensors_health; ///< 	Bitmask showing which onboard controllers and sensors are operational or have an error: Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        public uint load; ///< Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        public uint voltage_battery; ///< Battery voltage, in millivolts (1 = 1 millivolt)
        public int current_battery; ///< 	Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        public int battery_remaining; ///< 	Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        public uint drop_rate_comm; ///< 	Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        public uint errors_comm; ///< 	Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        public uint errors_count1; ///< Autopilot-specific errors
        public uint errors_count2; ///< Autopilot-specific errors
        public uint errors_count3; ///< Autopilot-specific errors
        public uint errors_count4; ///< Autopilot-specific errors
    } ;

    public struct mavlink_system_time_t
    {
        public uint time_unix_usec; ///< Timestamp of the master clock in microseconds since UNIX epoch.
        public uint time_boot_ms; ///< Timestamp of the component clock since boot time in milliseconds.
    } ;

    public struct mavlink_scaled_pressure_t
    {
        public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
        public float press_abs; ///< 	Absolute pressure (hectopascal)
        public float press_diff; ///< 	Differential pressure 1 (hectopascal)
        public uint temperature; ///< Temperature measurement (0.01 degrees celsius)
    } ;

    public struct mavlink_attitude_t
    {
        public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
        public float roll; ///< 	Roll angle (rad, -pi..+pi)
        public float pitch; ///< 	Pitch angle (rad, -pi..+pi)
        public float yaw; ///< Yaw angle (rad, -pi..+pi)
        public float rollspeed; ///< Roll angular speed (rad/s)
        public float pitchspeed; ///< 	Pitch angular speed (rad/s)
        public float yawspeed; ///< Yaw angular speed (rad/s)
    } ;

    public struct mavlink_scaled_imu_t
    {
        public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
        public float xacc; ///< X acceleration (mg)
        public float yacc; ///< Y acceleration (mg)
        public float zacc; ///< Z acceleration (mg)
        public float xgyro; ///< Angular speed around X axis (millirad /sec)
        public float ygyro; ///< Angular speed around Y axis (millirad /sec)
        public float zgyro; ///< Angular speed around Z axis (millirad /sec)
        public float xmag; ///< 	X Magnetic field (milli tesla)
        public float ymag; ///< 	Y Magnetic field (milli tesla)
        public float zmag; ///< 	Z Magnetic field (milli tesla)
    } ;

    public struct mavlink_scaled_imu_2
    {
        public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
        public float xacc; ///< X acceleration (mg)
        public float yacc; ///< Y acceleration (mg)
        public float zacc; ///< Z acceleration (mg)
        public float xgyro; ///< Angular speed around X axis (millirad /sec)
        public float ygyro; ///< Angular speed around Y axis (millirad /sec)
        public float zgyro; ///< Angular speed around Z axis (millirad /sec)
        public float xmag; ///< 	X Magnetic field (milli tesla)
        public float ymag; ///< 	Y Magnetic field (milli tesla)
        public float zmag; ///< 	Z Magnetic field (milli tesla)
    } ;

    public struct mavlink_hi_res_imu
    {
        public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
        public float xacc; ///< X acceleration (m/s^2)
        public float yacc; ///< Y acceleration (m/s^2)
        public float zacc; ///< Z acceleration (m/s^2)
        public float xgyro; ///< Angular speed around X axis (rad /sec)
        public float ygyro; ///< Angular speed around Y axis (rad /sec)
        public float zgyro; ///< Angular speed around Z axis (rad /sec)
        public float xmag; ///< 	X Magnetic field (Gauss)
        public float ymag; ///< 	Y Magnetic field (Gauss)
        public float zmag; ///< 	Z Magnetic field (Gauss)
        public float abs_press; ///< absolute pressure in millibar
        public float diff_press; ///< differential pressure in millibar
        public float press_alt; ///< altitude calculated from pressure
        public float temperature; ///< temperature in deg celcius
        public uint fields_updated; ///< bitmask for fields that have updated since last message, bit 0 = Xacc, bit 12 = temperature            
    } ;
    

    public struct mavlink_param_value_t
    {
        public object param_value; ///< Onboard parameter value
        public ushort param_count; ///< Total number of onboard parameters
        public ushort param_index; ///< Index of this onboard parameter
        public string param_id; ///< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        public MAV_PARAM_TYPE param_type; ///< Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    } ;

    public class MavlinkMessages
    {
        public const int MAVLINK_MSG_ID_HEARTBEAT               = 0;
        public const int MAVLINK_MSG_ID_SYSTEM_STATUS           = 1;
        public const int MAVLINK_MSG_ID_SYSTEM_TIME             = 2;
        public const int MAVLINK_MSG_ID_PARAM_VALUE             = 22;
        public const int MAVLINK_MSG_ID_SCALED_IMU              = 26;
        public const int MAVLINK_MSG_ID_SCALED_PRESSURE         = 29;
        public const int MAVLINK_MSG_ID_ATTITUDE                = 30;
        public const int MAVLINK_MSG_ID_GLOBAL_POSITION_INT     = 33;
        public const int MAVLINK_MSG_ID_HI_RES_IMU              = 105;
        public const int MAVLINK_MSG_ID_SCALED_IMU_2            = 116;
        //possibly include vibration (message id# 241)

        public const int MAVLINK_HEADER_SIZE = 6;
        public const int MAVLINK_CHECKSUM_SIZE = 2;

        public mavlink_global_position_int_t pos;
        public mavlink_heartbeat_t heartbeat;
        public mavlink_system_status_t system_stat;
        public mavlink_system_time_t system_time;
        public mavlink_scaled_pressure_t scaled_psi_temp;
        public mavlink_attitude_t attitude;
        public mavlink_scaled_imu_t scaled_imu;
        public mavlink_scaled_imu_2 scaled_imu_2;
        public mavlink_hi_res_imu hi_res_imu;
        public mavlink_param_value_t param_value;

        UdpClient mUdpClient;
        IPEndPoint mEndPoint;
        bool mRunning = true;
        Mutex parseMutex = new Mutex();

        //not in use
        public void CheckForInterrupt()
        {
                //blocks until keyhit:
                Console.ReadKey();
                Stop();
        }
        //delegate that is invoked when packet is received
        public static void AsyncRecv( System.IAsyncResult result )
        {
            MavlinkMessages localProgram = (MavlinkMessages)result.AsyncState;
            localProgram.parseMutex.WaitOne();
            if (result.IsCompleted)
            {
                byte[] buffer = localProgram.mUdpClient.EndReceive(result, ref localProgram.mEndPoint);
                //Debug.WriteLine("Size = " + buffer.Length);
                localProgram.parseMessage(ref buffer);
            }
            localProgram.parseMutex.ReleaseMutex();
        }
        
        //stop running and exit
        void Stop()
        {
            mRunning = false;
        }
        //example for how to break apart messages
        public void parseMessage(ref byte[] msg)
        {
            /*
            mavlink_global_position_int_t pos;
            mavlink_heartbeat_t heartbeat;
            mavlink_system_status_t system_stat;
            mavlink_system_time_t system_time;
            mavlink_scaled_pressure_t scaled_psi_temp;
            mavlink_attitude_t attitude;
            mavlink_scaled_imu_t scaled_imu;
            mavlink_param_value_t param_value;
            */


            //We need to be able to read at least the length of the payload
            for (int offset = 0; offset < msg.Length -1; )
            {
                //look for magic header
                if (msg[offset] == 0xfe)
                {
                    int msg_offset = MAVLINK_HEADER_SIZE; 
                    int len = msg[offset + 1]; //length of payload, not including header and checksum
                    //sanity check -- ditch this packet if it fails
                    if (offset + len + MAVLINK_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE > msg.Length)
                        break;
                   
                    int sequence = msg[offset + 2];
                    int msg_id = msg[offset + 5];
                    //I've done 2 message types as an example
                    //Look at 
                    switch (msg_id)
                    {
                        /*
                            public const int MAVLINK_MSG_ID_HEARTBEAT               = 0;
                            public const int MAVLINK_MSG_ID_SYSTEM_STATUS           = 1;
                            public const int MAVLINK_MSG_ID_SYSTEM_TIME             = 2;
                            public const int MAVLINK_MSG_ID_PARAM_VALUE             = 22;
                            public const int MAVLINK_MSG_ID_SCALED_IMU              = 26;
                            public const int MAVLINK_MSG_ID_SCALED_PRESSURE         = 29;
                            public const int MAVLINK_MSG_ID_ATTITUDE                = 30;
                            public const int MAVLINK_MSG_ID_GLOBAL_POSITION_INT     = 33;
                         * 
                            public struct mavlink_system_time_t
                            {
                                public uint time_unix_usec; ///< Timestamp of the master clock in microseconds since UNIX epoch.
                                public uint time_boot_ms; ///< Timestamp of the component clock since boot time in milliseconds.
                            } ;

                            public struct mavlink_scaled_pressure_t
                            {
                                public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
                                public float press_abs; ///< 	Absolute pressure (hectopascal)
                                public float press_diff; ///< 	Differential pressure 1 (hectopascal)
                                public uint temperature; ///< Temperature measurement (0.01 degrees celsius)
                            } ;

                            public struct mavlink_attitude_t
                            {
                                public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
                                public float roll; ///< 	Roll angle (rad, -pi..+pi)
                                public float pitch; ///< 	Pitch angle (rad, -pi..+pi)
                                public float yaw; ///< Yaw angle (rad, -pi..+pi)
                                public float rollspeed; ///< Roll angular speed (rad/s)
                                public float pitchspeed; ///< 	Pitch angular speed (rad/s)
                                public float yawspeed; ///< Yaw angular speed (rad/s)
                            } ;

                          
                        */

                        //This is an example of a "fixed" message
                        case MAVLINK_MSG_ID_ATTITUDE:
                            {
                                attitude.time_boot_ms = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                attitude.roll = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(float);  //maybe need to use toDouble instead
                                attitude.pitch = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(float);
                                attitude.yaw = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(float);
                                attitude.rollspeed = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(float);
                                attitude.pitchspeed = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(float);
                                attitude.yawspeed = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(float);
                                Console.WriteLine("MAVLINK_MSG_ID_ATTITUDE = " + attitude.time_boot_ms + "," + attitude.roll + "," + attitude.pitch + "," + attitude.yaw + "," + attitude.rollspeed + "," + attitude.pitchspeed + "," + attitude.yawspeed);
                            }
                            break;

                        //This is an example of a "fixed" message
                        case MAVLINK_MSG_ID_SCALED_IMU:
                            {
                                scaled_imu.time_boot_ms = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                scaled_imu.xacc = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.yacc = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.zacc = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.xgyro = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.ygyro = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.zgyro = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.xmag = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.ymag = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu.zmag = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                Console.WriteLine("MAVLINK_MSG_ID_SCALED_IMU = " + scaled_imu.time_boot_ms + "," + scaled_imu.xacc + "," + scaled_imu.yacc + "," + scaled_imu.zacc + "," + scaled_imu.xgyro + "," + scaled_imu.ygyro + "," + scaled_imu.zgyro + "," + scaled_imu.xmag + "," + scaled_imu.ymag + "," + scaled_imu.zmag);
                            }
                            break;

                        //This is an example of a "fixed" message
                        case MAVLINK_MSG_ID_SCALED_IMU_2:
                            {
                                scaled_imu_2.time_boot_ms = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                scaled_imu_2.xacc = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.yacc = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.zacc = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.xgyro = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.ygyro = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.zgyro = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.xmag = BitConverter.ToInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.ymag = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                scaled_imu_2.zmag = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                Console.WriteLine("MAVLINK_MSG_ID_SCALED_IMU_2 = " + scaled_imu.time_boot_ms + "," + scaled_imu.xacc + "," + scaled_imu.yacc + "," + scaled_imu.zacc + "," + scaled_imu.xgyro + "," + scaled_imu.ygyro + "," + scaled_imu.zgyro + "," + scaled_imu.xmag + "," + scaled_imu.ymag + "," + scaled_imu.zmag);
                            }
                            break;
                        
                            
                        //This is an example of a "fixed" message
                        case MAVLINK_MSG_ID_HI_RES_IMU:
                            {
                                hi_res_imu.time_boot_ms = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                hi_res_imu.xacc = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(float);
                                hi_res_imu.yacc = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.zacc = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.xgyro = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.ygyro = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.zgyro = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.xmag = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.ymag = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.zmag = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.abs_press = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.diff_press = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.press_alt = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.temperature = BitConverter.ToSingle(msg, offset + msg_offset); msg_offset += sizeof(short);
                                hi_res_imu.fields_updated = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(short);
                                Console.WriteLine("MAVLINK_MSG_ID_SCALED_IMU_2 = " + scaled_imu.time_boot_ms + "," + scaled_imu.xacc + "," + scaled_imu.yacc + "," + scaled_imu.zacc + "," + scaled_imu.xgyro + "," + scaled_imu.ygyro + "," + scaled_imu.zgyro + "," + scaled_imu.xmag + "," + scaled_imu.ymag + "," + scaled_imu.zmag);
                            }
                            break;


                        //This is an example of a "fixed" message
                        case MAVLINK_MSG_ID_HEARTBEAT:
                            {
                                //floating point values have to be scaled from the integer format they're transmitted as
                                msg_offset += sizeof(byte); //heartbeat.type  
                                msg_offset += sizeof(byte); //heartbeat.autopilot  
                                msg_offset += sizeof(byte); //heartbeat.base_mode  
                                msg_offset += sizeof(int); //heartbeat.custom_mode 
                                heartbeat.system_status = msg[offset + msg_offset]; msg_offset += sizeof(int);
                                heartbeat.previous_heartbeat_time = DateTime.UtcNow;
                                msg_offset += sizeof(byte); //heartbeat.base_mode  
                                Console.WriteLine("heartbeat.system_status = " + heartbeat.system_status);
                            }
                            break;





                            //This is an example of a "fixed" message
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                            {
                                //floating point values have to be scaled from the integer format they're transmitted as
                                //sorry, I don't know of an elegant way to calculate offsets in C# like I do in C++ :( 
                                pos.time_boot_ms = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof( int );
                                pos.lat = BitConverter.ToInt32(msg, offset + msg_offset)/(float)1e7; msg_offset += sizeof(int);
                                pos.lon = BitConverter.ToInt32(msg, offset + msg_offset)/(float)1e7; msg_offset += sizeof(int);
                                pos.alt = BitConverter.ToInt32(msg,offset + msg_offset)/(float)1000.0; msg_offset += sizeof(int);
                                pos.relative_alt = BitConverter.ToInt32( msg, offset + msg_offset)/(float)1000; msg_offset += sizeof(int);
                                pos.vx = BitConverter.ToInt16( msg, offset + msg_offset)/(float)100.0; msg_offset += sizeof( short );
                                pos.vy = BitConverter.ToInt16(msg, offset + msg_offset) / (float)100.0; msg_offset += sizeof(short);
                                pos.vz = BitConverter.ToInt16(msg, offset + msg_offset) / (float)100.0; msg_offset += sizeof(short);
                                pos.hdg = BitConverter.ToUInt16(msg, offset + msg_offset) / (float)100.0; msg_offset += sizeof(ushort);
                                Console.WriteLine(pos.time_boot_ms + "," + pos.lat + "," + pos.lon + "," + pos.alt);
                            }
                            break;

                            //This is an example of the more abstract "param-value" format
                        case MAVLINK_MSG_ID_PARAM_VALUE:
                            {
                                msg_offset += sizeof(float); //skip the param_value for now
                                param_value.param_count = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                param_value.param_index = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                param_value.param_id = System.Text.Encoding.ASCII.GetString(msg, offset + msg_offset, 16); msg_offset += 16;
                                param_value.param_type = (MAV_PARAM_TYPE)msg[offset + msg_offset];
                                switch (param_value.param_type)
                                {
                                        //I leave the other types as an exercise to the user:
                                    case MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32:
                                        param_value.param_value = BitConverter.ToInt32(msg, offset + MAVLINK_HEADER_SIZE);
                                        Console.WriteLine(param_value.param_id + " = " + param_value.param_value.ToString());
                                        break;
                                    case MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32:
                                        param_value.param_value = BitConverter.ToSingle(msg, offset + MAVLINK_HEADER_SIZE);
                                        Console.WriteLine(param_value.param_id + " = " + param_value.param_value.ToString());
                                        break;
                                }
                            }
                            break;
                    }
                    offset += len + 8; //header + checksum = 8
                }
                else
                    //ditch this packet -- not sure what we're reading
                    break;
            }
        }
        public void DoExample()
        {
            //When we get here we should first check to see if there is another instance of UdpClient that is already bound to port 14550


            mUdpClient = new UdpClient(14550);
            mEndPoint = new IPEndPoint(0, 14550); ///0 address translates as INADDR_ANY, for people with C++ background
            while( mRunning )
            {
                IAsyncResult result = mUdpClient.BeginReceive(AsyncRecv, this);
                //Always time out. Never block. 
                hresult.AsyncWaitHandle.WaitOne(1000);
             //   if (Console.KeyAvailable)
              //      break;
            }
        }
        //static void Main(string[] args)
        public static void Main(string[] args)
        {
            MavlinkMessages mavlinkExample = new MavlinkMessages();
            //You don't really need this because you have AsyncWaitHandle so there's a "pause" that you can use to check the console
            //but I've left it in if you do want thread later for other reasons
           // Thread interruptThread = new Thread(new ThreadStart(mavlinkExample.CheckForInterrupt));
          //  interruptThread.Start();
          //  while (!interruptThread.IsAlive) ;
            Console.WriteLine("Press any key to quit.");
            mavlinkExample.DoExample();
            Console.WriteLine("Program exit.");
        }
    }
}
