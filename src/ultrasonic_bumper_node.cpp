#include <ros/ros.h>
#include <serial/serial.h>
#include <stdlib.h>
#include <string>

#include <ros_ultrasonic_bumper/ultrasnd_bump_ranges.h>

using namespace std;


// >>>>> Global functions
void loadParams();
// <<<<< Global functions

// >>>>> Global variables
ros::NodeHandle* nh;
ros::NodeHandle* nhPriv;

string serial_port;
int baudrate;
int timeout_msec;

// <<<<< Global variables

#define DEFAULT_SER_PORT    "/dev/ttyUSB0"
#define DEFAULT_BAUDRATE    115200
#define DEFAULT_TIMEOUT     500

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ultrasonic_bumper_node");

    ROS_INFO_STREAM("-----------------------------------\r");
    ROS_INFO_STREAM("  Ultrasonic Bumper Board driver   \r");
    ROS_INFO_STREAM("-----------------------------------\r");

    nh = new ros::NodeHandle(); // Node
    nhPriv = new ros::NodeHandle( "~" ); // Private node to load parameters

    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) )
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    // Load parameters from param server
    loadParams();

    // >>>>> Serial driver
    try
    {
        serial::Serial serial;

        serial.setPort(serial_port);
        serial.open();

        if(serial.isOpen()){
            ROS_INFO_STREAM("Serial Port initialized: " << serial_port );
        }
        else
        {
            ROS_ERROR_STREAM( "Serial port not opened: " << serial_port );
            return EXIT_FAILURE;
        }

        serial.setBaudrate(baudrate);

        serial::Timeout to = serial::Timeout::simpleTimeout(timeout_msec);
        serial.setTimeout(to);
    }
    catch (serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to configure serial port " << serial_port << " - Error: "  << e.what() );
        return EXIT_FAILURE;
    }

    string ser_buffer;


    while( ros::ok() )
    {
        if( serial.waitReadable() )
        {
            int available = serial.available();
            ser_buffer += serial.read(available);
        }

        ros::spinOnce();
    }

    ROS_INFO_STREAM("... stopped!");

    return EXIT_SUCCESS;
}

void loadParams()
{
    ROS_INFO_STREAM( "Loading parameters from server" );

    if( !nhPriv->getParam( "serial_port", serial_port ) )
    {
        serial_port = DEFAULT_SER_PORT;
        nhPriv->setParam( "serial_port", serial_port );
        ROS_INFO_STREAM( "serial_port" << " not present. Default value set: " << serial_port );
    }
    else
        ROS_DEBUG_STREAM( "serial_port: " << serial_port );

    if( !nhPriv->getParam( "baudrate", baudrate ) )
    {
        baudrate = DEFAULT_BAUDRATE;
        nhPriv->setParam( "baudrate", baudrate );
        ROS_INFO_STREAM( "baudrate: " << " not present. Default value set: " << baudrate );
    }
    else
        ROS_DEBUG_STREAM( "baudrate: " << timeout_msec );

    if( !nhPriv->getParam( "timeout_msec", timeout_msec ) )
    {
        timeout_msec = DEFAULT_TIMEOUT;
        nhPriv->setParam( "timeout_msec", timeout_msec );
        ROS_INFO_STREAM( "timeout_msec" << " not present. Default value set: " << timeout_msec );
    }
    else
        ROS_DEBUG_STREAM( "timeout_msec " << timeout_msec );
}
