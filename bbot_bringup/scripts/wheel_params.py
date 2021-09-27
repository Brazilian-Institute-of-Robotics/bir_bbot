import rospy
from dynamixel_sdk import *
from std_msgs.msg import Int32MultiArray

WHEEL_ADDR_VELOCITY_KP	=   78
WHEEL_ADDR_VELOCITY_KI	=   76

PROTOCOL_VERSION            = 2.0               # See which protocol version is used in the Dynamixel
BAUDRATE                    = 1000000            # Dynamixel default baudrate : 57600
DEVICENAME                  = '/dev/ttyUSB0'    # Check which port is being used on your controller

portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

def callback(data):
    #SET KI
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 12, WHEEL_ADDR_VELOCITY_KI, data.data[1])
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 22, WHEEL_ADDR_VELOCITY_KI, data.data[1])
    
    #SET KP
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 12, WHEEL_ADDR_VELOCITY_KP, data.data[0])
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, 22, WHEEL_ADDR_VELOCITY_KP, data.data[0])

def main():
    rospy.init_node("wheel_params")
    
    sub = rospy.Subscriber("wheel_params", Int32MultiArray,callback)
    
    rospy.spin()

if __name__ == '__main__':
    main()
    