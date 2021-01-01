
import redis
import json

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Int32, Int16, Int8, Float32, Float64, UInt32, UInt16, UInt8, ColorRGBA
from visual.msg import DisplayPixel, DisplayConnectedPixel, Point, LED, LEDs
from systemcore.msg import I2Cwrite8, I2Cwrite16, I2CwriteArray
from head.msg import MotorPosition
from sound.msg import Spike

import copy
import time

class MessageClass():

    typeDict = {}

    def __init__(self, name, package, objectClass):
        self.name = name
        self.package = package
        self.objectClass = objectClass
        self.instance = objectClass.__class__

        MessageClass.typeDict[name] = self
        MessageClass.typeDict[package + '/' + name] = self
        MessageClass.typeDict[name.lower()] = self


class MessageType():
    """ Class containing all message types to provide generic publisher and message generation """

    String = MessageClass("String", "std_msgs", String)
    Bool = MessageClass("Bool", "std_msgs", Bool)

    Float32 = MessageClass("Float32", "std_msgs", Float32)
    Float64 = MessageClass("Float64", "std_msgs", Float64)

    Int8 = MessageClass("Int8", "std_msgs", Int8)
    Int16 = MessageClass("Int16", "std_msgs", Int16)
    Int32 = MessageClass("Int32", "std_msgs", Int32)    

    UInt8 = MessageClass("UInt8", "std_msgs", UInt8)
    UInt16 = MessageClass("UInt16", "std_msgs", UInt16)
    UInt32 = MessageClass("UInt32", "std_msgs", UInt32)

    MotorPosition = MessageClass("MotorPosition", "head", MotorPosition)

    DisplayPixel = MessageClass("DisplayPixel", "visual", DisplayPixel)
    DisplayConnectedPixel = MessageClass("DisplayConnectedPixel", " visual", DisplayConnectedPixel)
    Point = MessageClass("Point", " visual", Point)

    LED = MessageClass("LED", " visual", LED)
    LEDs = MessageClass("LEDs", " visual", LEDs)


    I2Cwrite8 = MessageClass("I2Cwrite8", "system", I2Cwrite8)
    I2Cwrite16 = MessageClass("I2Cwrite16", "system", I2Cwrite16)
    I2CwriteArray = MessageClass("I2CwriteArray", "system", I2CwriteArray)

    node = None

    @staticmethod
    def getPublisher(node, topic, messageType, queue_size=10):
        """ Creates a ROS-Publisher for the given topic and the message type """

        if messageType in MessageClass.typeDict:
            return node.create_publisher(MessageClass.typeDict[messageType].objectClass, str(topic), queue_size)

        raise TypeError("The given message type could not be handled by the bridge." , messageType)

    @staticmethod
    def createRosObject(messageType, value):
        """ Creates a ROS-object with the given type and the JSON-value. 
        If the JSON-value is primitive (so no object), the messageType should be primitive too (e.g. Int16) and this method will try to assign the value to the 'data' field of this object """

        jsonValue = None

        try:

            array = False
            if messageType.endswith("[]"):
                array = True
                messageType = messageType[:-2]

            # Get the object type and create a new instance
            mType = MessageClass.typeDict[messageType]
            # obj = mType.instance()
            obj = mType.objectClass()
            

            # Read all members to be filled from the value parameter
            objDir = dir(obj)
            # print(objDir)

            members = [attr for attr in objDir if not attr.startswith("__") and not attr.startswith("_") and not callable(getattr(obj, attr))]

            # print(value)
            # Parse the value string to json object
            # jsonValue = json.loads(value)
            # print(jsonValue)
            jsonValue = value

            # print("JsonValue = {}".format(jsonValue))

            if array:
                if type(jsonValue) == list:

                    # MessageType.node.get_logger().info("######## Got array ##########")

                    values = []
                    for v in jsonValue:
                        values.append(MessageType.createRosObject(messageType, v))

                    # MessageType.node.get_logger().info(f"######## Array: ${values} ##########")


                    return values

                else:
                    raise TypeError(f"The given message type does not fit the data" , type(jsonValue))


            else:
                # If JSON value is a complex object (thus a dict), search every member and set the attribute by recursive call
                if type(jsonValue) == dict:
                    for member in members:
                        if member in jsonValue:
                            val = jsonValue[member]
                            # print("CompType")
                            # print("Set " + member)
                            createdObject = copy.deepcopy(MessageType.createRosObject(val["type"], val["value"]))
                            # print("Set " + member + "to " + str(createdObject) + ": " + str(jsonValue) + " || " + str(type(obj)) + str(obj))
                            setattr(obj, member, createdObject)
                            

                # If JSON value is a primitive type, fill the 'data' field of the primitive object with the value
                else:
                    # print("PrimType")
                    return jsonValue
                    # obj = json
                    # setattr(obj, "data", jsonValue)
                    # print("Set data " + str(jsonValue) + " | " + str(type(jsonValue)) + " || " + str(type(obj)) + str(obj))


            


            return obj                

        
        except Exception as e:
            # print(str(e))
            print(jsonValue)
            print(type(jsonValue))
            raise e

        
