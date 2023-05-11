import rclpy
from rclpy.node import Node
import importlib

'''Instanceate as many publishers as required by network application'''
class Publishers(Node):
    
    def __init__(self, data: list):
        super.__init__('minimal_publisher')
        self.data = data
        self._instanceate_publishers()
        # data = [["type", "topic_name", "rate"]]
        # data = [["std_msgs/msg/String", "\hello", ]]

    def _instanceate_publishers(self) -> None:
        lenght_data = len(self.data)
        if (lenght_data ==1):                                #Type            #Topic name        #rate
            self.publisher1 = Node.create_publisher(self.data[0][0], self.data[0][1], int(self.data[0][2]))
        if (lenght_data ==2):
            self.publisher1 = Node.create_publisher(self.data[0][0], self.data[0][1], int(self.data[0][2]))
            self.publisher2 = Node.create_publisher(self.data[0][0], self.data[0][1], int(self.data[0][2]))

'''Instanceate as many subscribers as required by network application'''
class Subscriptions(Node):

    def __init__(self, data: list):
        super.__init__('minimal_publisher')
        self.data = data
        self._instanceate_subscriber()

    def _instanceate_subscriber(self) -> None:
        lenght_data = len(self.data)
        if (lenght_data ==1):
            self.subscription1 = Node.create_subscription(self.data[0][0], self.data[0][1], int(self.data[0][2]))
        if (lenght_data ==2):
            self.subscription1 = Node.create_subscription(self.data[0][0], self.data[0][1], int(self.data[0][2]))
            self.subscription2 = Node.create_subscription(self.data[0][0], self.data[0][1], int(self.data[0][2]))

class rosImports():

    def __init__(self, input_topics: dict, output_topics: dict):
        self.input_topics = input_topics
        self.output_topics = output_topics

    '''Import all ros message libraries required'''
    def load_all(self) -> None:
        def parser(topic_type:dict):
            for val in self.topic_type.values():
                pos = [i for i in range(len(val)) if val.startswith('/', i)]
                msg_type =val[:pos[0]]
                msg_msg =val[pos[0]+1:pos[1]]
                msg_name = val[pos[1]+1:]
                lib =  msg_type + '.' + msg_msg
                msg_name = importlib.import_module(lib)
        parser(self.input_topics)
        parser(self.output_topics)

    '''Import all ros message libraries required'''
    def load_library(self) -> None:
        
        if "std_msgs/msg/String" in self.input_topics.values() or "std_msgs/msg/String" in  self.output_topics.values():
            from std_msgs.msg import String

        if "geometry_msgs/msg/Twist" in self.input_topics.values() or "geometry_msgs/msg/Twist"  in self.input_topics.values() :
            from geometry_msgs.msg import Twist

        if "sensor_msgs/msg/LaserScan" in self.get_output_topics.values() or "sensor_msgs/msg/LaserScan" in self.get_output_topics.values():
            from sensor_msgs.msg import LaserScan
        
