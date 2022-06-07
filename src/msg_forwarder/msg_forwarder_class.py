import rospy
import roslibpy
import yaml
from rospy_message_converter import message_converter

class MsgForwarder(object):

    def __init__(self, topics_file, rosbridge_host ='127.0.0.1', rosbridge_port = 9090):

        self.rosbridge_host = rosbridge_host
        self.rosbridge_port = rosbridge_port

        self.remote_topics, self.local_topics = self.extract_topics(topics_file)
        self.rosbridge = self.connect_rosbridge()

        # Forward remote topics
        # - Subscribe remote
        # - Publish local
        for topic_data in self.remote_topics:
            try:
                ros_topic_name = topic_data['topic']
                ros_topic_type = topic_data['type']
            except KeyError as e:
                raise e

            publisher = rospy.Publisher(
                ros_topic_name,
                ros_topic_type,
                queue_size=10
            )
            subscriber = roslibpy.Topic(
                self.rosbridge,
                ros_topic_name,
                ros_topic_type
            )
            subscriber.subscribe(
                self.__create_callback_from_remote(ros_topic_type, publisher)
            )

            rospy.loginfo(f'Forwarding messages from remote topic {ros_topic_name} to local ROS instance')

        # Forward local topics
        # - Subscribe local
        # - Publish remote
        for topic_data in self.local_topics:
            try:
                ros_topic_name = topic_data['topic']
                ros_topic_type = topic_data['type']
            except KeyError as e:
                raise e

            publisher = roslibpy.Topic(
                self.rosbridge,
                ros_topic_name,
                ros_topic_type
            )
            subscriber = rospy.Subscriber(
                ros_topic_name,
                ros_topic_type,
                self.__create_callback_from_local(publisher)
            )

            rospy.loginfo(f'Forwarding messages from local topic {ros_topic_name} to remote ROS instance')

    def __create_callback_from_remote(type, publisher):
        def callback(message):
            publisher.publish(message_converter.convert_dictionary_to_ros_message(type, message))
        return callback

    def __create_callback_from_local(publisher):
        def callback(message):
            publisher.publish(roslibpy.Message(message_converter.convert_ros_message_to_dictionary(message)))
        return callback

    def extract_topics(self, topics_file):
        try:

            # Parse YAML file
            with open(topics_file, 'r') as file:
                try:
                    yaml_data = yaml.safe_load(file)
                except yaml.YAMLError as e:
                    raise e

            # Extract only relevant fields
            try:
                return yaml_data['remote_topics'], yaml_data['local_topics']
            except KeyError as e:
                raise e

        except IOError as e:
            raise e

    def connect_rosbridge(self, rosbridge_host, rosbridge_port):
        rosbridge = roslibpy.Ros(host=rosbridge_host, port=rosbridge_port)
        rosbridge.run()
        return rosbridge
