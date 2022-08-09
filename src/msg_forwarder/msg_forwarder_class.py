from tkinter import E
import rospy
import roslibpy
import yaml
from importlib import import_module
from rospy_message_converter import message_converter


class MsgForwarder:

    def __init__(self, topics_file, rosbridge_host ='127.0.0.1', rosbridge_port = 9090):

        self.remote_topics, self.local_topics = self.extract_topics(topics_file)
        self.rosbridge = self.connect_rosbridge(rosbridge_host, rosbridge_port)

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
                self.__extract_message_type_class(ros_topic_type),
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
                self.__extract_message_type_class(ros_topic_type),
                self.__create_callback_from_local(publisher)
            )

            rospy.loginfo(f'Forwarding messages from local topic {ros_topic_name} to remote ROS instance')

    def __create_callback_from_remote(self, type, publisher):
        def callback(message):
            publisher.publish(message_converter.convert_dictionary_to_ros_message(type, message))
        return callback

    def __create_callback_from_local(self, publisher):
        def callback(message):
            publisher.publish(roslibpy.Message(message_converter.convert_ros_message_to_dictionary(message)))
        return callback

    def __extract_message_type_class(self, type):
        package, cls = type.split('/')
        try:
            module = import_module(f'{package}.msg')
            return getattr(module, cls)
        except Exception as e:
            raise e

    def extract_topics(self, topics_file):
        try:

            # Parse YAML file
            with open(topics_file, 'r') as file:
                try:
                    yaml_data = yaml.safe_load(file)
                except yaml.YAMLError as e:
                    raise e

            # Extract only relevant fields
            remote_topics = yaml_data.get('remote_topics') or []
            local_topics = yaml_data.get('local_topics') or []
            if not remote_topics and not local_topics:
                raise Exception("Provided YAML file contains no useful topic information.")

            return remote_topics, local_topics

        except FileNotFoundError as e:
            raise e

    def connect_rosbridge(self, rosbridge_host, rosbridge_port):
        rosbridge = roslibpy.Ros(host=rosbridge_host, port=rosbridge_port)
        rosbridge.run()
        return rosbridge
