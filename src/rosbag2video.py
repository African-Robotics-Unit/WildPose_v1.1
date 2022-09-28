#!/usr/bin/python3
import argparse
from pprint import pprint
import queue

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message


parser = argparse.ArgumentParser()
parser.add_argument('--input_db', type=str, required=True, help='The input rosbag file [db3].')
args = parser.parse_args()


class BagFileParser():
    # https://answers.ros.org/question/358686/how-to-read-a-bag-file-in-ros2/
    def __init__(self, bag_file):
        self.conn = sqlite3.connect(bag_file)
        self.cursor = self.conn.cursor()

        ## create a message type map
        topics_data = self.cursor.execute("SELECT id, name, type FROM topics").fetchall()
        self.topic_type = {name_of: type_of for id_of, name_of,type_of in topics_data}
        self.topic_id = {name_of: id_of for id_of,name_of, type_of in topics_data}
        self.topic_msg_message = {name_of: get_message(type_of) for id_of, name_of,type_of in topics_data}

    def __del__(self):
        self.conn.close()

    def get_messages(
        self,
        topic_name: str,
        limit: int = None,
    ):
        '''
        Return [(timestamp0, message0), (timestamp1, message1), ...]
        '''
        topic_id = self.topic_id[topic_name]

        query = f"SELECT timestamp, data FROM messages WHERE topic_id = {topic_id} "
        if limit is not None:
            query += f"LIMIT {limit} "

        rows = self.cursor.execute(query).fetchall()
        # Deserialise all and timestamp them
        return [(timestamp, deserialize_message(data, self.topic_msg_message[topic_name])) for timestamp,data in rows]



if __name__ == "__main__":
    parser = BagFileParser(args.input_db)
    pprint(parser.topic_type)
    pprint(parser.topic_id)
    pprint(parser.topic_msg_message)

    image_msgs = parser.get_messages('/image_raw', limit=1)
    print(image_msgs[0][0])
    print(type(image_msgs[0][1]))   # sensor_msgs.msg._image.Image
