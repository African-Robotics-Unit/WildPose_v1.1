#!/usr/bin/python3
import sys
import argparse
from pprint import pprint
import queue
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import shlex, subprocess

import sqlite3
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message

MJPEG_VIDEO = 1
RAWIMAGE_VIDEO = 2
VIDEO_CONVERTER_TO_USE = "ffmpeg"   # or you may want to use "avconv"


parser = argparse.ArgumentParser()
parser.add_argument('--input_db', type=str, required=True, help='The input rosbag file [db3].')
parser.add_argument('--topic', type=str, default='image_raw', help='Only the images from topic "topic" are used for the video output.')
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


class RosVideoWriter():
    def __init__(self, fps=25.0, rate=1.0, topic="", output_filename ="", verbose=False):
        self.opt_topic = topic
        self.opt_out_file = output_filename
        self.opt_verbose = verbose
        self.rate = rate
        self.fps = fps
        self.opt_prefix = None
        self.t_first = {}
        self.t_file = {}
        self.t_video = {}
        self.p_avconv = {}

    def write_output_video(self, msg, topic, t, video_fmt, pix_fmt = ""):
        # no data in this topic
        if len(msg.data) == 0:
            return
        # initiate data for this topic
        if not topic in self.t_first:
            self.t_first[topic] = t # timestamp of first image for this topic
            self.t_video[topic] = 0
            self.t_file[topic] = 0
        # if multiple streams of images will start at different times the resulting video files will not be in sync
        # current offset time we are in the bag file
        self.t_file[topic] = (t - self.t_first[topic]).to_sec()
        # fill video file up with images until we reache the current offset from the beginning of the bag file
        while self.t_video[topic] < self.t_file[topic]/self.rate:
            if not topic in self.p_avconv:
                # we have to start a new process for this topic
                if self.opt_verbose:
                    print("Initializing pipe for topic", topic, "at time", t.to_sec())
                if self.opt_out_file=="":
                    out_file = self.opt_prefix + str(topic).replace("/", "_")+".mp4"
                else:
                    out_file = self.opt_out_file

                if self.opt_verbose:
                    print("Using output file ", out_file, " for topic ", topic, ".")

                if video_fmt == MJPEG_VIDEO:
                    cmd = [VIDEO_CONVERTER_TO_USE, '-v', '1', '-stats', '-r',str(self.fps),'-c','mjpeg','-f','mjpeg','-i','-','-an',out_file]
                    self.p_avconv[topic] = subprocess.Popen(cmd, stdin=subprocess.PIPE)
                    if self.opt_verbose:
                        print("Using command line:")
                        print(cmd)
                elif video_fmt == RAWIMAGE_VIDEO:
                    size = str(msg.width)+"x"+str(msg.height)
                    cmd = [VIDEO_CONVERTER_TO_USE, '-v', '1', '-stats','-r',str(self.fps),'-f','rawvideo','-s',size,'-pix_fmt', pix_fmt,'-i','-','-an',out_file]
                    self.p_avconv[topic] = subprocess.Popen(cmd, stdin=subprocess.PIPE)
                    if self.opt_verbose:
                        print("Using command line:")
                        print(cmd)

                else:
                    print("Script error, unknown value for argument video_fmt in function write_output_video.")
                    exit(1)
            # send data to ffmpeg process pipe
            self.p_avconv[topic].stdin.write(msg.data)
            # next frame time
            self.t_video[topic] += 1.0 / self.fps

    def addBag(self, filename: str):
        if self.opt_verbose:
            print("Bagfile: {}".format(filename))

        if not self.opt_prefix:
            # create the output in the same folder and name as the bag file minu '.bag'
            self.opt_prefix = bagfile[:-4]

        # Go through the bag file
        bag = BagFileParser(filename)
        if self.opt_verbose:
            print("Bag opened.")
        # loop over all topics
        topic = args.topic
        for _, msg in bag.get_messages(topic, limit=3):
            timestamp = img.header.stamp
            t = int(str(timestamp.sec) + str(timestamp.nanosec))

            try:
                if msg.format.find("jpeg")!=-1:
                    if msg.format.find("8")!=-1 and (msg.format.find("rgb")!=-1 or msg.format.find("bgr")!=-1 or msg.format.find("bgra")!=-1):
                        self.write_output_video(msg, topic, t, MJPEG_VIDEO)
                    elif msg.format.find("mono8")!=-1:
                        self.write_output_video(msg, topic, t, MJPEG_VIDEO)
                    elif msg.format.find("16UC1")!=-1:
                        self.write_output_video(msg, topic, t, MJPEG_VIDEO)
                    else:
                        print('unsupported jpeg format:', msg.format, '.', topic)
            # has no attribute 'format'
            except AttributeError:
                try:
                    pix_fmt=None
                    if msg.encoding.find("mono8")!=-1 or msg.encoding.find("8UC1")!=-1:
                        pix_fmt = "gray"
                    elif msg.encoding.find("bgra")!=-1:
                        pix_fmt = "bgra"
                    elif msg.encoding.find("bgr8")!=-1:
                        pix_fmt = "bgr24"
                    elif msg.encoding.find("bggr8")!=-1:
                        pix_fmt = "bayer_bggr8"
                    elif msg.encoding.find("rggb8")!=-1:
                        pix_fmt = "bayer_rggb8"
                    elif msg.encoding.find("rgb8")!=-1:
                        pix_fmt = "rgb24"
                    elif msg.encoding.find("16UC1")!=-1:
                        pix_fmt = "gray16le"
                    else:
                        print('unsupported encoding:', msg.encoding, topic)
                        #exit(1)
                    if pix_fmt:
                        self.write_output_video(msg, topic, t, RAWIMAGE_VIDEO, pix_fmt)

                except AttributeError:
                    # maybe theora packet
                    # theora not supported
                    if self.opt_verbose:
                        print("Could not handle this format. Maybe thoera packet? theora is not supported.")
                    pass
        if self.p_avconv == {}:
            print("No image topics found in bag:", filename)
        bag.close()




if __name__ == "__main__":
    bagfile = BagFileParser(args.input_db)

    image_msgs = bagfile.get_messages('/image_raw', limit=1)

    img = image_msgs[0][1]  # sensor_msgs.msg._image.Image
    timestamp = img.header.stamp
    print(type(image_msgs[0][0]), image_msgs[0][0])
    print(type(timestamp.sec), timestamp)
    print(img.encoding)



    # videowriter = RosVideoWriter()
    # videowriter.addBag(args.input_db)
