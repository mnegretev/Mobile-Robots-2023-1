#!/usr/bin/python

from time import sleep
import pyaudio
import rospy
from std_msgs.msg import String

def main():
    print("INITIALIZING AUDIO SENDER NODE BY MARCOSOFT...")
    rospy.init_node("audio_sender")
    pubSender = rospy.Publisher("/sphinx_audio", String, queue_size=10)
    #print("AudioSender.->Waiting 5 seconds just because I can...")
    #sleep(5)
    stream = pyaudio.PyAudio().open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=1024)
    stream.start_stream()
    while not rospy.is_shutdown():
        buf = stream.read(1024)
        if buf:
            pubSender.publish(buf)


if __name__ == "__main__":
    main()
