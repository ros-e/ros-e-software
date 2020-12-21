#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Finds the Microphone as audio input device and processes the audio data.
Publishes volume spikes to sound/spike

@author Johannes Sommerfeldt
"""

# ROS 2 Imports
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool, Float64
from sound.msg import Spike


import pyaudio, numpy
from collections import deque
import time


class CompareMode():
    """ Pseudo Enum class to select a way of comparing volume values. """
    AVERAGE = 1
    MINIMUM = 2
    # maybe add better alternative here, like average of the smaller half of values


# processing preferences:
BUFFER_SIZE = 1024                # size of processed audio chunks. bigger buffer means the volume is calculated less often, but over more samples
RMS_HISTORY_SECONDS = 5.0         # how many seconds to gather rms values for comparison over
COMPARE_TO = CompareMode.AVERAGE  # what to calculate from the rms history as reference for spike detection
SPIKE_THRESHOLD = 20              # how many dB the rms must be above the history's values to count as spike (depends on the comparison mode)

# microphone array:
DEVICE_NAME = "ReSpeaker 4 Mic Array"
CHANNEL_TO_MONITOR = 0  # 0 is the onboard-processed audio, recommended. see https://wiki.seeedstudio.com/ReSpeaker_Mic_Array_v2.0/#update-firmware


class MicrophoneNode(Node):
  
    def __init__(self):
        
        super().__init__('microphone_node')

        """ Constructor for this node. Sets up all values needed to process audio and starts a stream on the microphone array."""
        rclpy.get_default_context().on_shutdown(self.onShutdown)

        # define and announce publisher topic for ROS 
        # self.spike_publisher = self.create_publisher(Bool, 'sound/spike', 10)        # old
        self.spike_publisher = self.create_publisher(Spike, 'sound/spike', 10)         # new

        self.pyaudio_instance = pyaudio.PyAudio()

        # find the audio device to find out its index, sample rate and channel number
        self.device_index, self.sample_rate, self.channels = self.findDevice(DEVICE_NAME)

        # convenience variables
        self.buffers_per_second = float(self.sample_rate) / BUFFER_SIZE
        self.seconds_per_buffer = float(BUFFER_SIZE) / self.sample_rate
        
        # Queue that stores the rms_log values of the specified amount of seconds, dropping older values when full
        self.rms_log_history = deque(maxlen=int(RMS_HISTORY_SECONDS * self.buffers_per_second))

        # bool to check whether the spike value changed
        self.prev_is_spike = False

        # open stream
        self.stream = self.pyaudio_instance.open(
                format=pyaudio.paInt16, 
                channels=self.channels, 
                rate=self.sample_rate, 
                input=True, 
                frames_per_buffer=BUFFER_SIZE,
                input_device_index=self.device_index,
                stream_callback=self.callback)

        # variables to calculate timestamps
        self.starting_time = self.get_clock().now()
        self.current_cycle = 0

        self.get_logger().info("Microphone Node started")

    def findDevice(self, device_name):
        """ Helper method to handle finding the microphone array parameters to not clutter the constructor. \n
        device_name: Part of the name the desired device is identified by. \n
        returns: Tuple of the device's device index, its sample rate and its number of channels. \n
        raises: ValueError, if no device with a name that contains the specified string was found.
        """
        for i in range(self.pyaudio_instance.get_device_count()):
            device_info = self.pyaudio_instance.get_device_info_by_index(i)
            name = device_info['name'].encode('utf-8')

            if name.find(device_name) >= 0:
                device_index = i
                # Get the sample rate and channel count from the device info and not from constants since they can be changed with the firmware.
                sample_rate = int(device_info['defaultSampleRate'])
                channels = device_info['maxInputChannels']

                print('Found {} with {} max input channels and {} default sample rate at index {}'
                        .format(name, channels, sample_rate, device_index))
                return (device_index, sample_rate, channels)

        raise ValueError('Cannot find any input device with the name {}'.format(device_name))


    def callback(self, in_data, frame_count, time_info, status_flags):
        """ This method is called whenever one buffer full of audiodata is ready in the stream. """
        data = numpy.fromstring(in_data, dtype=numpy.int16)[CHANNEL_TO_MONITOR::self.channels]

        # calculate the time when the audio of this buffer was actually recorded, since this method is called delayed
        buffer_start_time = self.starting_time + self.current_cycle * self.seconds_per_buffer
        self.current_cycle += 1

        # self.get_logger().info("now: " + str(self.get_clock().now()) + "\t calculated: " + str(buffer_start_time))

        # absolute amplitude peak of the chunk
        #peak = numpy.max(numpy.absolute(data))

        # quadratic mean value (average volume) of the chunk. int32 so the square doesn't clip
        rms = numpy.sqrt(numpy.mean(numpy.square(data.astype('int32'))))
        rms_log = 20 * numpy.log10(rms)

        if self.rms_log_history:
            # use the newest rms_log values as reference for spike detection
            if COMPARE_TO == CompareMode.AVERAGE:
                rms_log_reference = numpy.mean(self.rms_log_history)

            elif COMPARE_TO == CompareMode.MINIMUM:
                rms_log_reference = numpy.min(self.rms_log_history)

            # determine, whether the new rms is a spike in volume
            is_spike = rms_log > rms_log_reference + SPIKE_THRESHOLD

            # publish spike changes
            if (is_spike != self.prev_is_spike): 
                self.get_logger().info("Spike changed to: " + str(is_spike))
                # self.spike_publisher.publish(Bool(data=is_spike))                                         # old
                self.spike_publisher.publish(Spike(spike_status=is_spike, timestamp=buffer_start_time))     # new

                self.prev_is_spike = is_spike

                # self.get_logger().info("buffer start time: {} \t now: {}".format(buffer_start_time, self.get_clock().now()))

        # store the newest rms_log values
        self.rms_log_history.append(rms_log)


        ### ONLY USE FOR DEBUGGING, FREQUENTLY CALLING self.get_logger().info() USES HIGH % OF CPU
        """
        # assemble info to log
        info = ""
        info += "peak: " + str(peak) + "\t"
        #info += "rms: " + str(int(rms)) + "  "
        info += "rms_log: " + str(int(rms_log)) + "  "
        info += "logDiff: " + str(int(rms_log - rms_log_reference)) + "  "
        info += "\n"
        info += "Spike: " + str(is_spike) + "\t"
        # one symbol for every couple of dB as a simple visual volume meter
        bars = "#" * int(0.5 * rms_log)
        info += bars
        self.get_logger().info(info)
        """

        return (None, pyaudio.paContinue)


    def onShutdown(self):
        try:
            self.stream.stop_stream()
            self.stream.close()
            self.pyaudio_instance.terminate
        except:
            raise
        finally:
            self.stream = None
            self.pyaudio_instance = None
 

def main(args=None):
    rclpy.init(args=args) # 'microphone_node'

    node = MicrophoneNode()

    rclpy.spin(node)
    node.onShutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
  main()