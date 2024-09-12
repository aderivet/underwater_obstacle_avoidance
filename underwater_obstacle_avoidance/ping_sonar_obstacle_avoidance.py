#!/usr/bin/env python3

# ping_sonar_obstacle_avoidance.py
import time
import math
import argparse
import csv
import brping
from brping import Ping1D


"""
Sources:
    * Repo: https://github.com/bluerobotics/pingpython/blob/master/examples/simplePingExample.py
    * api: https://docs.bluerobotics.com/pingpython/classbrping_1_1ping1d_1_1Ping1D.html
1. Connect to ping
1.1 Set devices mode to auto, change gain setting, range, ping rate
1.2 (optional) set speed of sound: 343 m/s in air,  1481 m/s in water, 1485000 mm/s in saltwater, 1475000 mm/s in fresh water
2. Get profile, distance, range, confidence
3. Convert distances from mm to m
4. Get time delta (dt) to calculate object velocity
5. (optional) add a filter to reduce noise. NOTE: DON'T BOTHER FOR NOW AS THE CONFIDENCE AND DISTANCES DO NOT NECESSARILY REFER TO ONE OBJECT ONLY
6. Filter/remove confidence below threshold (using a low pass filter). todo: Annette
7. Ignore measurements <= min distance and >= max distance
8. Save to CSV (https://discuss.bluerobotics.com/t/retrieve-ping-sonar-data-for-analysis/11795/2)
9. (optional) Plot results

Notes:
    * if auto mode does not work, switch to manual and adjust gains

In-person tests:
    * point it without facing any obstacles
    
Todo:
    * check (https://discuss.bluerobotics.com/t/interpreting-profile-data-in-ping-python/17272/8)
    * check the difference between ping numbers, e.g if current ping number == prev ping number, skip
    * limit the range to sensible values
    * adujust the speed of sound
    * adjust the gain
    * adjust the ping interval
"""

MM_TO_M = 0.001


class PingSonarObstacleAvoidance:
    def __init__(self, baudrate=115200, ping_port='COM3', udp_address=None, device_type='Ping1D',         # changed ping_port to COM3 -> windows
                 poll_rate=20.0, min_distance=1.0, max_distance=20.0, speed_of_sound=343.0,
                 min_confidence=30, gain=3, ping_interval=150, mode_auto=False, fov=30.0, csv_path=""
):
        """
        Input units are in meters but converted to millimeters for use with the device and returned values in meters
        :param baudrate:
        :param ping_port:
        :param udp_address:
        :param device_type:
        :param poll_rate:
        :param min_distance:
        :param max_distance:
        :param speed_of_sound:
        :param min_confidence:
        :param gain:
        :param ping_interval:
        :param mode_auto:
        :param fov:
        """
        self.baudrate = baudrate
        self.ping_port = ping_port  # /dev/ttyUSB0 for Linux and COM3 for Windows
        self.udp_address = udp_address
        self.device_type = device_type
        self.poll_rate = poll_rate  # call time.sleep(1 / poll_rate) in loop
        self.min_distance = min_distance  # in m
        self.max_distance = max_distance  # in m
        self.scan_start = min_distance / MM_TO_M  # min_distance given in meters, scan_start in millimeters
        self.scan_range = (self.max_distance / MM_TO_M) - self.scan_start  # scan_range in millimeters
        self.speed_of_sound = speed_of_sound  # in m/s. 343 m/s in air and 1481 m/s in water
        self.speed_of_sound_mms = self.speed_of_sound * MM_TO_M  # in mm/s
        self.min_confidence = min_confidence
        self.gain = gain
        self.ping_interval = ping_interval
        self.mode_auto = bool(mode_auto)
        self.fov = math.radians(fov)  # field of view in degrees

        # todo: @Elias: what are the values for minimum and maximum range, rates, lengths, etc?
        if self.poll_rate <= 5.0:
            self.poll_rate = 5.0
        elif self.poll_rate >= 30.0:
            self.poll_rate = 30.0

        self.ping = None
        if self.device_type == 'Ping1D':
            self.ping = Ping1D()
            if self.ping_port is not None:
                self.ping.connect_serial(self.ping_port, self.baudrate)
            elif self.udp_address is not None:
                (host, port) = self.udp_address.split(':')
                self.ping.connect_udp(host, int(port))

            if self.ping.initialize() is False:
                raise RuntimeError('Could not initialize Ping1D')

        self.gain_dict = {0: 0.6, 1: 1.8, 2: 5.5, 3: 12.9, 4: 30.2, 5: 66.1, 6: 144}

        if self.gain_dict.get(self.gain) is None:
            raise RuntimeError('Invalid gain setting')

        self.set_device_properties()
        self.csv_file = None
        if len(csv_path) > 0:
            self.csv_file = open(csv_path, 'w', newline='')
            self.csv_writer = csv.writer(self.csv_file, delimiter=',')
            self.write_to_csv([
                'time', 'distance', 'confidence', 'transmit_duration', 'ping_number', 'max_range',
                'gain_setting', 'profile_data', 'speed_of_sound',
                'firmware_version_major', 'firmware_version_minor', 'ping_interval', 'mode_auto'])

    def set_device_properties(self):
        self.ping.set_speed_of_sound(int(self.speed_of_sound))
        self.ping.set_range(int(self.scan_start), int(self.scan_range))
        self.ping.set_ping_interval(int(self.ping_interval))
        self.ping.set_gain_setting(int(self.gain))
        self.ping.set_mode_auto(int(self.mode_auto))

    def get_distance(self):
        # data = self.ping.get_distance()
        data = self.ping.get_profile()
        return data

    def write_to_csv(self, csv_row):
        if self.csv_file is not None:
            self.csv_writer.writerow(csv_row)

    def differentiate_distance(self, distance_delta, dt):
        pass

    def range_callback(self):
        current_time = time.time()
        data = self.get_distance()

            # Initialize variables with default values
        distance = None
        confidence = None
        transmit_duration = None
        ping_number = None
        scan_start = None
        scan_length = None
        max_range = None
        gain_setting = None
        profile_data = None
        speed_of_sound = None
        firmware_version_major = None
        firmware_version_minor = None
        ping_interval = None
        mode_auto = None

        if data:
            # Units: mm. The current return distance determined for the most recent acoustic measurement converted to m
            distance = data.get('distance', 0) * MM_TO_M  # in m
            # Units: %. Confidence in the most recent range measurement.
            confidence = data.get('confidence', 0)
            # Units: us. The acoustic pulse length during acoustic transmission/activation.
            transmit_duration = data.get('transmit_duration', 0)
            # The pulse/measurement count since boot.
            ping_number = data.get('ping_number', 0)


            # note that running self.ping.get_range() gives scan_start and scan_length
            # Units: mm; The beginning of the scan region in mm from the transducer.
            scan_start = data.get('scan_start', 0)* MM_TO_M
            # Units: mm; The length of the scan region.
            scan_length = data.get('scan_length', 0) * MM_TO_M

            max_range = scan_start + scan_length
            # The current gain setting. 0: 0.6, 1: 1.8, 2: 5.5, 3: 12.9, 4: 30.2, 5: 66.1, 6: 144
            gain_setting = data.get('gain_setting', 0)  # equivalent to self.ping.get_gain_setting()

            profile_data = data.get('profile_data', 0)
            if confidence >= 30:
               print(f"Distance: {distance} meters, Confidence: {confidence}, Transmit Duration: {transmit_duration}")

        else:
            print("Failed to get data")

        speed_of_sound_info = self.ping.get_speed_of_sound()
        if speed_of_sound:
            speed_of_sound = speed_of_sound_info.get("speed_of_sound", 343.0) * MM_TO_M

        general_info = self.ping.get_general_info()
        if general_info:
            firmware_version_major = general_info.get("firmware_version_major", 0)
            firmware_version_minor = general_info.get("firmware_version_minor", 0)
            ping_interval = general_info.get("ping_interval", 0)  #  Units: ms; The interval between acoustic measurements.
            gain_setting = general_info.get("gain_setting", 0)
            mode_auto = general_info.get("mode_auto", 0)  # 0: manual, 1: auto

        if confidence >= 30:
            self.write_to_csv([
                current_time, distance, confidence, transmit_duration, ping_number,
                max_range, gain_setting, profile_data, speed_of_sound, firmware_version_major,
                firmware_version_minor, ping_interval, mode_auto
            ])
        time.sleep(1 / self.poll_rate)
        return data, general_info, speed_of_sound


if __name__ == '__main__':
    psoa = PingSonarObstacleAvoidance(csv_path='Trial13.csv')
    while True:
        psoa.range_callback()
