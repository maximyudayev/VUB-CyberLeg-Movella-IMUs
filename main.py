############
#
# Copyright (c) 2024 Maxim Yudayev and KU Leuven eMedia Lab
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#
# Created 2024-2025 for the KU Leuven AidWear, AidFOG, and RevalExo projects
# by Maxim Yudayev [https://yudayev.com].
#
# ############

from collections import OrderedDict
import socket
from MovellaHandler import MovellaFacade
import numpy as np
import time
import struct


if __name__ == "__main__":
  ###########################
  ###### CONFIGURATION ######
  ###########################
  device_mapping = {
    "knee_right"  : "40195BFC800B01F2", # change the serial numbers to your dots (small text on the back of each dot)
    "foot_right"  : "40195BFC800B003B",
    "pelvis"      : "40195BFD80C20052",
    "knee_left"   : "40195BFC800B017A",
    "foot_left"   : "40195BFD80C200D1",
  }
  master_device = 'pelvis' # wireless dot relaying messages, must match a key in the `device_mapping`
  sampling_rate_hz = 60 # can be 30 or 60
  is_get_orientation = False # at 60Hz, Quaternion from DOTs makes packets too large -> dropout in some sensors
  is_sync_devices = True # hopefully your wireless driver supports the 

  desktop_ip = ''                   # ? your desktop's IP after it connected to prosthesis WiFi
  prosthesis_ip = '192.168.0.101'
  prosthesis_port = 51702           # ? your port from LabView

  ###################
  ###### LOGIC ######
  ###################
  handler = MovellaFacade(device_mapping=device_mapping, 
                          master_device=master_device,
                          sampling_rate_hz=sampling_rate_hz,
                          is_get_orientation=is_get_orientation,
                          is_sync_devices=is_sync_devices)
  num_trackers = len(device_mapping)
  row_id_mapping = OrderedDict([(device_id, row_id) for row_id, device_id in enumerate(device_mapping.values())])
  
  # Keep reconnecting until success
  while not handler.initialize(): 
    handler.cleanup()

  # Create a UDP socket
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock.bind((desktop_ip, prosthesis_port))


  def process_data() -> None:
    # Stamps full-body snapshot with system time of start of processing, not time-of-arrival.
    # NOTE: time-of-arrival available with each packet.
    process_time_s: float = time.time()
    # Retrieve the oldest enqueued packet for each sensor.
    snapshot = handler.get_snapshot()
    if snapshot is not None: 
      acceleration = np.empty((num_trackers, 3), dtype=np.float32)
      acceleration.fill(np.nan)
      gyroscope = np.empty((num_trackers, 3), dtype=np.float32)
      gyroscope.fill(np.nan)
      magnetometer = np.empty((num_trackers, 3), dtype=np.float32)
      magnetometer.fill(np.nan)
      if is_get_orientation:
        orientation = np.empty((num_trackers, 4), dtype=np.float32)
        orientation.fill(np.nan)
      timestamp = np.zeros((num_trackers), np.uint32)
      toa_s = np.empty((num_trackers), dtype=np.float32)
      toa_s.fill(np.nan)
      counter = np.zeros((num_trackers), np.uint32)

      for device, packet in snapshot.items():
        id = row_id_mapping[device]
        if packet and packet["acc"].size:
          acceleration[id] = packet["acc"]
          gyroscope[id] = packet["gyr"]
          magnetometer[id] = packet["mag"]
          if is_get_orientation:
            orientation[id] = packet["quaternion"]
          timestamp[id] = packet["timestamp_fine"]
          toa_s[id] = packet["toa_s"]
          counter[id] = packet["counter"]

      data = {
        'acceleration': acceleration,
        'gyroscope': gyroscope,
        'magnetometer': magnetometer,
      }
      if is_get_orientation:
        data["orientation"] = orientation
      
      # The payload contents are bytes with float32 structured as:
      # Acc-X: 5 x float32 -> tracker 1:5 in the same order as defined in 'device_mapping' 
      # Acc-Y:
      # Acc-Z:
      # Gyr-X:
      # Gyr-Y:
      # Gyr-Z:
      # Mag-X:
      # Mag-Y:
      # Mag-Z:
      payload: bytes = acceleration.tobytes()+gyroscope.tobytes()+magnetometer.tobytes()
      sock.sendto(payload, (prosthesis_ip, prosthesis_port))


  #######################
  ###### MAIN LOOP ######
  #######################
  try:
    while True:
      process_data()
  except KeyboardInterrupt:
    print("Keyboard interrupt signalled, quitting...", flush=True)
  finally:
    handler.cleanup()
    sock.close()
    print("Experiment ended, thank you for using our system <3", flush=True)
