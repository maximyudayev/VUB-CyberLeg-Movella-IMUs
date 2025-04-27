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
import threading
from MovellaHandler import MovellaFacade
import numpy as np
from time import perf_counter


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
  sampling_rate_hz = 30 # can be [1, 4, 10, 12, 15, 20, 30, 60] -> use 1Hz to visually test how long network latency is.
  is_get_orientation = False # at 60Hz, Quaternion from DOTs makes packets too large -> dropout in some sensors
  is_sync_devices = True # hopefully your wireless driver supports the 

  prosthesis_ip = '192.168.0.101'   # ? Prosthesis IP or localhost (to simulate receiving)
  prosthesis_port = 51705           # ? your port from LabView
  daq_ip = '192.168.0.100'
  daq_port = 51705

  ###################
  ###### LOGIC ######
  ###################
  handler = MovellaFacade(device_mapping=device_mapping,
                          master_device=master_device,
                          sampling_rate_hz=sampling_rate_hz,
                          is_get_orientation=is_get_orientation,
                          is_sync_devices=is_sync_devices,
                          timesteps_before_stale=10)
  num_trackers = len(device_mapping)
  row_id_mapping = OrderedDict([(device_id, row_id) for row_id, device_id in enumerate(device_mapping.values())])
  
  # Keep reconnecting until success
  while not handler.initialize(): 
    handler.cleanup()
  print("Started streaming.", flush=True)

  # Create a UDP socket
  sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

  def process_data() -> bool:
    # Stamps full-body snapshot with system time of start of processing, not time-of-arrival.
    # NOTE: time-of-arrival available with each packet.
    # Retrieve the oldest enqueued packet for each sensor.
    snapshot = handler.get_snapshot()
    if snapshot is not None:
      # process_time_s: float = perf_counter()
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
      toa_s = np.empty((num_trackers), dtype=np.float64)
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
      
      # The payload contents are bytes with float32 structured as:
      payload: bytes = acceleration.tobytes()+gyroscope.tobytes()+magnetometer.tobytes() # 5x3 (tracker dimensions) x3 (acc/gyr/mag) x4 (bytes) = 180 bytes
      sock.sendto(payload, (prosthesis_ip, prosthesis_port))
      # print("TOA diff: ", process_time_s-toa_s, " @ %.6f"%process_time_s, flush=True)
      return False
    elif snapshot is None and not handler._is_more:
      return True


  #######################
  ###### MAIN LOOP ######
  #######################
  def foo():
    is_continue = False
    while not is_continue:
      is_continue = process_data()

  t = threading.Thread(target=foo)
  t.start()
  is_exit = False
  while not is_exit:
    is_exit = input("Enter 'q' to exit: ") == 'q'
  handler.cleanup()
  t.join()
  handler.close()
  sock.close()
  print("Experiment ended, thank you for using our system <3", flush=True)
