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

import socket
import time
import struct


if __name__ == "__main__":
  ###########################
  ###### CONFIGURATION ######
  ###########################
  prosthesis_ip = '192.168.0.101'   # ? Prosthesis IP
  prosthesis_port = 51702           # ? your port from LabView

  ###################
  ###### LOGIC ######
  ###################
  # Create a UDP socket
  sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
  sock_recv.bind((prosthesis_ip, prosthesis_port))


  def process_data() -> None:
    payload_recv = sock_recv.recv(200) # 180 bytes of IMU data and 20 bytes of counter data to keep measure how long it takes to deliver. 
    counters = struct.unpack('IIIII', payload_recv[180:200]) # 0,1,2,3,4
    print("Received: ", counters, flush=True) # This should get printed close to instantly to the print log in the sender code.
    
    recv_time_s: float = time.time()
    # Cast each 12 bytes (3 dimensions of 4 byte-fp32 of 1 tracker) into separate measurements
    #  NOTE: you can slice/index in LabView into the packed bytes to match your LabView code better
    #   i.e. Take X dimension of acceleration of all trackers, or have 45 individual signals with 1x fp32 value (below). 
    acc_tracker_0 = struct.unpack('fff', payload_recv[0:12])    # x,y,z
    acc_tracker_1 = struct.unpack('fff', payload_recv[12:24])   # x,y,z
    acc_tracker_2 = struct.unpack('fff', payload_recv[24:36])   # x,y,z
    acc_tracker_3 = struct.unpack('fff', payload_recv[36:48])   # x,y,z
    acc_tracker_4 = struct.unpack('fff', payload_recv[48:60])   # x,y,z

    gyr_tracker_0 = struct.unpack('fff', payload_recv[60:72])   # x,y,z
    gyr_tracker_1 = struct.unpack('fff', payload_recv[72:84])   # x,y,z
    gyr_tracker_2 = struct.unpack('fff', payload_recv[84:96])   # x,y,z
    gyr_tracker_3 = struct.unpack('fff', payload_recv[96:108])  # x,y,z
    gyr_tracker_4 = struct.unpack('fff', payload_recv[108:120]) # x,y,z

    mag_tracker_0 = struct.unpack('fff', payload_recv[120:132]) # x,y,z
    mag_tracker_1 = struct.unpack('fff', payload_recv[132:144]) # x,y,z
    mag_tracker_2 = struct.unpack('fff', payload_recv[144:156]) # x,y,z
    mag_tracker_3 = struct.unpack('fff', payload_recv[156:168]) # x,y,z
    mag_tracker_4 = struct.unpack('fff', payload_recv[168:180]) # x,y,z

    # OR:
    # Interpret each 4 packed bytes as floats, for a total of 45 times.
    # flattened_results: tuple[float] = struct.unpack('f'*45, payload_recv)
    # NOTE: if `is_sync_devices` is False, most values will be None (nan)
    #   expected because clocks are off:
    #   (reset device buttons and increase `timesteps_before_stale` on MovellaFacade).
    # NOTE: if `is_sync_devices` is True, all data will be synced and consistent
    #   your BLE driver must support the feature for syncing the DOTs through their SDK.

  #######################
  ###### MAIN LOOP ######
  #######################
  try:
    while True:
      process_data()
  except KeyboardInterrupt:
    print("Keyboard interrupt signalled, quitting...", flush=True)
  finally:
    sock_recv.close()
    print("Experiment ended, thank you for using our system <3", flush=True)
