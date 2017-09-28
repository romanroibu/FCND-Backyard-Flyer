# -*- coding: utf-8 -*-

from pymavlink import mavutil
import threading
import os
from curio import Queue
# force use of mavlink v2.0
os.environ['MAVLINK20'] = '1'

class Connection:
    def __init__(self, device, fn):
        self.callback = fn
        self.master = mavutil.mavlink_connection(device, source_system=190)
        self._running = True
        self.read_handle = threading.Thread(
            target=self.read_thread)
        self.read_handle.start()
        self.write_handle = 0

        self.target_system = 0
        self.target_component = 0
        
        self.msg_types = ['HEARTBEAT','GLOBAL_POSITION_INT']
        self.callbacks = {}
        for m_types in self.msg_types:
            self.callbacks[m_types] = set()
        


    def read_thread(self):

        while self._running:

            # get the next message
            # NOTE: this is a blocking call, which is why we have a thread for it
            msg = self.master.recv_match(blocking=True)
            
            
            # if it's a good message, send it back to the callback
            #if msg.get_type() != 'BAD_DATA':
            #    self.callback(msg.get_type(), msg)

            # want to send a heartbeat periodically, so can just do that when we receive one
            if msg.get_type() == 'HEARTBEAT':
                # print("sending heartbeat")
                # send -> type, autopilot, base mode, custom mode, system status
                self.master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                               mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                               0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            
            if msg.get_type() in self.callbacks.keys():
                for cb in self.callbacks[msg.get_type()]:
                    cb(msg)
                    

        print("read ended")
    
    def subscribe(self, m_type, callback):
        if m_type in self.msg_types:
            self.callbacks[m_type].add(callback)
        else:
            print("Invalid Mavlink type")
    
    def unsubscribe(self, m_type, callback):
        if m_type in self.msg_types:
            self.callbacks[m_type].remove(callback)
        else:
            print("Invalid Mavlink type")
        


    def send_mav_command(self, command_type, param1, param2, param3, param4, x,
                         y, z):
        self.master.mav.command_int_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, command_type, 1, 0, param1,
            param2, param3, param4, int(x), int(y), z)

    def disconnect(self):
        # stop running the read loop, and wait for the thread to finish
        self._running = False
        self.read_handle.join()

        # close the mavutil connection (tcp/udp/serial, etc)
        self.master.close()

