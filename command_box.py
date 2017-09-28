# -*- coding: utf-8 -*-



import time

from drone import Drone


if __name__ == "__main__":
    drone = Drone()
    drone.connect("tcp:127.0.0.1:5760")
    #Give time to establish the connection
    time.sleep(2)
    
    drone.mainLoop()
        
    # terminate the connection
    drone.disconnect()