from scipy.interpolate import splprep, splrep, splev
import numpy as np
import matplotlib.pyplot as plt
import pickle
from fleet1tenthpy import Fleet1tenth
import time
import socket



 # load paths
paths=[]
with open("city_demo_all_paths.pkl", "rb") as f:
    while True:
        try:
            paths.append(pickle.load(f))
        except EOFError:
            break


# start F1TENTH vehicle
fleet1tenth=Fleet1tenth(config_file_path=None)
cid="AI_car_01"
fleet1tenth.fleet.init_cars(cid)
fleet1tenth.fleet.launch_cars(cid)
c1=fleet1tenth.fleet.get_car_by_ID(cid)



# SKYBRUSH TCP connections
drone_server_ip="192.168.2.77"
demo_port=6001
live_port=6002
dummy_port=6003


try: # try to open demo port
    skybrush_client_socket=socket.socket()
    skybrush_client_socket.connect((drone_server_ip, dummy_port))
    MODE="DEMO" # launch demo mode
except:
    # demo port failed, try live port
    try:
        skybrush_client_socket=socket.socket()
        skybrush_client_socket.connect((drone_server_ip, live_port))
        MODE="LIVE" # launch live mode
    except Exception as e:
        print(e)
        MODE="MANUAL"


print(f"Launch mode: {MODE}")
if MODE == "MANUAL":
    msg=input("Specify demo countdown: ")

else:
    msg=skybrush_client_socket.recv(1024)
    skybrush_client_socket.close()

# sleep for the predefined time
print(f"Waiting {float(msg)} to launch!")
time.sleep(float(msg))
print("Trajectory execution started!")

# start execution
for i, path in enumerate(paths):

    # send trajectory index in demo mode
    if MODE == "DEMO":
        demo_client_socket=socket.socket()
        demo_client_socket.connect((drone_server_ip, demo_port))
        demo_client_socket.sendall(f"{i}EOF".encode("utf-8"))
        demo_client_socket.close()

    # sleep for 1 second
    time.sleep(1)

    # execute trajectory
    res=c1.execute_trajectory(path.tck, path.speed_tck, (0,path.length))
    if res:
        print(f"Trajectory {i} executed successfully!")
    else:
        print(f"Execution of trajectory {i} failed!")
        break