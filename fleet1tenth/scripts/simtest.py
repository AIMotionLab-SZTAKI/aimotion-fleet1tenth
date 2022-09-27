from fleet1tenthpy import Fleet1tenth
import time

fleet1tenth=Fleet1tenth(config_file_path="config/simulation.yaml")

fleet1tenth.fleet.init_cars()
fleet1tenth.fleet.launch_cars()

time.sleep(10)
print("goodbye")

