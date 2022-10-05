from fleet1tenthpy import Fleet
import time
fleet=Fleet("config/configuration.yaml")

# opens a dialog where users can choose vehicless from fleet to initialize
# then creates the control interface for the selected cars
# if the car ID is specified as an argument no GUI is opened
fleet.init_cars("AI_car_01")


# launches the onboard nodes on the selected cars
fleet.launch_cars("AI_car_01")

car1=fleet.get_car_by_ID("AI_car_01")
car1.init_logger("drive_logs/motion", gui=True)
# starts the control application
fleet.keyboard_remote()


# shutdown car nodes properly
fleet.shutdown_cars()