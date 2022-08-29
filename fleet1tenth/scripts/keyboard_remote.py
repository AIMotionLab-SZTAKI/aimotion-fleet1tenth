from fleet1tenthpy import Fleet

fleet=Fleet("config/configuration.yaml")

# openc a dialog where users can choose vehicless from fleet to initialize
# then creates the control interface for the selected cars
fleet.init_cars()


# launches the onboard nodes on the selected cars
fleet.launch_cars()

car1=fleet.get_car_by_ID("RC_car_01")
car1.init_logger("drive_logs/circle05", gui=True)
# starts the control application
fleet.keyboard_remote()


# shutdown car nodes properly
fleet.shutdown_cars()