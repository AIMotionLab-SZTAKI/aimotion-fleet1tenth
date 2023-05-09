from fleet1tenthpy import Fleet
import argparse
import time


def parse_arguments():
    """Argument parser for for convenient utilization"""

    # init parser and add arrguments
    parser=argparse.ArgumentParser(description="Remote keyboard operation argument parser")
    parser.add_argument("--car_id", type=str, help="ID of the vehicle")
    parser.add_argument("--logging", action="store_true", help="Enable state logging")

    # parse known args
    args,_=parser.parse_known_args()

    return str(args.car_id), bool(args.logging)


# init fleet
fleet=Fleet("config/configuration.yaml")


# parse args, init vehicles or use GUI to choose
car_id, logging=parse_arguments()
print(car_id)
if car_id is not None:
    print(type(car_id))
    fleet.init_cars(car_id)
else:
    print("???????????,")


# launches the onboard nodes on the selected cars
fleet.launch_cars(car_id)

if logging:
    car1=fleet.get_car_by_ID("AI_car_01")
    car1.init_logger("identify/motion", gui=True)

# starts the control application
fleet.keyboard_remote()

# shutdown car nodes properly
fleet.shutdown_cars()