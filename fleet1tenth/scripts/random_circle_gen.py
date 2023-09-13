import numpy as np
from fleet1tenthpy.utils import Path
import pickle


def get_arc_points(r, deg_start, deg_end, num_of_points):
    p = np.linspace(deg_start/180*np.pi, deg_end/180*np.pi, num_of_points)
    x = r*np.cos(p)
    y = r*np.sin(p)
    return np.vstack((x,y)).T

startup=np.array([[1.77,  0.827],
                  [1.279, 0.658],
                  [0.956, 0.710],
                  [0.534, 0.827]])

# path1 in -> 57° -> 360°
path1=Path(np.vstack((startup,get_arc_points(0.9,57,360,100))), .75, .1)
path1.plot_traj(block=False)

# path2 in -> 0° -> 300°
path2=Path(get_arc_points(0.9,0,300,100), .75, .1)
path2.plot_traj(block=False)

# path3 in -> 300° -> 30°
path3=Path(np.flip(get_arc_points(0.85,0,300,100),axis=0), -0.75, .1)
print(path3.get_starting_data())
path3.plot_traj(block=False)

# path4 in -> 30° -> 50° (+360°)
path4=Path(get_arc_points(0.95,0,270,100), .75, .1)
path4.plot_traj(block=False)

# path5 in -> 410° -> 57° -> out
path5=Path(np.vstack((get_arc_points(0.85,270,57,200),np.flip(startup, axis=0))), -0.7, .1)
path5.plot_traj(block=True)

paths=[path1, path2, path3, path4, path5]

pickle_file_path="city_demo_all_paths_tuple.pkl"

with open(pickle_file_path, "wb") as f:
    for path in paths:
        todump=(path.tck, path.const_speed)
        pickle.dump(todump, f)


pickle_file_path="city_demo_all_paths.pkl"

with open(pickle_file_path, "wb") as f:
    for path in paths:
        pickle.dump(path, f)