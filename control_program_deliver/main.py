from Dynamics_model import Dy_Car
from pid_lon import PID
from Kinematics_model import Ki_Car
from lqr_lat import LQR_controller
from visual import Visual
from generate_path import Path


FILENAME = 'target_list_2024-10-21_14_49_29.txt'
# FILENAME = 'target_list_2024-10-29_15_16_33.txt'
MAX_ITERATOR = 100000

if __name__ == '__main__':
    lqr = LQR_controller()
    dy_car = Dy_Car(1412, -112600, -89500, 2, 1.0, 1.9, 2.9, 1536, 3.14, 0, 0, 0.01, -1058.5960008467866,
                    -12017.00799999886, 0)
    # ki_car = Ki_Car(-1058.5960008467866, -12017.00799999886, 3.14, 2.9, 1, 0.01)
    pid = PID(1, 0, 0, 0.001)

    tar_path = Path(FILENAME)
    tar_path.path_preparation()

    my_visual = Visual(dy_car, tar_path, lqr)

    lqr.update_trajectory(tar_path.planning_x_list, tar_path.planning_y_list, tar_path.planning_h_list, tar_path.planning_v_list)
    lqr.LQR_Init()

    for i in range(MAX_ITERATOR):
        print("match_point = ", lqr.i)
        print("_________________________________________________", '\n')

        lqr.track(dy_car)
        dy_car.vx += pid.pid_lon(dy_car.vx, tar_path.planning_v_list[lqr.i])
        dy_car.update(lqr.steer)
        if lqr.i > len(tar_path.planning_x_list) - 5:
            break

    my_visual.print_result()
