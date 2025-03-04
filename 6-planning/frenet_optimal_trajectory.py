"""

Frenet optimal trajectory generator

author: Atsushi Sakai (@Atsushi_twi)

Ref:

- [Optimal Trajectory Generation for Dynamic Street Scenarios in a Frenet Frame]
(https://www.researchgate.net/profile/Moritz_Werling/publication/224156269_Optimal_Trajectory_Generation_for_Dynamic_Street_Scenarios_in_a_Frenet_Frame/links/54f749df0cf210398e9277af.pdf)

- [Optimal trajectory generation for dynamic street scenarios in a Frenet Frame]
(https://www.youtube.com/watch?v=Cj6tAQe7UCY)

"""

import numpy as np
import matplotlib.pyplot as plt
import copy
import math
import sys
import pathlib
import matplotlib
from quintic_polynomials_planner import QuinticPolynomial 
import cubic_spline_planner
from params import *
from time import time
import multiprocessing
from functools import partial
import itertools

SIM_LOOP = 2500

show_animation = True

try: cpu_pool = multiprocessing.Pool(None)  # Auto-infer CPUs number
except: pass

RANGE_2 = np.arange(MIN_T, MAX_T, DT)
RANGE_3 = np.arange(TARGET_SPEED - D_T_S * N_S_SAMPLE, TARGET_SPEED + D_T_S * N_S_SAMPLE, D_T_S)

LON_LAT_WEIGHTS_VECTOR = np.array([K_J, K_T, K_D])
TOTAL_COST_WEIGHTS_VECTOR = np.array([K_LAT, K_LON])


class QuarticPolynomial:

    def __init__(self, xs, vxs, axs, vxe, axe, time):
        # calc coefficient of quartic polynomial

        self.a0 = xs
        self.a1 = vxs
        self.a2 = axs / 2.0

        A = np.array([[3 * time ** 2, 4 * time ** 3],
                      [6 * time, 12 * time ** 2]])
        b = np.array([vxe - self.a1 - 2 * self.a2 * time,
                      axe - 2 * self.a2])
        x = np.linalg.solve(A, b)

        self.a3 = x[0]
        self.a4 = x[1]

    def calc_point(self, t):
        xt = self.a0 + self.a1 * t + self.a2 * t ** 2 + \
             self.a3 * t ** 3 + self.a4 * t ** 4

        return xt

    def calc_first_derivative(self, t):
        xt = self.a1 + 2 * self.a2 * t + \
             3 * self.a3 * t ** 2 + 4 * self.a4 * t ** 3

        return xt

    def calc_second_derivative(self, t):
        xt = 2 * self.a2 + 6 * self.a3 * t + 12 * self.a4 * t ** 2

        return xt

    def calc_third_derivative(self, t):
        xt = 6 * self.a3 + 24 * self.a4 * t

        return xt


class FrenetPath:

    def __init__(self):
        self.t = []
        self.d = []
        self.d_d = []
        self.d_dd = []
        self.d_ddd = []
        self.s = []
        self.s_d = []
        self.s_dd = []
        self.s_ddd = []
        self.cd = 0.0
        self.cv = 0.0
        self.cf = 0.0

        self.x = []
        self.y = []
        self.yaw = []
        self.ds = []
        self.c = []

def calc_frenet_paths(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    
    # CPU chunking test
    # chunk_size = MAX_ROAD_WIDTH/2
    # displacement = w_idx*chunk_size
    # start_w = -MAX_ROAD_WIDTH + displacement
    # end_w = min(MAX_ROAD_WIDTH, start_w+chunk_size)

    # generate path to each offset goal
    for di in np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W):

        # Lateral motion planning
        for Ti in RANGE_2:
            fp = FrenetPath()

            # lat_qp = quintic_polynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)
            lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

            fp.t = np.arange(0.0, Ti, DT).tolist()
            fp.d = [lat_qp.calc_point(t) for t in fp.t]
            fp.d_d = [lat_qp.calc_first_derivative(t) for t in fp.t]
            fp.d_dd = [lat_qp.calc_second_derivative(t) for t in fp.t]
            fp.d_ddd = [lat_qp.calc_third_derivative(t) for t in fp.t]

            # Longitudinal motion planning (Velocity keeping)
            for tv in RANGE_3:
                tfp = copy.copy(fp)
                lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

                tfp.s = [lon_qp.calc_point(t) for t in fp.t]
                tfp.s_d = [lon_qp.calc_first_derivative(t) for t in fp.t]
                tfp.s_dd = [lon_qp.calc_second_derivative(t) for t in fp.t]
                tfp.s_ddd = [lon_qp.calc_third_derivative(t) for t in fp.t]

                Jp = np.sum(np.power(tfp.d_ddd, 2))  # square of jerk
                Js = np.sum(np.power(tfp.s_ddd, 2))  # square of jerk

                # square of diff from target speed
                ds = np.square(TARGET_SPEED - tfp.s_d[-1])

                tfp.cd = K_J * Jp + K_T * Ti + K_D * np.square(tfp.d[-1])
                tfp.cv = K_J * Js + K_T * Ti + K_D * ds
                tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

                frenet_paths.append(tfp)

    return frenet_paths

def calc_frenet_paths_inner(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0, di):
    res = []
    # Lateral motion planning
    for Ti in RANGE_2:
        fp = FrenetPath()

        lat_qp = QuinticPolynomial(c_d, c_d_d, c_d_dd, di, 0.0, 0.0, Ti)

        fp.t = np.arange(0.0, Ti, DT)
        fp.d = np.array([lat_qp.calc_point(t) for t in fp.t])
        fp.d_d = np.array([lat_qp.calc_first_derivative(t) for t in fp.t])
        fp.d_dd = np.array([lat_qp.calc_second_derivative(t) for t in fp.t])
        fp.d_ddd = np.array([lat_qp.calc_third_derivative(t) for t in fp.t])

        # Longitudinal motion planning (Velocity keeping)
        for tv in RANGE_3:
            tfp = copy.copy(fp)
            lon_qp = QuarticPolynomial(s0, c_speed, c_accel, tv, 0.0, Ti)

            tfp.s = np.array([lon_qp.calc_point(t) for t in fp.t])
            tfp.s_d = np.array([lon_qp.calc_first_derivative(t) for t in fp.t])
            tfp.s_dd = np.array([lon_qp.calc_second_derivative(t) for t in fp.t])
            tfp.s_ddd = np.array([lon_qp.calc_third_derivative(t) for t in fp.t])

            Jp = np.sum(np.power(tfp.d_ddd, 2))  # square of jerk
            Js = np.sum(np.power(tfp.s_ddd, 2))  # square of jerk

            # square of diff from target speed
            ds = np.square(TARGET_SPEED - tfp.s_d[-1])

            # tfp.cd = K_J * Jp + K_T * Ti + K_D * np.square(tfp.d[-1])
            # tfp.cv = K_J * Js + K_T * Ti + K_D * ds
            # tfp.cf = K_LAT * tfp.cd + K_LON * tfp.cv

            LAT_VECTOR = np.array([Jp, Ti, np.square(tfp.d[-1])])
            LON_VECTOR = np.array([Js, Ti, ds])

            tfp.cd = np.inner(LON_LAT_WEIGHTS_VECTOR, LAT_VECTOR)
            tfp.cv = np.inner(LON_LAT_WEIGHTS_VECTOR, LON_VECTOR)
            tfp.cf = np.inner(TOTAL_COST_WEIGHTS_VECTOR, np.array((tfp.cd, tfp.cv)))

            res.append(tfp)

    return res

def calc_frenet_paths_parallel(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0):
    frenet_paths = []
    _calc_frenet_paths_inner = partial(calc_frenet_paths_inner, c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)

    # generate path to each offset goal
    frenet_paths = cpu_pool.map(
        _calc_frenet_paths_inner,  # Function to be parallelized
        np.arange(-MAX_ROAD_WIDTH, MAX_ROAD_WIDTH, D_ROAD_W)
    )

    flattened_list = list(itertools.chain.from_iterable(frenet_paths))
    
    return flattened_list


def calc_global_paths_parallel(fplist, csp):
    _calc_global_paths_inner = partial(calc_global_paths_inner, csp)

    return cpu_pool.map(
        _calc_global_paths_inner,
        fplist
    )

def calc_global_paths_inner(csp, fp):
    # calc global positions

    # s = np.array(
    #     list(
    #         map(
    #             csp.calc_position,
    #             fp.s
    #         )
    #     )
    # )

    s = np.array(fp.s)

    x, y = csp.calc_position_vectorized(s)

    # yaw = np.array(
    #     [*map(
    #         csp.calc_yaw,
    #         fp.s
    #     )]
    # )
    
    yaw = csp.calc_yaw_vectorized(s)

    d = np.array(fp.d)

    yaw_phased = yaw + math.pi / 2.0

    cos = np.cos(yaw_phased)
    sin = np.sin(yaw_phased)

    fx = x + d * cos
    fy = y + d * sin
    
    
    # calc yaw and ds
    dx = np.diff(fx)
    dy = np.diff(fy)

    yaw = np.atan2(dy, dx)
    yaw += yaw[-1]

    ds = np.hypot(dx, dy)
    ds += ds[-1]

    # calc curvature
    dyaw = np.diff(yaw)

    fp.c = dyaw/ds[:-1]
    fp.yaw = yaw
    fp.ds = ds
    fp.x = fx
    fp.y = fy

    return fp

def calc_global_paths(fplist, csp):
    for fp in fplist:
        calc_global_paths_inner(csp, fp)

    return fplist

def check_collision(fp, ob):
    if(len(ob[0]) == 0):
        return True
    
    obstacles_num = len(ob[:, 0])

    for i in range(obstacles_num):
        ob_x, ob_y = ob[i]
        ob_s = np.ones((len(fp.x), 1)) * np.array([ob_x, ob_y], ndmin=2)

        fp_s = np.array([fp.x, fp.y]).T
        squared_diff = np.square(fp_s - ob_s).T
        squared_dist = squared_diff[0] + squared_diff[1]

        collision = np.any(squared_dist <= ROBOT_RADIUS ** 2)
        if collision: return False
    
    return True
    

def check_collision_parallel(ob, fp):
    if(len(ob[0]) == 0):
        return fp
    
    obstacles_num = len(ob[:, 0])

    for i in range(obstacles_num):
        ob_x, ob_y = ob[i]
        ob_s = np.ones((len(fp.x), 1)) * np.array([ob_x, ob_y], ndmin=2)

        fp_s = np.array([fp.x, fp.y]).T
        squared_diff = np.square(fp_s - ob_s).T
        squared_dist = squared_diff[0] + squared_diff[1]

        collision = np.any(squared_dist <= ROBOT_RADIUS ** 2)
        if collision: return None
    
    return fp

def check_paths(fplist, ob):
    ok_ind = []
    for i, _ in enumerate(fplist):
        # if any([v > MAX_SPEED for v in fplist[i].s_d]):  # Max speed check
        #     print("max speed exceeded")
        #     continue
        # elif any([abs(a) > MAX_ACCEL for a in
        #           fplist[i].s_dd]):  # Max accel check
        #     print("max accel exceeded")
        #     continue
        # elif any([abs(c) > MAX_CURVATURE for c in
        #           fplist[i].c]):  # Max curvature check
        #     print("max curv exceeded")
        #     continue
        # elif not check_collision(fplist[i], ob):
        #     print("collision checks failed")
        #     continue
        
        if not check_collision(fplist[i], ob):
            # print("Collision checks not passed")
            continue
        
        ok_ind.append(i)

    return [fplist[i] for i in ok_ind]

def check_paths_parallel(fplist, ob):
    _check_collision = partial(check_collision_parallel, ob)


    collision_free_fp = cpu_pool.map(
        _check_collision,
        fplist
    )

    return list(
        filter(
            lambda x: x is not None,
            collision_free_fp
        )
    )

def frenet_optimal_planning(csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob):
    # start = time()
    fplist = calc_frenet_paths_parallel(c_speed, c_accel, c_d, c_d_d, c_d_dd, s0)
    # print(f"{len(fplist)} paths after calc_frenet_paths_parallel in {time()-start}")

    # start = time()
    fplist = calc_global_paths(fplist, csp)
    # print(f"{len(fplist)} converted to global coordinates in {time()-start}")

    # start = time()
    fplist = check_paths_parallel(fplist, ob)
    # print(f"{len(fplist)} paths after checks in {time()-start}")

    # find minimum cost path
    # start = time()
    min_cost = float("inf")
    best_path = None

    for fp in fplist:
        if min_cost >= fp.cf:
            min_cost = fp.cf
            best_path = fp
    
    # print(f"Found min cost path in {time()-start}")

    return best_path


def generate_target_course(x, y):
    csp = cubic_spline_planner.Spline2D(x, y)
    s = np.arange(0, csp.s[-1], 0.1)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = csp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(csp.calc_yaw(i_s))
        rk.append(csp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, csp

def load_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []

    while(file.readline()):
        line = file.readline()
        xs.append( float(line.split(",")[0]) )
        ys.append( float(line.split(",")[1]) )
    return xs, ys

def main():
    print(__file__ + " start!!")

    # obstacle lists
    ob = np.array([[40.0, 0.0],
                    [100.0, -0.5],
                    [400.0, 0.5],
                    [570.0, 29.0],
                    [120.0, 200.0],
                    [33.0, 200.0],
                    [-70.0, 171.0]
                    ])
    
    # Load path and create a spline
    wx, wy = load_path("oval_trj.txt")
    tx, ty, tyaw, tc, csp = generate_target_course(wx, wy)

    # initial state
    c_speed = 10.0  # current speed [m/s]
    c_accel = 0.0  # current acceleration [m/ss]
    c_d = 0.0  # current lateral position [m]
    c_d_d = 0.0  # current lateral speed [m/s]
    c_d_dd = 0.0  # current lateral acceleration [m/s]
    s0 = 0.0  # current course position

    area = 40.0  # animation area length [m]

    for i in range(SIM_LOOP):
        path = frenet_optimal_planning(
            csp, s0, c_speed, c_accel, c_d, c_d_d, c_d_dd, ob)

        if not path:
            print("Feasable Frenet path not found")
            break

        s0 = path.s[1]
        c_d = path.d[1]
        c_d_d = path.d_d[1]
        c_d_dd = path.d_dd[1]
        c_speed = path.s_d[1]
        c_accel = path.s_dd[1]

        if np.hypot(path.x[1] - tx[-1], path.y[1] - ty[-1]) <= 1.0:
            print("Goal")
            break

        if show_animation:  # pragma: no cover
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(tx, ty)
            plt.plot(ob[:, 0], ob[:, 1], "xk")
            plt.plot(path.x[1:], path.y[1:], "-or")
            plt.plot(path.x[1], path.y[1], "vc")
            plt.xlim(path.x[1] - area, path.x[1] + area)
            plt.ylim(path.y[1] - area, path.y[1] + area)
            plt.title("v[km/h]:" + str(c_speed * 3.6)[0:4])
            plt.grid(True)
            plt.pause(0.0001)

    print("Finish")
    
    if show_animation:  # pragma: no cover
        plt.grid(True)
        plt.pause(0.0001)
        plt.show()


if __name__ == '__main__':
    main()
