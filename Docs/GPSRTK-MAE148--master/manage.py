"""
manage.py
management all of donkey part 
@authors: Chanyang Yim
"""

import donkeycar
from donkeycar.vehicle import Vehicle
from donkeycar.parts.actuator import PCA9685, PWMSteering, PWMThrottle
from GpsRTK import GpsRTK
from Controller import Controller
from ReportPart import ReportPart
from GpsManualMode import GpsManualMode

"""
round_down
this function is just return after round down 
@author: Omid Hasanli
"""
def round_down(n, decimals=0):
    multiplier = 10 ** decimals
    return math.floor(n * multiplier) / multiplier

"""
GPSPathFinder
By descent algorithm, it automatically find path to goal
input: initial_location final_location, obstacles_loaction_1, obstacles_location_2
output: path to goal from start avoding two obstacles
@author: Omid Hasanli
"""
def GPSPathFinder(initial_location, final_location, obstacles_location_1, obstacles_location_2):
    steps = [0.0, 0.0]
    steps[0] = round_down(((final_location[0] - initial_location[0]) / 100), 10)
    steps[1] = round_down(((final_location[1] - initial_location[1]) / 100), 10)

    a1 = [0.0, 0.0]
    a1[0] = round_down(((obstacles_location_1[0] - initial_location[0]) / steps[0]), 10)
    a1[1] = round_down(((obstacles_location_1[1] - initial_location[1]) / steps[1]), 10)
    a2 = [0.0, 0.0]
    a2[0] = round_down(((obstacles_location_2[0] - initial_location[0]) / steps[0]), 10)
    a2[1] = round_down(((obstacles_location_2[1] - initial_location[1]) / steps[1]), 10)

    initial_loc = [0.0, 0.0]
    cur_loc = initial_loc

    final_loc = [100.0, 100.0]
    mu = [a1, a2]
    sigma = [[50.0, 0.0], [0.0, 50.0]]
    u, v = sp.symbols('u v')
    w = (np.power(final_loc[0] - u, 2) + np.power(final_loc[1] - v, 2)) / 20000 + \
        1e4 * (1 / (sp.det(sp.Matrix(sigma)) * 2 * np.pi)) * sp.exp(
        -(1 / 2) * np.transpose([(u - mu[0][0]), (v - mu[0][1])]).dot(
            np.linalg.pinv(sigma).dot([(u - mu[0][0]), (v - mu[0][1])]))) + \
        1e4 * (1 / (sp.det(sp.Matrix(sigma)) * 2 * np.pi)) * sp.exp(
        -(1 / 2) * np.transpose([(u - mu[1][0]), (v - mu[1][1])]).dot(
            np.linalg.pinv(sigma).dot([(u - mu[1][0]), (v - mu[1][1])])))

    dx = sp.diff(w, u)
    dy = sp.diff(w, v)
    dz = sp.lambdify([u, v], w, 'numpy')

    x = list(np.arange(0.0, 104.0, 4.0))
    y = list(np.arange(0.0, 104.0, 4.0))
    x, y = np.meshgrid(x, y)
    z = dz(x, y)

    p = []
    while not (cur_loc == final_loc):
        if (cur_loc[0] > final_loc[0] or cur_loc[1] > final_loc[1]):
            break
        dX = round_down(dx.evalf(subs={u: cur_loc[0], v: cur_loc[1]}), 10)
        dY = round_down(dy.evalf(subs={u: cur_loc[0], v: cur_loc[1]}), 10)
        d = sp.sqrt(dX ** 2 + dY ** 2)
        alpha = round_down(1 / d, 10)

        cur_loc = list(cur_loc - (alpha * np.array([dX, dY], dtype='float')))
        cur_loc = [round_down(inx, 10) for inx in cur_loc]
        p.append(cur_loc)

    for i in p:
        i[0] = (i[0] * steps[0]) + initial_location[0]
        i[1] = (i[1] * steps[1]) + initial_location[1]
    return p


"""
drive function 
It is call vehicle.py in DonkeyCar to run acutal car
"""
def drive(cfg):
    V = Vehicle()

    #path which is second point in parking lot 
    #path = [[32.865852, -117.21047316666667]]
    
    #path which is make circle start from first point in parking lot 
    """
    path = [
            [32.865863,-117.210494],
            [32.865870,-117.210487],
            [32.865881,-117.210474],
            [32.865888,-117.210463],
            [32.865890,-117.210460],
            [32.865895,-117.210444],
            [32.865895,-117.210432],
            [32.865880,-117.210425],
            [32.865862,-117.210423],
            [32.865849,-117.210428],
            [32.865837,-117.210440],
            [32.865826,-117.210455],
            [32.865823,-117.210457],
            [32.865818,-117.210477],
            [32.865825,-117.210497],
            [32.865828,-117.210502],
            [32.865839,-117.210508],
            [32.865855,-117.210504],
            [32.865863,-117.210495]
            ]
    """
    """
    path = [
            [32.865886333333336, -117.21044666666667],
            [32.865871166666665, -117.21044216666667],
            [32.865856666666666, -117.21045283333333],
            [ 32.86585383333333 , -117.21047116666666]
            ]

    """
    initial_location = [ 32.865891 , -117.2104515 ]
    final_location = [ 32.86583533333334 , -117.21051933333334 ]

    obstacles_location_1 = [ 32.865858333333335 , -117.21049116666667 ]
    obstacles_location_2 = [ 32.86587683333333 , -117.21046883333334 ]
    path = GPSPathFinder(initial_location, final_location, obstacles_location_1, obstacles_location_2)

    #init two new part of donkeycar
    control = Controller(path) 
    #init actuator parts which are steering and throttle

    steering_controller = PCA9685(cfg.STEERING_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    steering = PWMSteering(controller=steering_controller,
                                        left_pulse=0, 
                                        right_pulse=1000)
    
    throttle_controller = PCA9685(cfg.THROTTLE_CHANNEL, cfg.PCA9685_I2C_ADDR, busnum=cfg.PCA9685_I2C_BUSNUM)
    throttle = PWMThrottle(controller=throttle_controller,
                                        max_pulse=cfg.THROTTLE_FORWARD_PWM,
                                        zero_pulse=cfg.THROTTLE_STOPPED_PWM, 
                                        min_pulse=cfg.THROTTLE_REVERSE_PWM)
    
    mode = input("Input m: manual, a: auto:")
    if mode is not 'm' and  mode is not 'a':
        print("wrong input:")
        return 
    if mode == 'm':
        gpsManualMode = GpsManualMode()
        V.add(gpsManualMode,outputs = ["latitude","longditude"])
    if mode == 'a':
        gpsPart = GpsRTK('/dev/ttyACM0',9600,1)
        V.add(gpsPart,outputs = ["latitude","longditude"],threaded = True)
    
    # add webcam to get camera image
    from donkeycar.parts.camera import Webcam
    cam = Webcam(image_w=cfg.IMAGE_W, image_h=cfg.IMAGE_H, image_d=cfg.IMAGE_DEPTH)
    inputs=['cam/image_array',
            'user/angle', 'user/throttle', 
            'user/mode']

    types=['image_array',
           'float', 'float',  
           'str']
    th = TubHandler(path=cfg.DATA_PATH)
    tub = th.new_tub_writer(inputs=inputs, types=types)

    #add parts in vehicle
    
    V.add(control, inputs = ["latitude","longditude"],outputs=["user/angle","user/throttle"])
    V.add(steering, inputs=["user/angle"])
    V.add(throttle, inputs=["user/throttle"])
    V.add(cam, outputs=['cam/image_array'], threaded=True)
    V.add(tub, inputs=inputs, run_condition='recording')
    #V.add(ReportPart(),inputs=["latitude","longditude","user/angle","user/throttle"])
    
    print("Start...")
    V.start()

if __name__ == '__main__':
    cfg = donkeycar.load_config(config_path = './config.py')
    drive(cfg)

                
