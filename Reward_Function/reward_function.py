import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):

            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(
                first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum(
                [times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time /
                                  current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the breadcentricloop track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[2.04655, 2.17258, 4.0, 0.07484],
                        [1.76534, 2.0655, 4.0, 0.07523],
                        [1.48377, 1.95746, 4.0, 0.0754],
                        [1.2022, 1.84942, 4.0, 0.0754],
                        [0.92062, 1.74139, 4.0, 0.0754],
                        [0.63905, 1.63336, 4.0, 0.0754],
                        [0.35747, 1.52533, 4.0, 0.0754],
                        [0.07589, 1.4173, 4.0, 0.0754],
                        [-0.20568, 1.30926, 3.47845, 0.0867],
                        [-0.48726, 1.20123, 3.05117, 0.09884],
                        [-0.76873, 1.0938, 2.7124, 0.11107],
                        [-1.04934, 0.99109, 2.42883, 0.12303],
                        [-1.3282, 0.89696, 2.13852, 0.13763],
                        [-1.60421, 0.81516, 1.9091, 0.15079],
                        [-1.8762, 0.74895, 1.6877, 0.16586],
                        [-2.1428, 0.70136, 1.48534, 0.18233],
                        [-2.40255, 0.67512, 1.48534, 0.17576],
                        [-2.65378, 0.67275, 1.48534, 0.16915],
                        [-2.89414, 0.69781, 1.48534, 0.1627],
                        [-3.12091, 0.75353, 1.48534, 0.15722],
                        [-3.33003, 0.84476, 1.48534, 0.1536],
                        [-3.51456, 0.97906, 1.81256, 0.12592],
                        [-3.68063, 1.14226, 2.17591, 0.107],
                        [-3.83242, 1.32637, 1.96493, 0.12144],
                        [-3.97437, 1.52432, 1.74094, 0.13992],
                        [-4.11156, 1.73217, 1.55202, 0.16046],
                        [-4.2592, 1.93033, 1.45615, 0.1697],
                        [-4.41719, 2.11153, 1.3, 0.18493],
                        [-4.58534, 2.2705, 1.3, 0.178],
                        [-4.76388, 2.40177, 1.3, 0.17047],
                        [-4.9522, 2.50061, 1.3, 0.1636],
                        [-5.14885, 2.56155, 1.3, 0.15836],
                        [-5.35086, 2.58134, 1.3, 0.15614],
                        [-5.55364, 2.54928, 1.41012, 0.14559],
                        [-5.75081, 2.47103, 1.55371, 0.13653],
                        [-5.93752, 2.35222, 1.6977, 0.13036],
                        [-6.10953, 2.19772, 1.8543, 0.12469],
                        [-6.26339, 2.01311, 2.03684, 0.11799],
                        [-6.39718, 1.80493, 2.16709, 0.11419],
                        [-6.50918, 1.57846, 2.27608, 0.111],
                        [-6.59841, 1.3386, 2.39362, 0.10692],
                        [-6.66512, 1.08973, 2.28019, 0.113],
                        [-6.71031, 0.83528, 1.92731, 0.13409],
                        [-6.73448, 0.57777, 1.92731, 0.1342],
                        [-6.73789, 0.31926, 1.92731, 0.13414],
                        [-6.72019, 0.06158, 1.92731, 0.13402],
                        [-6.67968, -0.19331, 1.92731, 0.13391],
                        [-6.61285, -0.44255, 1.92731, 0.13389],
                        [-6.51044, -0.67943, 2.11029, 0.12229],
                        [-6.37993, -0.90222, 2.27533, 0.11348],
                        [-6.22662, -1.11038, 2.42727, 0.10651],
                        [-6.05455, -1.30379, 2.56548, 0.1009],
                        [-5.86685, -1.48253, 2.72719, 0.09504],
                        [-5.66633, -1.64726, 2.79696, 0.09278],
                        [-5.45479, -1.79825, 2.61495, 0.09939],
                        [-5.23368, -1.93573, 2.3876, 0.10905],
                        [-5.00399, -2.05959, 2.15584, 0.12105],
                        [-4.76666, -2.16973, 2.15584, 0.12137],
                        [-4.52219, -2.26533, 2.15584, 0.12176],
                        [-4.27053, -2.34377, 2.15584, 0.12227],
                        [-4.01193, -2.40183, 2.15584, 0.12294],
                        [-3.7469, -2.43454, 2.15584, 0.12387],
                        [-3.47692, -2.4349, 2.61545, 0.10323],
                        [-3.20512, -2.41256, 2.89585, 0.09417],
                        [-2.93246, -2.37114, 3.23013, 0.08538],
                        [-2.65944, -2.31391, 3.61524, 0.07716],
                        [-2.38634, -2.24368, 4.0, 0.0705],
                        [-2.11332, -2.16303, 4.0, 0.07117],
                        [-1.84042, -2.07401, 4.0, 0.07176],
                        [-1.56767, -1.97834, 4.0, 0.07226],
                        [-1.29507, -1.87775, 4.0, 0.07264],
                        [-1.02258, -1.77351, 3.78045, 0.07717],
                        [-0.75444, -1.6685, 3.37334, 0.08537],
                        [-0.48699, -1.56758, 3.02045, 0.09464],
                        [-0.22062, -1.47282, 2.7532, 0.10269],
                        [0.04429, -1.38619, 2.41592, 0.11537],
                        [0.3073, -1.30953, 2.14418, 0.12777],
                        [0.5679, -1.24473, 1.86512, 0.14398],
                        [0.82541, -1.19395, 1.6291, 0.16112],
                        [1.07901, -1.1595, 1.51229, 0.16923],
                        [1.3278, -1.14331, 1.51229, 0.16486],
                        [1.57021, -1.14902, 1.51229, 0.16034],
                        [1.80425, -1.18033, 1.51229, 0.15614],
                        [2.02663, -1.24287, 1.51229, 0.15275],
                        [2.23212, -1.34378, 1.51229, 0.15138],
                        [2.41439, -1.4878, 2.00383, 0.11593],
                        [2.58102, -1.6565, 2.01441, 0.11771],
                        [2.73917, -1.83782, 1.65667, 0.14523],
                        [2.90182, -2.02116, 1.65667, 0.14794],
                        [3.06964, -2.19796, 1.65667, 0.14714],
                        [3.24623, -2.36312, 1.65667, 0.14595],
                        [3.43503, -2.51054, 1.65667, 0.14459],
                        [3.63885, -2.63254, 1.65667, 0.14339],
                        [3.85979, -2.71517, 1.78894, 0.13186],
                        [4.09018, -2.76262, 1.90731, 0.12333],
                        [4.32482, -2.77841, 2.04496, 0.115],
                        [4.55995, -2.76638, 2.15518, 0.10925],
                        [4.79301, -2.7293, 2.25839, 0.10449],
                        [5.0221, -2.66954, 2.17885, 0.10867],
                        [5.24588, -2.58908, 1.89548, 0.12546],
                        [5.46326, -2.48952, 1.89548, 0.12614],
                        [5.67269, -2.37081, 1.89548, 0.127],
                        [5.8724, -2.23284, 1.89548, 0.12806],
                        [6.06039, -2.07549, 1.89548, 0.12933],
                        [6.23235, -1.89623, 1.89548, 0.13105],
                        [6.3787, -1.69036, 2.06807, 0.12214],
                        [6.49971, -1.46517, 2.28338, 0.11196],
                        [6.59766, -1.22658, 2.4663, 0.10458],
                        [6.67443, -0.97853, 2.62263, 0.09901],
                        [6.73153, -0.72396, 2.74288, 0.09512],
                        [6.77002, -0.46518, 2.83847, 0.09217],
                        [6.79079, -0.2041, 2.91953, 0.08971],
                        [6.79468, 0.05761, 2.97373, 0.08802],
                        [6.78248, 0.31853, 2.80443, 0.09314],
                        [6.75491, 0.57739, 2.56768, 0.10139],
                        [6.71265, 0.83307, 2.34022, 0.11074],
                        [6.65616, 1.08448, 2.12114, 0.12148],
                        [6.58571, 1.33045, 1.93713, 0.13209],
                        [6.50121, 1.56968, 1.68077, 0.15095],
                        [6.40186, 1.8002, 1.54274, 0.16271],
                        [6.28611, 2.01909, 1.54274, 0.1605],
                        [6.15259, 2.22261, 1.54274, 0.15778],
                        [6.00014, 2.40611, 1.54274, 0.15464],
                        [5.82838, 2.56461, 1.54274, 0.1515],
                        [5.63556, 2.68897, 1.54274, 0.14872],
                        [5.42275, 2.77162, 1.91373, 0.11929],
                        [5.19861, 2.82616, 2.10648, 0.10951],
                        [4.96532, 2.85627, 2.32173, 0.10132],
                        [4.72432, 2.86482, 2.52178, 0.09563],
                        [4.47645, 2.85357, 2.76013, 0.0899],
                        [4.22242, 2.82426, 3.0616, 0.08352],
                        [3.96297, 2.77891, 3.38323, 0.07785],
                        [3.69864, 2.71923, 3.82278, 0.07089],
                        [3.43013, 2.64735, 4.0, 0.06949],
                        [3.15813, 2.56547, 4.0, 0.07101],
                        [2.88326, 2.47551, 4.0, 0.0723],
                        [2.60601, 2.37897, 4.0, 0.0734],
                        [2.32692, 2.27747, 4.0, 0.07424]]
        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0  # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(
            self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME,
                               reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3

        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3

        ## Incentive for finishing the lap in less steps ##
        # should be adapted to track length and other rewards
        REWARD_FOR_FASTEST_TIME = 1500
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                                       (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward

        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################

        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" %
                  (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)

        #################### RETURN REWARD ####################

        # Always return a float value
        return float(reward)


reward_object = Reward()  # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
