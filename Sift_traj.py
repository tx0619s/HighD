import pandas as pd
import numpy as np
import os
import re
#import matplotlib.pyplot as plt


def extract_invalid(states_dic, range_dic, _recording_path):
    recording_df = pd.read_csv(_recording_path)
    upper_lane_markings = recording_df["upperLaneMarkings"].iloc[0].split(";")
    lower_lane_markings = recording_df["lowerLaneMarkings"].iloc[0].split(";")
    lane_markings = upper_lane_markings+lower_lane_markings

    y = range_dic["y_max"]
    y_acceleration = range_dic["yAcceleration"]
    y_acc_min = y_acceleration[0]
    y_acc_max = y_acceleration[1]
    y_velocity = range_dic["yVelocity"]
    y_vel_min = y_velocity[0]
    y_vel_max = y_velocity[1]

    invalid_by_y = set()
    invalid_by_y_velocity = set()
    left = 0
    right = 0
    for key, value in states_dic.items():
        car_id = key
        print(car_id)

        #print(value.shape[0])
        for index in range(0,11):
            curr_y = value["y_ego"].iloc[index]
            #print(curr_y)
            car_central_y = curr_y + (value["height_ego"].iloc[index] / 2)
            lane_central_y = 0.0

            for i in range(0, len(lane_markings) - 1):
                cur = float(lane_markings[i])
                nxt = float(lane_markings[i + 1])
                if cur < car_central_y < nxt:
                    lane_central_y = (cur + nxt) / 2
                    break

            distance = abs(lane_central_y - car_central_y)
            #print(distance)
            if distance > y:
                _invalid_by_y.add(car_id)


            curr_y_velocity = value["yVelocity_ego"].iloc[index]
            if curr_y_velocity > y_vel_max or curr_y_velocity < y_vel_min:
                _invalid_by_y_velocity.add(car_id)



        for index in range(value.shape[0]-11,value.shape[0]):
            curr_y = value["y_ego"].iloc[index]
            #print(curr_y)
            car_central_y = curr_y + (value["height_ego"].iloc[index] / 2)
            lane_central_y = 0.0
            if float(upper_lane_markings[0]) < float(car_central_y) < float(upper_lane_markings[1]):
                lane_central_y = (float(upper_lane_markings[0]) + float(upper_lane_markings[1])) / 2
            elif float(upper_lane_markings[1]) < float(car_central_y) < float(upper_lane_markings[2]):
                lane_central_y = (float(upper_lane_markings[1]) + float(upper_lane_markings[2])) / 2
            elif float(lower_lane_markings[0]) < float(car_central_y) < float(lower_lane_markings[1]):
                lane_central_y = (float(lower_lane_markings[0]) + float(lower_lane_markings[1])) / 2
            elif float(lower_lane_markings[1]) < float(car_central_y) < float(lower_lane_markings[2]):
                lane_central_y = (float(lower_lane_markings[1]) + float(lower_lane_markings[2])) / 2
            distance = abs(lane_central_y - car_central_y)
            #print(distance)
            if distance > y:
                invalid_by_y.add(car_id)


            curr_y_velocity = value["yVelocity_ego"].iloc[index]
            if curr_y_velocity > y_vel_max or curr_y_velocity < y_vel_min:
                invalid_by_y_velocity.add(car_id)

        # Additional tasks: to count whether it turns left or right
        print(value["laneId_ego"].iloc[0])
        print(value["laneId_ego"].iloc[value.shape[0]-1])
        if value["laneId_ego"].iloc[0] < value["laneId_ego"].iloc[value.shape[0]-1]:
            right = right + 1
            print("there")
        elif value["laneId_ego"].iloc[0] > value["laneId_ego"].iloc[value.shape[0]-1]:
            left = left + 1
            print(car_id)



    return invalid_by_y, invalid_by_y_velocity, left, right






def calculate_range(_recording_path, _meta_path, _tracks_path):
    recording_df = pd.read_csv(_recording_path)
    meta_df = pd.read_csv(_meta_path)
    tracks_df = pd.read_csv(_tracks_path)
    # To select vehicles which changed lane once
    selected_vehicles = meta_df[meta_df.numLaneChanges == 0]
    selected_vehicles_id = selected_vehicles["id"]

    # Get all information from tracks_path for selected cars
    selected_vehicles_info = tracks_df[tracks_df.id.isin(selected_vehicles_id)]
    selected_vehicles_info.index = np.arange(0, len(selected_vehicles_info))

    selected_vehicles_has_pos_v = selected_vehicles_info[selected_vehicles_info.yVelocity>0]
    selected_vehicles_has_neg_v = selected_vehicles_info[selected_vehicles_info.yVelocity<0]

    range_dic = {}
    # To calculate y acceleration range
    y_acc_max = selected_vehicles_info["yAcceleration"].max()
    y_acc_min = selected_vehicles_info["yAcceleration"].min()
    range_dic["yAcceleration"] = [y_acc_min, y_acc_max]

    # To calculate y velocity range
    y_vel_max = selected_vehicles_info["yVelocity"].max()
    y_vel_min = selected_vehicles_info["yVelocity"].min()
    y_vel_pos_mean = selected_vehicles_has_pos_v["yVelocity"].mean()
    y_vel_neg_mean = selected_vehicles_has_neg_v["yVelocity"].mean()
    range_dic["yVelocity"] = [y_vel_min, y_vel_max]

    # To calculate y range. Y range is the distance between central axis of car and central axis of lane
    upper_lane_markings = recording_df["upperLaneMarkings"].iloc[0].split(";")
    #print(upper_lane_markings)
    lower_lane_markings = recording_df["lowerLaneMarkings"].iloc[0].split(";")
    #print(lower_lane_markings)
    lane_markings = upper_lane_markings+lower_lane_markings
    y_range = 0.0
    # To keep track of distance between lane and car center, then calculate the mean, median, etc.
    list_of_distance = []
    for index, row in selected_vehicles_info.iterrows():
        car_central_y = row["y"] + (row["height"] / 2)
        lane_central_y = 0.0

        for i in range(0, len(lane_markings)-1):
            cur = float(lane_markings[i])
            nxt = float(lane_markings[i+1])
            if cur < car_central_y <= nxt :
                lane_central_y = (cur+nxt)/2
                break

        y_range = max(abs(lane_central_y - car_central_y), y_range)
        list_of_distance.append(abs(lane_central_y - car_central_y))

    range_dic["y_max"] = y_range
    range_dic["y_mean"] = sum(list_of_distance) / len(list_of_distance)
    print(range_dic["y_mean"])
    #print([y_acc_min, y_acc_max])
    print([y_vel_pos_mean, y_vel_neg_mean])
    return range_dic







# get a dictionary to store data frame for each car, key is carId and value is data frame
def get_states(prefix, file_id):
    directory = os.path.join("/Users/SummerXia/Desktop/Berkeley/highD/", "tables/")
    result = {}
    for root, dirs, files in os.walk(directory):
        for file in files:
            if file.startswith("lcS" + "_R" + file_id):
                file_id = file[5:7]
                car_id = file[8:]
                car_id = car_id[:-4]
                f = pd.read_csv(prefix + file)
                result[file_id+"_"+car_id] = f
    return result




if __name__=="__main__":

    data_prefix = "../data/"
    state_prefix = "../tables/"
    # Additional tasks: to count whether it turns left or right
    left_count = 0
    right_count = 0
    _invalid_by_y_total = set()
    _invalid_by_y_velocity_total = set()
    for file_id in range(10, 11):
        file_id_str = None
        if file_id < 10:
            file_id_str = "0" + str(file_id)
        else:
            file_id_str = str(file_id)
        tracks_path = data_prefix + file_id_str + "_tracks.csv"
        meta_path = data_prefix + file_id_str + "_tracksMeta.csv"
        recording_path = data_prefix + file_id_str + "_recordingMeta.csv"
        states_dic = get_states(state_prefix, file_id_str)
        range_dic = calculate_range(recording_path,meta_path,tracks_path)



        _invalid_by_y, _invalid_by_y_velocity, _left, _right = extract_invalid(states_dic, range_dic, recording_path)
        left_count = left_count + _left
        right_count = right_count + _right
        _invalid_by_y_total.union(_invalid_by_y)
        _invalid_by_y_velocity_total.union(_invalid_by_y_velocity)

    print(left_count)
    print(right_count)
    print(_invalid_by_y_total)
    print(_invalid_by_y_velocity_total)

