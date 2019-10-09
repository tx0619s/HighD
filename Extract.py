import numpy as np
import pandas as pd
import os.path


def extract(_recording_path, _meta_path, _tracks_path, _file_id, _range_dic):
    recording_path = pd.read_csv(_recording_path)
    meta_path = pd.read_csv(_meta_path)
    tracks_path = pd.read_csv(_tracks_path)
    #print(tracks_df.head(3))
    upper_lane_markings = recording_path["upperLaneMarkings"].iloc[0].split(";")
    lower_lane_markings = recording_path["lowerLaneMarkings"].iloc[0].split(";")
    lane_markings = upper_lane_markings + lower_lane_markings

    # To select vehicles which changed lane once
    selected_vehicles = meta_path[meta_path.numLaneChanges == 1]
    selected_vehicles_id = selected_vehicles["id"]

    # Get all information from tracks_path for selected cars
    selected_vehicles_info = tracks_path[tracks_path.id.isin(selected_vehicles_id)]
    selected_vehicles_info.index = np.arange(0, len(selected_vehicles_info))

    gaps_df = extract_moment(selected_vehicles_info, tracks_path, _meta_path, _file_id,_range_dic, lane_markings)



def extract_moment(selected_vehicles_info, tracks_path, _meta_path, _file_id, _range_dic, lane_markings):
    d ={}
    name_list = ["gaps_df_vel", "gaps_df_dis"]
    # To get gaps between surrounding cars and ego car, accelerations and relative velocity
    for name in name_list:
        d[name] = pd.DataFrame(columns=['veh_id',
                                        'ego_lane_id',
                                        'ego_lane_pre_veh_id',
                                        'ego_lane_pre_veh_x',
                                        'ego_lane_pre_veh_v',
                                        'ego_lane_fol_veh_id',
                                        'ego_lane_fol_veh_x',
                                        'ego_lane_fol_veh_v',
                                        'new_lane_id',
                                        'new_lane_pre_veh_id',
                                        'new_lane_pre_veh_x',
                                        'new_lane_pre_veh_v',
                                        'new_lane_alo_veh_id',
                                        'new_lane_alo_veh_x',
                                        'new_lane_alo_veh_v',
                                        'new_lane_fol_veh_id',
                                        'new_lane_fol_veh_x',
                                        'new_lane_fol_veh_v'
                                        ])
    gaps_df = pd.DataFrame(columns=['frame', # The moment
                                        'veh_id',  # Ego vehicle ID
                                        'ego_lane_id',  # The current lane ID
                                        'ego_lane_pre_veh_id',  # Preceding vehicle ID in the current lane
                                        'ego_lane_pre_dis',  # Distance to preceding car in the current lane
                                        'v_ego_pre',  # The relative velocity between ego and preceding vehicle
                                        'ego_lane_fol_veh_id',  # Following vehicle ID in the current lane
                                        'ego_lane_fol_dis',  # Distance to following car in the current lane
                                        'v_ego_fol',  # The relative velocity between ego and following vehicle
                                        'new_lane_id',  # New lane ID after changing lane
                                        'new_lane_pre_veh_id',  # Preceding vehicle ID in the new lane
                                        'new_lane_pre_dis',  # Distance to preceding car in the new lane
                                        'new_v_ego_pre',  # The relative velocity between ego and preceding vehicle
                                        'new_lane_fol_veh_id',  # Following vehicle ID in the new lane
                                        'new_lane_fol_dis',  # Distance to following car in the new lane
                                        'new_v_ego_fol'  # The relative velocity between ego and following vehicle
                                        ])

    curr_car_id = selected_vehicles_info["id"].iloc[0]
    curr_lane = selected_vehicles_info["laneId"].iloc[0]


    # 1) Extract the moment when its laneId just changed
    for index, row in selected_vehicles_info.iterrows():
        if row['id'] == curr_car_id and row['laneId'] != curr_lane:
            prev_frame = selected_vehicles_info["frame"].iloc[index - 1]
            curr_frame = row["frame"]
            prev_side_info = get_side_info(curr_car_id, prev_frame, tracks_path)
            curr_side_info = get_side_info(curr_car_id, curr_frame, tracks_path)
            ego_lane_pre_veh_id = prev_side_info["ego_preceding"]["id"]
            ego_lane_fol_veh_id = prev_side_info["ego_following"]["id"]
            new_lane_pre_veh_id = curr_side_info["ego_preceding"]["id"]
            new_lane_fol_veh_id = curr_side_info["ego_following"]["id"]
            ego_lane_pre_dis = ego_lane_fol_dis = new_lane_pre_dis = new_lane_fol_dis \
                = v_ego_pre = v_ego_fol = new_v_ego_pre = new_v_ego_fol = -1
            ego_x = selected_vehicles_info["x"].iloc[index - 1]
            ego_v = selected_vehicles_info["xVelocity"].iloc[index - 1]
            new_x = row["x"]
            new_v = row["xVelocity"]
            if ego_lane_pre_veh_id != 0:
                ego_preceding_veh = prev_side_info["ego_preceding"]
                ego_lane_pre_veh_x = ego_preceding_veh["x"]
                ego_lane_pre_dis = abs(ego_lane_pre_veh_x - ego_x)
                ego_lane_pre_veh_v = ego_preceding_veh["xVelocity"]
                v_ego_pre = ego_v - ego_lane_pre_veh_v
            if ego_lane_fol_veh_id != 0:
                ego_following_veh = prev_side_info["ego_following"]
                ego_lane_fol_veh_x = ego_following_veh["x"]
                ego_lane_fol_dis = abs(ego_lane_fol_veh_x - ego_x)
                ego_lane_fol_veh_v = ego_following_veh["xVelocity"]
                v_ego_fol = ego_v - ego_lane_fol_veh_v
            if new_lane_pre_veh_id != 0:
                new_preceding_veh = curr_side_info["ego_preceding"]
                new_lane_pre_veh_x = new_preceding_veh["x"]
                new_lane_pre_dis = abs(new_lane_pre_veh_x - new_x)
                new_lane_pre_veh_v = new_preceding_veh["xVelocity"]
                new_v_ego_pre = new_v - new_lane_pre_veh_v
            if new_lane_fol_veh_id != 0:
                new_following_veh = curr_side_info["ego_following"]
                new_lane_fol_veh_x = new_following_veh["x"]
                new_lane_fol_dis = abs(new_lane_fol_veh_x - new_x)
                new_lane_fol_veh_v = new_following_veh["xVelocity"]
                new_v_ego_fol = new_v - new_lane_fol_veh_v
            new_lane_id = row['laneId']
            buffer = pd.DataFrame({'frame': [row["frame"]], "veh_id": [curr_car_id], "ego_lane_id": [curr_lane], "ego_lane_pre_veh_id":
                [ego_lane_pre_veh_id], "ego_lane_pre_dis": [ego_lane_pre_dis], "v_ego_pre": [v_ego_pre],
                                   "ego_lane_fol_veh_id": [ego_lane_fol_veh_id], "ego_lane_fol_dis": [ego_lane_fol_dis],
                                   "v_ego_fol": [v_ego_fol],
                                   "new_lane_id": [new_lane_id], "new_lane_pre_veh_id": [new_lane_pre_veh_id],
                                   "new_lane_pre_dis": [new_lane_pre_dis], "new_v_ego_pre": [new_v_ego_pre],
                                   "new_lane_fol_veh_id": [new_lane_fol_veh_id], "new_lane_fol_dis": [new_lane_fol_dis],
                                   "new_v_ego_fol": [new_v_ego_fol]})
            gaps_df = gaps_df.append(buffer, ignore_index=True)
            curr_lane = new_lane_id

        elif row['id'] != curr_car_id:
            curr_car_id = row['id']
            curr_lane = row['laneId']


    # 2) Extract the moment when it just have intention
    curr_car_id = selected_vehicles_info["id"].iloc[0]
    curr_lane = selected_vehicles_info["laneId"].iloc[0]
    is_checked_dis = False
    is_checked_vel = False

    for index, row in selected_vehicles_info.iterrows():
        if row['id'] == curr_car_id and row['laneId'] == curr_lane and \
                (not is_checked_dis) and (not is_checked_vel):
            tmp = gaps_df[gaps_df["veh_id"] == curr_car_id]
            new_lane_id = tmp["new_lane_id"].iloc[0]
            if new_lane_id > curr_lane:
                right_turn = True
            else:
                right_turn = False
            # based on velocity
            if right_turn and row['yVelocity'] > range_dic['yVelocity'][0]:
                moment_info = get_side_info(curr_car_id, row['frame'], tracks_path)
                buffer = pd.DataFrame({'veh_id': [curr_car_id],
                                       'ego_lane_id': [curr_lane],
                                       'ego_lane_pre_veh_id': [moment_info["ego_preceding"]["id"]],
                                       'ego_lane_pre_veh_x': [moment_info["ego_preceding"]["x"]],
                                       'ego_lane_pre_veh_v': [moment_info["ego_preceding"]["xVelocity"]],
                                       'ego_lane_fol_veh_id': [moment_info["ego_following"]["id"]],
                                       'ego_lane_fol_veh_x': [moment_info["ego_following"]["x"]],
                                       'ego_lane_fol_veh_v': [moment_info["ego_following"]["xVelocity"]],
                                       'new_lane_id': [new_lane_id],
                                       'new_lane_pre_veh_id': [moment_info["right_preceding"]["id"]],
                                       'new_lane_pre_veh_x': [moment_info["right_preceding"]["x"]],
                                       'new_lane_pre_veh_v': [moment_info["right_preceding"]["xVelocity"]],
                                       'new_lane_alo_veh_id': [moment_info["right_alongside"]["id"]],
                                       'new_lane_alo_veh_x': [moment_info["right_alongside"]["x"]],
                                       'new_lane_alo_veh_v': [moment_info["right_alongside"]["xVelocity"]],
                                       'new_lane_fol_veh_id': [moment_info["right_following"]["id"]],
                                       'new_lane_fol_veh_x': [moment_info["right_following"]["x"]],
                                       'new_lane_fol_veh_v': [moment_info["right_following"]["xVelocity"]]
                                       })
                is_checked_vel = True
                d["gaps_df_vel"] = d["gaps_df_vel"].append(buffer, ignore_index=True)
            elif (not right_turn) and row['yVelocity'] < range_dic['yVelocity'][1]:
                buffer = pd.DataFrame({'veh_id': [curr_car_id],
                                       'ego_lane_id': [curr_lane],
                                       'ego_lane_pre_veh_id': [moment_info["ego_preceding"]["id"]],
                                       'ego_lane_pre_veh_x': [moment_info["ego_preceding"]["x"]],
                                       'ego_lane_pre_veh_v': [moment_info["ego_preceding"]["xVelocity"]],
                                       'ego_lane_fol_veh_id': [moment_info["ego_following"]["id"]],
                                       'ego_lane_fol_veh_x': [moment_info["ego_following"]["x"]],
                                       'ego_lane_fol_veh_v': [moment_info["ego_following"]["xVelocity"]],
                                       'new_lane_id': [new_lane_id],
                                       'new_lane_pre_veh_id': [moment_info["left_preceding"]["id"]],
                                       'new_lane_pre_veh_x': [moment_info["left_preceding"]["x"]],
                                       'new_lane_pre_veh_v': [moment_info["left_preceding"]["xVelocity"]],
                                       'new_lane_alo_veh_id': [moment_info["left_alongside"]["id"]],
                                       'new_lane_alo_veh_x': [moment_info["left_alongside"]["x"]],
                                       'new_lane_alo_veh_v': [moment_info["left_alongside"]["xVelocity"]],
                                       'new_lane_fol_veh_id': [moment_info["left_following"]["id"]],
                                       'new_lane_fol_veh_x': [moment_info["left_following"]["x"]],
                                       'new_lane_fol_veh_v': [moment_info["left_following"]["xVelocity"]]
                                       })
                is_checked_vel = True
                d["gaps_df_vel"] = d["gaps_df_vel"].append(buffer, ignore_index=True)


            # based on distance
            car_central_y = row["y"] + (row["height"] / 2)
            lane_central_y = 0.0
            for i in range(0, len(lane_markings) - 1):
                cur = float(lane_markings[i])
                nxt = float(lane_markings[i + 1])
                if cur < car_central_y <= nxt:
                    lane_central_y = (cur + nxt) / 2
                    break
            if right_turn and car_central_y - lane_central_y > range_dic["y_mean"]:
                moment_info = get_side_info(curr_car_id, row['frame'], tracks_path)
                buffer = pd.DataFrame({'veh_id': [curr_car_id],
                                       'ego_lane_id': [curr_lane],
                                       'ego_lane_pre_veh_id': [moment_info["ego_preceding"]["id"]],
                                       'ego_lane_pre_veh_x': [moment_info["ego_preceding"]["x"]],
                                       'ego_lane_pre_veh_v': [moment_info["ego_preceding"]["xVelocity"]],
                                       'ego_lane_fol_veh_id': [moment_info["ego_following"]["id"]],
                                       'ego_lane_fol_veh_x': [moment_info["ego_following"]["x"]],
                                       'ego_lane_fol_veh_v': [moment_info["ego_following"]["xVelocity"]],
                                       'new_lane_id': [new_lane_id],
                                       'new_lane_pre_veh_id': [moment_info["right_preceding"]["id"]],
                                       'new_lane_pre_veh_x': [moment_info["right_preceding"]["x"]],
                                       'new_lane_pre_veh_v': [moment_info["right_preceding"]["xVelocity"]],
                                       'new_lane_alo_veh_id': [moment_info["right_alongside"]["id"]],
                                       'new_lane_alo_veh_x': [moment_info["right_alongside"]["x"]],
                                       'new_lane_alo_veh_v': [moment_info["right_alongside"]["xVelocity"]],
                                       'new_lane_fol_veh_id': [moment_info["right_following"]["id"]],
                                       'new_lane_fol_veh_x': [moment_info["right_following"]["x"]],
                                       'new_lane_fol_veh_v': [moment_info["right_following"]["xVelocity"]]
                                       })
                is_checked_dis = True
                d["gaps_df_dis"] = d["gaps_df_dis"].append(buffer, ignore_index=True)
            elif (not right_turn) and lane_central_y - car_central_y > range_dic["y_mean"]:
                buffer = pd.DataFrame({'veh_id': [curr_car_id],
                                       'ego_lane_id': [curr_lane],
                                       'ego_lane_pre_veh_id': [moment_info["ego_preceding"]["id"]],
                                       'ego_lane_pre_veh_x': [moment_info["ego_preceding"]["x"]],
                                       'ego_lane_pre_veh_v': [moment_info["ego_preceding"]["xVelocity"]],
                                       'ego_lane_fol_veh_id': [moment_info["ego_following"]["id"]],
                                       'ego_lane_fol_veh_x': [moment_info["ego_following"]["x"]],
                                       'ego_lane_fol_veh_v': [moment_info["ego_following"]["xVelocity"]],
                                       'new_lane_id': [new_lane_id],
                                       'new_lane_pre_veh_id': [moment_info["left_preceding"]["id"]],
                                       'new_lane_pre_veh_x': [moment_info["left_preceding"]["x"]],
                                       'new_lane_pre_veh_v': [moment_info["left_preceding"]["xVelocity"]],
                                       'new_lane_alo_veh_id': [moment_info["left_alongside"]["id"]],
                                       'new_lane_alo_veh_x': [moment_info["left_alongside"]["x"]],
                                       'new_lane_alo_veh_v': [moment_info["left_alongside"]["xVelocity"]],
                                       'new_lane_fol_veh_id': [moment_info["left_following"]["id"]],
                                       'new_lane_fol_veh_x': [moment_info["left_following"]["x"]],
                                       'new_lane_fol_veh_v': [moment_info["left_following"]["xVelocity"]]
                                       })
                is_checked_dis = True
                d["gaps_df_dis"] = d["gaps_df_dis"].append(buffer, ignore_index=True)

        elif row['id'] != curr_car_id:
            curr_car_id = row['id']
            curr_lane = row['laneId']
            is_checked_vel = False
            is_checked_dis = False

    #print(gaps_df.shape)

    # Convert to CSV
    gaps_df.to_csv(r'/Users/SummerXia/Desktop/Berkeley/DAgger/output/gaps/' + _file_id + '_gaps_df.csv')
    d["gaps_df_dis"].to_csv(r'/Users/SummerXia/Desktop/Berkeley/DAgger/output/gaps/' + _file_id + '_gaps_df_dis.csv')
    d["gaps_df_vel"].to_csv(r'/Users/SummerXia/Desktop/Berkeley/DAgger/output/gaps/' + _file_id + '_gaps_df_vel.csv')
    return gaps_df






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
    range_dic["yVelocity"] = [y_vel_neg_mean, y_vel_pos_mean]

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



# given the vehicle ID and current moment, get its surrounding information
# Packed as dictionary to output
def get_side_info(v_id, frame, _tracks_path):
    tmp = _tracks_path[_tracks_path.frame == frame]
    target_row = tmp[tmp.id == v_id]
    ego_preceding_id = target_row["precedingId"].iloc[0]
    ego_following_id = target_row["followingId"].iloc[0]
    left_preceding_id = target_row["leftPrecedingId"].iloc[0]
    left_alongside_id = target_row["leftAlongsideId"].iloc[0]
    left_following_id = target_row["leftFollowingId"].iloc[0]
    right_preceding_id = target_row["rightPrecedingId"].iloc[0]
    right_alongside_id = target_row["rightAlongsideId"].iloc[0]
    right_following_id = target_row["rightFollowingId"].iloc[0]
    result = {
        "ego_preceding": get_this_veh_info(ego_preceding_id, frame, _tracks_path),
        "ego_following": get_this_veh_info(ego_following_id, frame, _tracks_path),
        "left_preceding": get_this_veh_info(left_preceding_id, frame, _tracks_path),
        "left_alongside": get_this_veh_info(left_alongside_id, frame, _tracks_path),
        "left_following": get_this_veh_info(left_following_id, frame, _tracks_path),
        "right_preceding": get_this_veh_info(right_preceding_id, frame, _tracks_path),
        "right_alongside": get_this_veh_info(right_alongside_id, frame, _tracks_path),
        "right_following": get_this_veh_info(right_following_id, frame, _tracks_path)
    }
    return result


# given the vehicle ID and current moment, get this vehicle information
# Packed as dictionary to output
def get_this_veh_info(v_id, frame, _tracks_path):
    tmp = _tracks_path[_tracks_path.frame == frame]
    target_row = tmp[tmp["id"] == v_id]
    result = {
        "id":            v_id,
        "x":             0 if target_row.shape[0] == 0 else target_row["x"].iloc[0],
        "y":             0 if target_row.shape[0] == 0 else target_row["y"].iloc[0],
        "xVelocity":     0 if target_row.shape[0] == 0 else target_row["xVelocity"].iloc[0],
        "yVelocity":     0 if target_row.shape[0] == 0 else target_row["yVelocity"].iloc[0],
        "xAcceleration": 0 if target_row.shape[0] == 0 else target_row["xAcceleration"].iloc[0],
        "yAcceleration": 0 if target_row.shape[0] == 0 else target_row["yAcceleration"].iloc[0],
        "laneId":        0 if target_row.shape[0] == 0 else target_row["laneId"].iloc[0]
    }
    return result



if __name__ == "__main__":
    data_prefix = "/Users/SummerXia/Desktop/Berkeley/highD/data/"
    for file_id in range(5, 6):
        file_id_str = None
        if file_id < 10:
            file_id_str = "0" + str(file_id)
        else:
            file_id_str = str(file_id)
        tracks_path_file = data_prefix + file_id_str + "_tracks.csv"
        meta_path_file = data_prefix + file_id_str + "_tracksMeta.csv"
        recording_path_file = data_prefix + file_id_str + "_recordingMeta.csv"
        # preprocessed tracks file to save time
        # tracks_is_file = os.path.isfile(tracks_path_file)
        # meta_is_file = os.path.isfile(meta_path_file)
        # recording_is_file = os.path.isfile(recording_path_file)
        # complete_file = tracks_is_file and meta_is_file and recording_is_file
        # if not complete_file:
        #     continue
        range_dic = calculate_range(recording_path_file, meta_path_file, tracks_path_file)
        extract(recording_path_file, meta_path_file, tracks_path_file, file_id_str, range_dic)
        #extract_process(recording_path_file, meta_path_file, tracks_path_file, file_id_str)

