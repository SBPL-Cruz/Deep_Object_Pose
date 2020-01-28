
import matplotlib.pyplot as plt
import pandas as pd
import argparse
import sys
import numpy as np

def parse_arguments():
    # Command-line flags are defined here.
    parser = argparse.ArgumentParser()
    parser.add_argument('--pose-file', dest='pose_file', type=str, help="Path of pose file.")

    return parser.parse_args()

if __name__ == '__main__':
    args = parse_arguments()
    # pose_df = pd.read_csv(args.pose_file, delimiter=' ', names=['x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w', 'add'])
    # pose_path = "{}/{}".format("/media/aditya/A69AFABA9AFA85D9/Cruzr/code/DOPE/catkin_ws/src/Deep_Object_Pose/pr2/multi_object", "sugar")
    # pose_path = "{}/{}".format("/media/aditya/A69AFABA9AFA85D9/Cruzr/code/DOPE/catkin_ws/src/Deep_Object_Pose/pr2/multi_object", "mustard")
    # pose_path = "{}/{}".format("/media/aditya/A69AFABA9AFA85D9/Cruzr/code/DOPE/catkin_ws/src/Deep_Object_Pose/pr2/multi_object", "soup")
    pose_path = "{}/{}".format("/media/aditya/A69AFABA9AFA85D9/Cruzr/code/DOPE/catkin_ws/src/Deep_Object_Pose/pr2/multi_object", "drill")
    # pose_icp_df = pd.read_csv("{}/pred_icp_1.txt".format(pose_path), delimiter=' ', names=['x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w', 'add'])
    
    pose_dope_df = pd.read_csv("{}/pred_icp_3.txt".format(pose_path), delimiter=' ', names=['x', 'y', 'z', 'q_x', 'q_y', 'q_z', 'q_w', 'add'])
    pose_dope_df['add'] = pose_dope_df['add'].expanding(min_periods=4).mean();
    pose_dope_df = pose_dope_df.iloc[::4, :]

    # pose_dope_df['x'] = pose_dope_df['x'] - pose_dope_df['x'].expanding(min_periods=4).mean()
    # pose_dope_df['add'] = pose_dope_df['add'].expanding(min_periods=4).std()
    # for i in pose_dope_df.index:
        # print("{} , standard dev moving : {}, y : {}".format(i, pose_dope_df['x'][i], pose_dope_df['y'][i]))
    # print(pose_df)
    # pose_icp_df = pose_icp_df[(pose_icp_df['y'] > 0.8) & (pose_icp_df['y'] < 1.5)]
    # pose_dope_df = pose_dope_df[(pose_dope_df['y'] > 0.8) & (pose_icp_df['y'] < 1.5)]

    # pose_icp_df = pose_icp_df[pose_icp_df['x'] < 0.44]
    # pose_dope_df = pose_dope_df[pose_dope_df['x'] < 0.44]


    plt.figure()
    plt.title("ADD")
    plt.ylabel("ADD")
    plt.xlabel("Decreasing Y")
    # plt.plot(pose_icp_df.index, pose_icp_df['add'], label="DOPE + ICP")
    # plt.plot(pose_dope_df.index, pose_dope_df['add'], label="DOPE")
    plt.plot(pose_dope_df.index, pose_dope_df['add'], 'bo-', label="ICP")
    
    count = 0
    for x, add, y in zip(pose_dope_df.index, pose_dope_df['add'], pose_dope_df['y']):
        if count % 1 == 0:
            label = "{:.2f}".format(y)
            
            plt.annotate(label, # this is the text
                        (x, add), # this is the point to label
                        textcoords="offset points", # how to position the text
                        xytext=(0,10), # distance from text to points (x,y)
                        ha='center') # horizontal alignment can be left, right or center
        count += 1
    # plt.xticks(pose_dope_df.index)
    # plt.yticks(np.arange(0,0.02, 0.001))
    plt.legend()

    # plt.figure()
    # plt.title("Z Coordinate")
    # plt.ylim(0, 1.00)
    # plt.plot(pose_icp_df.index, pose_icp_df['z'], label="DOPE + ICP")
    # plt.plot(pose_dope_df.index, pose_dope_df['z'], label="DOPE")

    # plt.figure()
    # plt.title("X Coordinate")
    # plt.ylim(0, 0.50)
    # # plt.plot(pose_icp_df.index, pose_icp_df['x'], label="DOPE + ICP")
    # plt.plot(pose_dope_df.index, pose_dope_df['x'], label="DOPE")

    # plt.figure()
    # plt.title("Y Coordinate")
    # plt.ylim(0, 2.00)
    # plt.plot(pose_icp_df.index, pose_icp_df['y'], label="DOPE + ICP")
    # plt.plot(pose_dope_df.index, pose_dope_df['y'], label="DOPE")
    plt.savefig("test.png")
    plt.show()

