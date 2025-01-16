import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

file_path = "test_dis_145cm.txt"

import os

directory = './'

for filename in os.listdir(directory):
    if filename.startswith('test_') and filename.endswith('.txt'):
        file_path = filename
        data = pd.read_csv(file_path)

        test_type = None
        if "daylight" in file_path:
            test_type = 'day'
        else:
            test_type = 'night'

        distance = file_path.split('_')[2]
        distance = distance.split('.')[0]

        #clean column names
        data.columns = data.columns.str.strip()

        unique_inlier_deviation = sorted(data['DISTANCE_THRESHOLD'].unique())
        unique_angle_deviation = sorted(data['ANGLE_DVA'].unique())
        colors = sns.color_palette("Set2", len(data['INLIER_NUM'].unique()))

        fig, axes = plt.subplots(1, len(unique_inlier_deviation), figsize=(18, 6), sharey=True)
        fig.suptitle(f"RANSAC Performance: {test_type} ambient light, distance {distance}", fontsize=16)

        for ax, inlier_dev in zip(axes, unique_inlier_deviation):
            subset = data[data['DISTANCE_THRESHOLD'] == inlier_dev]
            for angle, line_style in zip(unique_angle_deviation, ['-', '--', ':']):
                for inliers_num, color in zip(sorted(subset['INLIER_NUM'].unique()), colors):
                    filtered = subset[(subset['ANGLE_DVA'] == angle) & (subset['INLIER_NUM'] == inliers_num)]
                    ax.plot(filtered['MAX_ITERATION'], filtered['CORR_DETECT_SCAN_NUM'],
                            label=f"#Inliers: {inliers_num}, $\\Delta\\theta$: {angle}",
                            color=color, linestyle=line_style)

            ax.set_title(f"Inlier Deviation: {inlier_dev}")
            ax.set_xlabel("Iterations")
            ax.set_ylabel("Correct Observations")
            #ax.legend(fontsize=8, loc='upper left', bbox_to_anchor=(1, 1))  # Adjust legend location
            ax.grid(True)
        plt.legend(fontsize=12, loc='upper left', bbox_to_anchor=(1, 1))  # Adjust legend location
        plt.tight_layout()
        plt.savefig(f"{test_type}{distance}.svg", format='svg')
        plt.show()
