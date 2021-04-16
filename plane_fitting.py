#!/usr/bin/env python

import open3d as o3d
import numpy as np

pcd = o3d.io.read_point_cloud("room_scan2.pcd")
pcd = pcd.voxel_down_sample(0.2)

# original_size = len(np.asarray(pcd.points))

while(1):
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.3,
                                             ransac_n=5000,
                                             num_iterations=100)

    # inliers_size = len(np.asarray(inliers.points))
    # if(inliers_size < original_size/10):
    #     break

    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    inlier_cloud = pcd.select_by_index(inliers)
    inlier_cloud.paint_uniform_color([1.0, 0, 0])
    outlier_cloud = pcd.select_by_index(inliers, invert=True)
    outlier_cloud.paint_uniform_color([0, 1, 0])
    o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])
    pcd = outlier_cloud

print('open3d working')
