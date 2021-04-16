#!/usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import random

iterations = 2000
threshold = 0.2

pcd = o3d.io.read_point_cloud("lego.pcd")
pcd = pcd.voxel_down_sample(0.2)

pcd_numpy = np.asarray(pcd.points)


def normalize(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return v / norm


def check_distance(n, p, x):
    if(abs(n[0]*(x[0]-p[0])+n[1]*(x[1]-p[1])+n[2]*(x[2]-p[2])) < threshold):
        return True
    else:
        return False


def calculate_votes(pcd_numpy, normal, p):
    votes = 0
    inliers = []
    outliers = []
    for point in pcd_numpy:
        if(check_distance(normal, p, point)):
            votes = votes+1
            inliers.append(point)
        else:
            outliers.append(point)
    return votes, inliers, outliers


def perform_ransac(pcd_numpy):
    max_votes = 0
    three_points = []
    final_inliers = []
    final_outliers = []

    for i in range(iterations):
        random_1 = random.randint(0, pcd_numpy.shape[0])
        random_2 = random.randint(0, pcd_numpy.shape[0])
        random_3 = random.randint(0, pcd_numpy.shape[0])

        random_point_1 = pcd_numpy[random_1-1]
        random_point_2 = pcd_numpy[random_2-1]
        random_point_3 = pcd_numpy[random_3-1]

        normal_vector = np.cross(
            random_point_2-random_point_1, random_point_3-random_point_1)

        normalized_normal_vec = normalize(normal_vector)

        votes, inliers, outliers = calculate_votes(
            pcd_numpy, normal_vector, random_point_1)

        if(votes > max_votes):
            max_votes = votes
            three_points = [random_point_1, random_point_2, random_point_3]
            final_inliers = inliers
            final_outliers = outliers

    return three_points, final_inliers, final_outliers


plane_points, inliers_pcd, outliers_pcd = perform_ransac(pcd_numpy)
# print(plane_points)

point = plane_points[0]
normal = normalize(
    np.cross(plane_points[1]-plane_points[0], plane_points[2]-plane_points[0]))

d = -point.dot(normal)
xx, yy = np.meshgrid(range(10), range(10))
z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

plt3d = plt.figure().gca(projection='3d')
plt3d.plot_surface(xx, yy, z, alpha=0.2)
for point in inliers_pcd:
    # ax = plt3d.add_subplot(111, projection='3d')
    # ax.plot_surface(xx, yy, z, alpha=0.2)
    # plt3d.scatter(point[0], point[1], point[2],  color='green')
    # print("printing point : ", point)
    break
plt.show()
