#!/usr/bin/env python

from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import open3d as o3d
import numpy as np
import random

iterations = 200
threshold = 1

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


def unique(list1):
    unique_list = []
    for x in list1:
        if x not in unique_list:
            unique_list.append(x)
    return unique_list


def perform_ransac(pcd_numpy):
    votes_list = []
    votes_and_planes = []
    for_searching = []
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

        votes_and_planes = [votes, random_point_1,
                            random_point_2, random_point_3]
        for_searching.append(votes_and_planes)

        votes_list.append(votes)

        # if(votes > max_votes):
        #     max_votes = votes
        # three_points = [random_point_1, random_point_2, random_point_3]
        #     final_inliers = inliers
        #     final_outliers = outliers
    votes_list = unique(votes_list)
    votes_list.sort(reverse=True)
    print(votes_list)
    # print(votes_list)

    voting_number_1 = votes_list[0]
    voting_number_2 = votes_list[1]
    voting_number_3 = votes_list[2]

    plane_1 = []
    plane_2 = []
    plane_3 = []

    # print(for_searching)

    for item in for_searching:
        if item[0] == voting_number_1:
            plane_1_a = item[1]
            plane_1_b = item[2]
            plane_1_c = item[3]
            plane_1 = [plane_1_a, plane_1_b, plane_1_c]
        elif item[0] == voting_number_2:
            plane_2_a = item[1]
            plane_2_b = item[2]
            plane_2_c = item[3]
            plane_2 = [plane_2_a, plane_2_b, plane_2_c]
        elif item[0] == voting_number_3:
            plane_3_a = item[1]
            plane_3_b = item[2]
            plane_3_c = item[3]
            plane_3 = [plane_3_a, plane_3_b, plane_3_c]

    return plane_1, plane_2, plane_3


plane_1, plane_2, plane_3 = perform_ransac(pcd_numpy)

point_of_plane_1 = plane_1[0]

print(point_of_plane_1)

normal = normalize(
    np.cross(plane_1[1]-plane_1[0], plane_1[2]-plane_1[0]))

d = -point_of_plane_1.dot(normal)
xx, yy = np.meshgrid(range(10), range(10))
z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

plt3d = plt.figure().gca(projection='3d')
plt3d.plot_surface(xx, yy, z, alpha=0.2)

plt.show()

point_of_plane_2 = plane_2[0]

print(point_of_plane_2)

normal = normalize(
    np.cross(plane_2[1]-plane_2[0], plane_2[2]-plane_2[0]))

d = -point_of_plane_2.dot(normal)
xx, yy = np.meshgrid(range(10), range(10))
z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

plt3d = plt.figure().gca(projection='3d')
plt3d.plot_surface(xx, yy, z, alpha=0.2)

plt.show()

point_of_plane_3 = plane_3[0]

print(point_of_plane_3)

normal = normalize(
    np.cross(plane_3[1]-plane_3[0], plane_3[2]-plane_3[0]))

d = -point_of_plane_3.dot(normal)
xx, yy = np.meshgrid(range(10), range(10))
z = (-normal[0] * xx - normal[1] * yy - d) * 1. / normal[2]

plt3d = plt.figure().gca(projection='3d')
plt3d.plot_surface(xx, yy, z, alpha=0.2)

plt.show()

# for point_of_plane_1 in inliers_pcd:
# ax = plt3d.add_subplot(111, projection='3d')
# ax.plot_surface(xx, yy, z, alpha=0.2)
# plt3d.scatter(point_of_plane_1[0], point_of_plane_1[1], point_of_plane_1[2],  color='green')
# print("printing point_of_plane_1 : ", point_of_plane_1)
# break
