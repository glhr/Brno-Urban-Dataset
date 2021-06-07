import numpy as np
import open3d as o3d
import cv2

intrinsics = np.array([[1.3763974496214614e+03, 0.0000000000000000e+00, 9.5923661859185984e+02],
            [0.0000000000000000e+00, 1.3771721669205588e+03, 5.7674337567054886e+02],
            [0.0000000000000000e+00, 0.0000000000000000e+00, 1.0000000000000000e+00]])

dist = np.array([-1.7245433170404437e-01, 1.1239016157509915e-01, -1.5253720111333031e-04, 3.8487261087130094e-04, -2.0770262391598841e-02])

translation_vec = np.array([0.0, 0.0, 0.0])
rotation_vec =  np.array([0.0, 0.0, 0.0])

if __name__ == "__main__":

    print("Load a ply point cloud, print it, and render it")
    pcd = o3d.io.read_point_cloud("data/session_2/2_2_1_2/lidar_left/scans/scan000000.pcd")
    print(pcd)
    points = np.asarray(pcd.points)
    print(np.unique(points))
    #o3d.visualization.draw_geometries([pcd])

    projected_points, _ = cv2.projectPoints(points,rotation_vec,translation_vec, intrinsics,dist)
    print(np.min(projected_points))
