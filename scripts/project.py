import os
import open3d as o3d
import cv2
import numpy as np
import matplotlib.pyplot as plt

TOOL_PATH = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
DATA_PATH = 'data'
RESULT_PATH = 'result'

IMG_NAME = 'test.png'
PCD_NAME = 'test.pcd'

CAMERA_MATRIX = np.array([[1085.8801, 0, 1255.37351],
                          [0, 1087.46558, 747.00803],
                          [0, 0, 1]], dtype=np.float32)

DIST_COEFFS = np.array([-0.084180, 0.000464, 0.000143, -0.001763, 0.000000], dtype=np.float32)

def project(pc_path, img_path, transform_path):
    pcd = o3d.io.read_point_cloud(pc_path)
    img = cv2.imread(img_path)

    data = np.load(transform_path)
    R = data['r']
    t = data['t']

    points3D = np.asarray(pcd.points)
    points2D, _ = cv2.projectPoints(points3D, R, t, CAMERA_MATRIX, DIST_COEFFS)

    xyz = []
    rgb = []


    for point2d,point3d in zip(points2D,points3D):
        if (point2d[:, 0] >= 0) & (point2d[:, 1] >= 0) & \
        (point2d[:, 0] < img.shape[1]) & (point2d[:, 1] < img.shape[0]):
            
            rgb_ = img[point2d[:, 1].astype(int), point2d[:, 0].astype(int)]

            rgb_ = rgb_.flatten().tolist()[::-1]

            rgb_ = [ x / 255 for x in rgb_]

            # rgb.append(rgb_.flatten().tolist()[::-1])
            rgb.append(rgb_)
            xyz.append([point3d[0],point3d[1],point3d[2]]) 

    color_pc = o3d.geometry.PointCloud()

    color_pc.points = o3d.utility.Vector3dVector(np.array(xyz))
    color_pc.colors = o3d.utility.Vector3dVector(np.array(rgb))


    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window(width=800, height=600, left=500, top = 200)
    vis.add_geometry(color_pc)

    # setting backgroud black 
    render_option = vis.get_render_option()
    render_option.background_color = np.asarray([0, 0, 0])  
    render_option.point_size = 1.0


    vis.run()  # user picks points


if __name__ == '__main__':
    img_path = os.path.join(TOOL_PATH, os.path.join(DATA_PATH, IMG_NAME))
    pc_path = os.path.join(TOOL_PATH, os.path.join(DATA_PATH, PCD_NAME))
    transform_path = os.path.join(TOOL_PATH, os.path.join(RESULT_PATH, "extrinsics.npz"))

    project(pc_path, img_path, transform_path)
