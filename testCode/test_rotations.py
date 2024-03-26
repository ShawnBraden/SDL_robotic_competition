import numpy as np
import math
import time
import cv2
import pyrealsense2 as rs
from global_map import global_map_obj

def rotation_matrix_x(angle):
    """
    Returns the rotation matrix for rotation about the x-axis by the given angle (in radians).
    """
    return np.array([[1, 0, 0],
                     [0, np.cos(angle), -np.sin(angle)],
                     [0, np.sin(angle), np.cos(angle)]])

def rotation_matrix_y(angle):
    """
    Returns the rotation matrix for rotation about the y-axis by the given angle (in radians).
    """
    return np.array([[np.cos(angle), 0, np.sin(angle)],
                     [0, 1, 0],
                     [-np.sin(angle), 0, np.cos(angle)]])

def rotation_matrix_z(theta):
    """
    Generate a 3D rotation matrix for rotation around the z-axis (yaw) by angle theta (in radians).
    
    Parameters:
        theta (float): Angle of rotation in radians.
        
    Returns:
        numpy.ndarray: 3x3 rotation matrix.
    """
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta), 0],
                     [0, 0, 1]])


def rotate_on_camera_pos(camera_pos, messarued_vector, theata, beta):
    # We need to translate the cammers refreance to ours
    x = messarued_vector[2]
    y = messarued_vector[1]
    z = messarued_vector[0]

    messarued_vector = [x, y, z]

    # print(messarued_vector)

    rotation_z = np.dot(rotation_matrix_z(theata), messarued_vector)
    # print(rotation_z)
    rotation_z_y = np.dot(rotation_matrix_y(beta), rotation_z)
    # print(rotation_z_y)

    return camera_pos + rotation_z_y

if __name__ == '__main__':
    map_obj = global_map_obj(resolution=0.01)

    pos = np.array([0, 0, 0])
    # data_point = np.array([1, 2, 3])
    # theata =  - np.pi / 2 # up down angle of the cammera
    theata =  0 # up down angle of the cammera
    #beta = np.pi / 2  # rotations of the cammera
    beta = 0  # rotations of the cammera
    # val = rotate_on_camera_pos(pos, data_point, theata, -beta)
    # print(f"Golbal locations {val}, camera pos {pos}, data point {data_point}, theta {theata} beta {beta}")

    # Configure depth and color streams
    pipeline = rs.pipeline()    
    config = rs.config()    
        
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

    # Get stream profile and camera intrinsics
    profile = pipeline.get_active_profile()
    depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
    depth_intrinsics = depth_profile.get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    # # Processing blocks
    pc = rs.pointcloud()
    decimate = rs.decimation_filter()
    decimate.set_option(rs.option.filter_magnitude, 2)
    colorizer = rs.colorizer()

    # Wait for a coherent pair of frames: depth and color
    frames = pipeline.wait_for_frames()

    depth_frame = frames.get_depth_frame()
    color_frame = frames.get_color_frame()

    depth_frame = decimate.process(depth_frame)

    # Grab new intrinsics (may be changed by decimation)
    depth_intrinsics = rs.video_stream_profile(
        depth_frame.profile).get_intrinsics()
    w, h = depth_intrinsics.width, depth_intrinsics.height

    depth_image = np.asanyarray(depth_frame.get_data())
    color_image = np.asanyarray(color_frame.get_data())

    depth_colormap = np.asanyarray(
        colorizer.colorize(depth_frame).get_data())

    # if state.color:
    #     mapped_frame, color_source = color_frame, color_image
    # else:
    mapped_frame, color_source = depth_frame, depth_colormap

    points = pc.calculate(depth_frame)
    pc.map_to(mapped_frame)

    # Pointcloud data to arrays
    v, t = points.get_vertices(), points.get_texture_coordinates()
    verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
    texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
    
    # file = open('test.txt', 'w')
    for v_list in verts:
        val = rotate_on_camera_pos(pos, v_list, theata, -beta)
        # file.write(f"{val}\n")
        map_obj.set_pos(val[0], val[1], val[2])
    # file.close()
    map_obj.display_graph()
