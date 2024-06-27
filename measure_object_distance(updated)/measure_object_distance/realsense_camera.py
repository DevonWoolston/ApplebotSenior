#!/usr/bin/env python3
import pyrealsense2 as rs
import numpy as np
import cv2

class RealsenseCamera:
    def __init__(self):
        # Configure depth and color streams
        print("Loading Intel Realsense Camera")
        self.pipeline = rs.pipeline()

        config = rs.config()
        config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

        # Start streaming
        profile = self.pipeline.start(config)
        align_to = rs.stream.color
        self.align = rs.align(align_to)

        # Get intrinsics
        intr = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        self.camera_info = self.CameraInfo(intr)

    class CameraInfo:
        def __init__(self, intrinsics):
            self.width = intrinsics.width
            self.height = intrinsics.height
            self.K = [intrinsics.fx, 0, intrinsics.ppx, 0, intrinsics.fy, intrinsics.ppy, 0, 0, 1]
            self.D = [0, 0, 0, 0, 0]  # Assuming no distortion for simplicity

    def get_frame_stream(self):
        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            # If there is no frame, probably camera not connected, return False
            print("Error, impossible to get the frame, make sure that the Intel Realsense camera is correctly connected")
            return False, None, None
        
        # Apply filter to fill the Holes in the depth image
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.holes_fill, 3)
        filtered_depth = spatial.process(depth_frame)

        hole_filling = rs.hole_filling_filter()
        filled_depth = hole_filling.process(filtered_depth)

        # Convert images to numpy arrays
        depth_image = np.asanyarray(filled_depth.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        return True, color_image, depth_image
    
    def release(self):
        self.pipeline.stop()

    def convert_depth_to_phys_coord_using_realsense(self, x, y, depth):
        _intrinsics = rs.intrinsics()
        _intrinsics.width = self.camera_info.width
        _intrinsics.height = self.camera_info.height
        _intrinsics.ppx = self.camera_info.K[2]
        _intrinsics.ppy = self.camera_info.K[5]
        _intrinsics.fx = self.camera_info.K[0]
        _intrinsics.fy = self.camera_info.K[4]
        _intrinsics.model = rs.distortion.none  # Assuming no distortion
        _intrinsics.coeffs = self.camera_info.D

        result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
        # result[0]: right, result[1]: down, result[2]: forward
        return result[0], result[1], result[2]

