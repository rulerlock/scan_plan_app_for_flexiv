# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

"""
OpenCV and Numpy Point cloud Software Renderer

Usage:
------
Mouse: 
    Drag with left button to rotate around pivot (thick small axes), 
    with right button to translate and the wheel to zoom.

Keyboard: 
    [p]     Pause
    [r]     Reset View
    [d]     Open gripper
    [g]     Close gripper
    [s]     Stabilize robot arm
    [e]     Export points to ply (./out.ply)
    [q\ESC] Quit
"""
from hgtm import homogeneous_transform


import math
import time
import cv2
import numpy as np
import pyrealsense2 as rs

import time
import argparse
import json
# Import Flexiv RDK Python library
# fmt: off
import sys
sys.path.insert(0, "../lib_py")
import flexivrdk
# fmt: on

def append_line_to_txt(filename, line):
    # 打开文件，如果文件不存在则创建
    with open(filename, 'a+') as file:
        # 将光标移动到文件末尾
        file.seek(0, 2)
        # 如果文件不为空，则在行末添加换行符
        if file.tell() > 0:
            file.write('\n')
        # 添加新行
        file.write(line)

class AppState:

    def __init__(self, *args, **kwargs):
        self.WIN_NAME = 'RealSense'
        #self.pitch, self.yaw = math.radians(-10), math.radians(-15)
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation = np.array([0, 0, -1], dtype=np.float32)
        self.distance = 2
        self.prev_mouse = 0, 0
        self.mouse_btns = [False, False, False]
        self.paused = False
        self.decimate = 1
        self.scale = False
        self.color = True

    def reset(self):
        self.pitch, self.yaw, self.distance = 0, 0, 2
        self.translation[:] = 0, 0, -1

    @property
    def rotation(self):
        Rx, _ = cv2.Rodrigues((self.pitch, 0, 0))
        Ry, _ = cv2.Rodrigues((0, self.yaw, 0))
        return np.dot(Ry, Rx).astype(np.float32)

    @property
    def pivot(self):
        return self.translation + np.array((0, 0, self.distance), dtype=np.float32)
    

class AppLoop:
    def __init__(self, robot_ip, local_ip):
        # argparser = argparse.ArgumentParser()
        # argparser.add_argument("robot_ip", help="IP address of the robot server")
        # argparser.add_argument("local_ip", help="IP address of this PC")
        # args = argparser.parse_args()
        # robot = flexivrdk.Robot(args.robot_ip, args.local_ip)
        robot = flexivrdk.Robot(robot_ip, local_ip)
        gripper = flexivrdk.Gripper(robot)
        gripper_states = flexivrdk.GripperStates()

        time.sleep(2)
        log = flexivrdk.Log()
        mode = flexivrdk.Mode

        base_T_camera = np.array([[ 0.99906299,  0.02242375,  0.03701784,  0.82142274],
        [-0.03600151, -0.04414769,  0.99837612, -0.33092116],
        [ 0.02402159, -0.99877333, -0.04329903,  0.26998527],
        [ 0.,          0.,          0.,          1.        ]])                         

        # Clear fault on robot server if any
        if robot.isFault():
            log.warn("Fault occurred on robot server, trying to clear ...")
            # Try to clear the fault
            robot.clearFault()
            time.sleep(2)
            # Check again
            if robot.isFault():
                log.error("Fault cannot be cleared, exiting ...")
                exit()
            log.info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        log.info("Enabling robot ...")
        robot.enable()

        # Wait for the robot to become operational
        while not robot.isOperational():
            time.sleep(1)

        log.info("Robot is now operational")

        robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)
        robot.executePrimitive("ZeroFTSensor()")
        time.sleep(3)

        robot.setMode(mode.NRT_PLAN_EXECUTION)
        log.info("Executing plan")
        robot.executePlan(22, True)
        robot_states = flexivrdk.RobotStates()
        count = 1
        time.sleep(3)

        log.info("Opening gripper")
        gripper.move(0.14, 0.1, 20) # Gipper parameters: position, force, speed
        time.sleep(3)

        self.state = AppState()

    def run(self):
        state = self.state
        # Configure depth and color streams
        pipeline = rs.pipeline()
        config = rs.config()

        # Load the camera configuration from the JSON file      
        with open('./realsense/med_acc.json', 'r') as file:
            json_text = file.read().strip()
            config_dict = json.loads(json_text)


        pipeline_wrapper = rs.pipeline_wrapper(pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)

        # Apply the configuration to the pipeline
        device = pipeline_profile.get_device()
        advanced_mode = rs.rs400_advanced_mode(device)
        advanced_mode.load_json(json_text)


        # Start streaming
        pipeline.start(config)

        # Get stream profile and camera intrinsics
        profile = pipeline.get_active_profile()
        depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
        depth_intrinsics = depth_profile.get_intrinsics()
        w, h = depth_intrinsics.width, depth_intrinsics.height

        # Processing blocks
        pc = rs.pointcloud()
        decimate = rs.decimation_filter()
        decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
        colorizer = rs.colorizer()

        # Using spatial filter
        spatial = rs.spatial_filter()
        spatial.set_option(rs.option.filter_magnitude, 5)
        spatial.set_option(rs.option.filter_smooth_alpha, 1)
        spatial.set_option(rs.option.filter_smooth_delta, 50)
        spatial.set_option(rs.option.holes_fill, 3)

        # Using temporal filter
        temporal = rs.temporal_filter()

        def mouse_cb(event, x, y, flags, param):

            if event == cv2.EVENT_LBUTTONDOWN:
                state.mouse_btns[0] = True

            if event == cv2.EVENT_LBUTTONUP:
                state.mouse_btns[0] = False

            if event == cv2.EVENT_RBUTTONDOWN:
                state.mouse_btns[1] = True

            if event == cv2.EVENT_RBUTTONUP:
                state.mouse_btns[1] = False

            if event == cv2.EVENT_MBUTTONDOWN:
                state.mouse_btns[2] = True

            if event == cv2.EVENT_MBUTTONUP:
                state.mouse_btns[2] = False

            if event == cv2.EVENT_MOUSEMOVE:

                h, w = out.shape[:2]
                dx, dy = x - state.prev_mouse[0], y - state.prev_mouse[1]

                if state.mouse_btns[0]:
                    state.yaw += float(dx) / w * 2
                    state.pitch -= float(dy) / h * 2

                elif state.mouse_btns[1]:
                    dp = np.array((dx / w, dy / h, 0), dtype=np.float32)
                    state.translation -= np.dot(state.rotation, dp)

                elif state.mouse_btns[2]:
                    dz = math.sqrt(dx**2 + dy**2) * math.copysign(0.01, -dy)
                    state.translation[2] += dz
                    state.distance -= dz

            if event == cv2.EVENT_MOUSEWHEEL:
                dz = math.copysign(0.1, flags)
                state.translation[2] += dz
                state.distance -= dz

            state.prev_mouse = (x, y)


        cv2.namedWindow(state.WIN_NAME, cv2.WINDOW_AUTOSIZE)
        cv2.resizeWindow(state.WIN_NAME, w, h)
        cv2.setMouseCallback(state.WIN_NAME, mouse_cb)


        def project(v):
            """project 3d vector array to 2d"""
            h, w = out.shape[:2]
            view_aspect = float(h)/w

            # ignore divide by zero for invalid depth
            with np.errstate(divide='ignore', invalid='ignore'):
                proj = v[:, :-1] / v[:, -1, np.newaxis] * \
                    (w*view_aspect, h) + (w/2.0, h/2.0)

            # near clipping
            znear = 0.03
            proj[v[:, 2] < znear] = np.nan
            return proj


        def view(v):
            """apply view transformation on vector array"""
            return np.dot(v - state.pivot, state.rotation) + state.pivot - state.translation


        def line3d(out, pt1, pt2, color=(0x80, 0x80, 0x80), thickness=1):
            """draw a 3d line from pt1 to pt2"""
            p0 = project(pt1.reshape(-1, 3))[0]
            p1 = project(pt2.reshape(-1, 3))[0]
            if np.isnan(p0).any() or np.isnan(p1).any():
                return
            p0 = tuple(p0.astype(int))
            p1 = tuple(p1.astype(int))
            rect = (0, 0, out.shape[1], out.shape[0])
            inside, p0, p1 = cv2.clipLine(rect, p0, p1)
            if inside:
                cv2.line(out, p0, p1, color, thickness, cv2.LINE_AA)


        def grid(out, pos, rotation=np.eye(3), size=1, n=10, color=(0x80, 0x80, 0x80)):
            """draw a grid on xz plane"""
            pos = np.array(pos)
            s = size / float(n)
            s2 = 0.5 * size
            for i in range(0, n+1):
                x = -s2 + i*s
                line3d(out, view(pos + np.dot((x, 0, -s2), rotation)),
                    view(pos + np.dot((x, 0, s2), rotation)), color)
            for i in range(0, n+1):
                z = -s2 + i*s
                line3d(out, view(pos + np.dot((-s2, 0, z), rotation)),
                    view(pos + np.dot((s2, 0, z), rotation)), color)


        def axes(out, pos, rotation=np.eye(3), size=0.075, thickness=2):
            """draw 3d axes"""
            line3d(out, pos, pos +
                np.dot((0, 0, size), rotation), (0xff, 0, 0), thickness)
            line3d(out, pos, pos +
                np.dot((0, size, 0), rotation), (0, 0xff, 0), thickness)
            line3d(out, pos, pos +
                np.dot((size, 0, 0), rotation), (0, 0, 0xff), thickness)


        def gaussian_blur(image, kernel_size=(7, 7), sigma=1):
            """apply gaussian blur to an image"""
            blurred_image = cv2.GaussianBlur(image, kernel_size, sigma)
            return blurred_image

        def frustum(out, intrinsics, color=(0x40, 0x40, 0x40)):
            """draw camera's frustum"""
            orig = view([0, 0, 0])
            w, h = intrinsics.width, intrinsics.height

            for d in range(1, 6, 2):
                def get_point(x, y):
                    p = rs.rs2_deproject_pixel_to_point(intrinsics, [x, y], d)
                    line3d(out, orig, view(p), color)
                    return p

                top_left = get_point(0, 0)
                top_right = get_point(w, 0)
                bottom_right = get_point(w, h)
                bottom_left = get_point(0, h)

                line3d(out, view(top_left), view(top_right), color)
                line3d(out, view(top_right), view(bottom_right), color)
                line3d(out, view(bottom_right), view(bottom_left), color)
                line3d(out, view(bottom_left), view(top_left), color)


        def pointcloud(out, verts, texcoords, color, painter=True):
            """draw point cloud with optional painter's algorithm"""
            if painter:
                # Painter's algo, sort points from back to front

                # get reverse sorted indices by z (in view-space)
                # https://gist.github.com/stevenvo/e3dad127598842459b68
                v = view(verts)
                s = v[:, 2].argsort()[::-1]
                proj = project(v[s])
            else:
                proj = project(view(verts))

            if state.scale:
                proj *= 0.5**state.decimate

            h, w = out.shape[:2]

            # proj now contains 2d image coordinates
            j, i = proj.astype(np.uint32).T

            # create a mask to ignore out-of-bound indices
            im = (i >= 0) & (i < h)
            jm = (j >= 0) & (j < w)
            m = im & jm

            cw, ch = color.shape[:2][::-1]
            if painter:
                # sort texcoord with same indices as above
                # texcoords are [0..1] and relative to top-left pixel corner,
                # multiply by size and add 0.5 to center
                v, u = (texcoords[s] * (cw, ch) + 0.5).astype(np.uint32).T
            else:
                v, u = (texcoords * (cw, ch) + 0.5).astype(np.uint32).T
            # clip texcoords to image
            np.clip(u, 0, ch-1, out=u)
            np.clip(v, 0, cw-1, out=v)

            # perform uv-mapping
            out[i[m], j[m]] = color[u[m], v[m]]


        out = np.empty((h, w, 3), dtype=np.uint8)

        use_gaussian_blur = True
        depth_to_disparity = rs.disparity_transform(True)
        disparity_to_depth = rs.disparity_transform(False)

        while True:
            # Grab camera data
            if not state.paused:
                # Wait for a coherent pair of frames: depth and color

                framesets = []

                for i in range(10):

                    frames = pipeline.wait_for_frames()
                    depth_frame = frames.get_depth_frame()
                    framesets.append(depth_frame)
                
                color_frame = frames.get_color_frame()

                for i in range(10):

                    frame = framesets[i]
                    frame = decimate.process(frame)
                    frame = depth_to_disparity.process(frame)
                    frame = spatial.process(frame)
                    frame = temporal.process(frame)
                    frame = disparity_to_depth.process(frame)
                    #frame = hole_filling.process(frame)
                depth_frame = frame

                #depth_frame = decimate.process(depth_frame)

                # Grab new intrinsics (may be changed by decimation)
                depth_intrinsics = rs.video_stream_profile(
                    depth_frame.profile).get_intrinsics()
                w, h = depth_intrinsics.width, depth_intrinsics.height

                depth_image = np.asanyarray(depth_frame.get_data())
                color_image = np.asanyarray(color_frame.get_data())

                depth_image = depth_image if not use_gaussian_blur else gaussian_blur(depth_image)

                depth_colormap = np.asanyarray(
                    colorizer.colorize(depth_frame).get_data())

                if state.color:
                    mapped_frame, color_source = color_frame, color_image
                else:
                    mapped_frame, color_source = depth_frame, depth_colormap

                points = pc.calculate(depth_frame)
                pc.map_to(mapped_frame)

                # Pointcloud data to arrays
                v, t = points.get_vertices(), points.get_texture_coordinates()
                verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
                texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv

            # Render
            now = time.time()

            out.fill(0)

            grid(out, (0, 0.5, 1), size=1, n=10)
            frustum(out, depth_intrinsics)
            axes(out, view([0, 0, 0]), state.rotation, size=0.1, thickness=1)

            if not state.scale or out.shape[:2] == (h, w):
                pointcloud(out, verts, texcoords, color_source)
            else:
                tmp = np.zeros((h, w, 3), dtype=np.uint8)
                pointcloud(tmp, verts, texcoords, color_source)
                tmp = cv2.resize(
                    tmp, out.shape[:2][::-1], interpolation=cv2.INTER_NEAREST)
                np.putmask(out, tmp > 0, tmp)

            if any(state.mouse_btns):
                axes(out, view(state.pivot), state.rotation, thickness=4)

            dt = time.time() - now

            cv2.setWindowTitle(
                state.WIN_NAME, "RealSense (%dx%d) %dFPS (%.2fms) %s" %
                (w, h, 1.0/dt, dt*1000, "PAUSED" if state.paused else ""))

            cv2.imshow(state.WIN_NAME, out)
            key = cv2.waitKey(1)

            if key == ord("r"):
                state.reset()

            if key == ord("p"):
                state.paused ^= True

            if key == ord("d"):
                state.decimate = (state.decimate + 1) % 3
                decimate.set_option(rs.option.filter_magnitude, 2 ** state.decimate)
                #print(state.decimate)

            if key == ord("z"):
                state.scale ^= True

            if key == ord("g"):
                AppLoop.log.info("Closing gripper")
                AppLoop.gripper.move(0.11, 0.8, 60) # Gipper parameters: position, force, speed
                time.sleep(3)


            if key == ord("d"):
                AppLoop.log.info("Opening gripper")
                AppLoop.gripper.move(0.14, 0.1, 20) # Gipper parameters: position, force, speed
                time.sleep(3)

            if key == ord("n"):
                use_gaussian_blur = not use_gaussian_blur

            if key == ord("c"):
                state.color ^= True

            if key == ord("s"):
                # cv2.imwrite('./out.png', out)
                AppLoop.robot.setMode(mode.NRT_PRIMITIVE_EXECUTION)
                AppLoop.robot.executePrimitive("ZeroFTSensor()")
                AppLoop.log.info("ZeroFTSensor")
                time.sleep(3)

                AppLoop.robot.setMode(mode.NRT_PLAN_EXECUTION)
                AppLoop.log.info("Executing plan")
                AppLoop.robot.executePlan(22, True)
                time.sleep(3)      

            if key in (27, ord("q")) or cv2.getWindowProperty(state.WIN_NAME, cv2.WND_PROP_AUTOSIZE) < 0:
                break

        # Stop streaming
        pipeline.stop()