import pyrealsense2 as rs
import numpy as np
import cv2

# Initialize RealSense pipeline and config
pipe = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipe.start(cfg)

# Global vars for mouse callback
depth_frame_global = None
cursor_x, cursor_y = 0, 0

# Mouse callback to update cursor position
def mouse_move(event, x, y, flags, param):
    global cursor_x, cursor_y
    if event == cv2.EVENT_MOUSEMOVE:
        cursor_x, cursor_y = x, y

# Set up window and attach mouse callback
cv2.namedWindow('depth')
cv2.setMouseCallback('depth', mouse_move)

try:
    while True:
        frame = pipe.wait_for_frames()
        depth_frame = frame.get_depth_frame()
        color_frame = frame.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Save latest depth frame for distance lookup
        depth_frame_global = depth_frame

        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        depth_cm = cv2.applyColorMap(
            cv2.convertScaleAbs(depth_image, alpha=0.5), cv2.COLORMAP_JET
        )

        # Get depth in meters at the cursor position
        distance = depth_frame.get_distance(cursor_x, cursor_y) * 100
        label = f"Depth: ({cursor_x}, {cursor_y}): {distance:.3f}cm"

        # Draw label on the depth colormap image
        cv2.circle(depth_cm, (cursor_x, cursor_y), 5, (255, 255, 255), -1)
        cv2.putText(depth_cm, label, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        gray_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        #cv2.imshow('rgb', color_image)
        cv2.imshow('depth', depth_cm)
        cv2.imshow('gray', gray_image)

        if cv2.waitKey(1) == ord('x'):
            break

finally:
    pipe.stop()
    cv2.destroyAllWindows()