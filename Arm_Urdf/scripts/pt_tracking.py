#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import tf
import tf.transformations
import pinocchio as pin
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from IK_gazebo import (
    model,
    data,
    end_effector_frame,
    q_max,
    q_min,
    damping,
    dt,
    solve_qp,
)
import matplotlib.pyplot as plt
from std_msgs.msg import Float32

# Initialize ROS node
rospy.init_node("visual_servoing", anonymous=True)

# Visual Servoing functions
listener = tf.TransformListener()
global q
q = np.zeros(8)
errors = []
try:
    listener.waitForTransform(
        "camera_link2", "Link_6", rospy.Time(0), rospy.Duration(1.0)
    )
    (trans, rot) = listener.lookupTransform("camera_link2", "Link_6", rospy.Time(0))
    listener.waitForTransform("camera_link1", "Link_6", rospy.Time(0), rospy.Duration(1.0))
    (trans, rot) = listener.lookupTransform("camera_link1", "Link_6", rospy.Time(0))
    # Convert quaternion to rotation matrix
    Re_c = tf.transformations.quaternion_matrix(rot)[:3, :3]
    de_c = trans
    S_de_c = np.matrix(
        [[0, -de_c[2], de_c[1]], [de_c[2], 0, -de_c[0]], [-de_c[1], de_c[0], 0]]
    )
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print("Could not get transform")

try:
    listener.waitForTransform("Link_6", "base_link", rospy.Time(0), rospy.Duration(1.0))
    (transb, rotb) = listener.lookupTransform("Link_6", "base_link", rospy.Time(0))
    # Convert quaternion to rotation matrix
    Rb_e = tf.transformations.quaternion_matrix(rot)[:3, :3]
except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
    print("Could not get transform")

def callback(msg: JointTrajectoryControllerState):
    global q
    n=msg.actual.positions
    for i in range(0,len(n)):
        q[i]=n[i]       

rospy.Subscriber("/body_controller/state",JointTrajectoryControllerState,callback)   
#  
pub=rospy.Publisher('/body_controller/command',JointTrajectory,queue_size=10)
def publish_joint_angles(joint_angles):
    trajectory_msg = JointTrajectory()
    trajectory_msg.joint_names = [
        "Joint_1",
        "Joint_2",
        "Joint_3",
        "Joint_4",
        "Joint_5",
        "Joint_6",
    ]
    point = JointTrajectoryPoint()
    point.positions = joint_angles
    point.time_from_start = rospy.Duration(0.1)
    trajectory_msg.points = [point]
    pub.publish(trajectory_msg)


def jaco(ee_b):
    global q

    # Compute forward kinematics and Jacobian
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    pin.computeJointJacobians(model, data, q)

    J = pin.computeFrameJacobian(
        model, data, q, end_effector_frame, pin.ReferenceFrame.WORLD
    )
    # print(J)
    # Get the desired twist from key input
    desired_twist = ee_b.A1

    # Quadratic program matrices
    H = J.T @ J + damping * np.eye(model.nv)  # Regularized Hessian
    g = J.T @ desired_twist  # Gradient term
    # Inequality constraints for joint velocity limits
    theta_dot_max = 1.0 * np.ones(model.nv)
    theta_dot_min = -1.0 * np.ones(model.nv)
    # Constraints for joint position limits
    q_upper_violation = (q_max - q) / dt
    q_lower_violation = (q_min - q) / dt
    # Combine velocity and position constraints
    C = np.vstack(
        [np.eye(model.nv), -np.eye(model.nv), np.eye(model.nv), -np.eye(model.nv)]
    )
    b = np.hstack(
        [theta_dot_min, -theta_dot_max, q_lower_violation, -q_upper_violation]
    )
    # Solve the quadratic program
    theta_dot = solve_qp(H, g, C.T, b)[0]
    # Update joint configuration using integration
    q = pin.integrate(model, q, theta_dot * 0.05)
    # viz.display(q)
    # Publish the calculated joint angles
    print(q)
    publish_joint_angles(q)


def ee_velo_base(e, r):
    zero_matrix = np.zeros((3, 3))
    ee_b = np.block([[r, zero_matrix], [zero_matrix, r]]) @ e
    print(ee_b)
    jaco(ee_b)


def ee_velo(c, R, S_d):
    zero_matrix = np.zeros((3, 3))
    m = np.block([[R, S_d @ R], [zero_matrix, R]])
    print(m @ c)
    ee_velo_base(m @ c, Rb_e)


def int_matrix(s, lam, z):
    n = len(s)
    Le = []
    for i in range(0, n):
        u = s[i][0]
        v = s[i][1]
        Lp = np.matrix(
            [
                [-lam / z, 0, u / z, u * v / lam, -(lam**2 + u**2) / lam, v],
                [0, -lam / z, v / z, (lam**2 + v**2) / lam, -u * v / lam, -u],
            ]
        )
        Le.append(Lp)

    return np.vstack(Le)
def plot_err(desired_pos,actual_pos):
    return np.linalg.norm(np.array(actual_pos) - np.array(desired_pos))

def depth_callback(msg: Float32):
    global z
    z=msg.data

def final_velo(theta,sigma,s):
    global z
    Kwz=0.11
    Kvz=0.11
    Kp=0.1
    area=500
    
    sd=np.array([[395,445],[582,447],[391,664],[585,668]])
    print(s)
    e = s - sd
    e = e.reshape(6, 1)
    wz = Kwz * (theta - 0)
    print(theta)
    vz = Kvz * (sigma - area)
    zi_z = np.array([[vz], [wz]])  # 2x1

    width = 800
    fov = 1.3962634
    focal_length = width / (2 * np.tan(fov / 2))
    print(z)
    Le = int_matrix(s, focal_length, z)

    Lz = np.hstack([Le[:, 2], Le[:, 5]])  # 6x2
    Lxy = np.hstack([Le[:, 0:2], Le[:, 3:5]])  # 6x4
    L_in = np.linalg.pinv(Lxy)
    z_mat = np.dot(Lz, zi_z)
    post = -(Kp * e) - z_mat
    zi_xy = L_in @ post
    zi_cam = np.vstack([zi_xy[0], zi_xy[1], zi_z[0], zi_xy[2], zi_xy[3], zi_z[1]])
    print(zi_cam)
    ee_velo(zi_cam, Re_c, S_de_c)
    errors.append(plot_err(sd, s))


# Parameters for Lucas-Kanade optical flow
lk_params = dict(
    winSize=(15, 15),
    maxLevel=2,
    criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
)

# Create random colors for tracking points
colors = np.random.randint(0, 255, (100, 3))

# Initialize CvBridge
bridge = CvBridge()

# List to store manually selected points
points = []
p0 = None
old_gray = None
mask = None


def select_point():
    global points, p0
    for i in range(0, 3):
        x = int(input("p1 x "))
        y = int(input("p1 y "))
        points.append((x, y))
        print(f"Point selected: {x}, {y}")


def calculate_area(points):
    if len(points) < 3:
        return 0  # Area is 0 for fewer than 3 points

    x1, y1 = points[0]
    x2, y2 = points[1]
    x3, y3 = points[2]

    return 0.5 * abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))


def calculate_orientation(points):
    if len(points) < 3:
        return None  # No orientation for fewer than 2 points
    angle = np.arctan2((points[2][1] - points[1][1]), (points[2][0] - points[1][0]))
    return angle


def image_callback(msg):
    global old_gray, p0, mask

    frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
    frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    if old_gray is None:
        old_gray = frame_gray.copy()
        mask = np.zeros_like(frame)
        return

    if points:
        new_p0 = np.array(points, dtype=np.float32).reshape(-1, 1, 2)
        p0 = new_p0 if p0 is None else np.vstack((p0, new_p0))
        points.clear()

    if p0 is not None and len(p0) > 0:
        p1, st, err = cv2.calcOpticalFlowPyrLK(
            old_gray, frame_gray, p0, None, **lk_params
        )

        good_new = p1[st == 1]
        good_old = p0[st == 1]

        for i, (new, old) in enumerate(zip(good_new, good_old)):
            x_new, y_new = new.ravel()
            x_old, y_old = old.ravel()
            color = colors[i % len(colors)].tolist()
            cv2.line(mask, (int(x_new), int(y_new)), (int(x_old), int(y_old)), color, 2)
            cv2.circle(frame, (int(x_new), int(y_new)), 5, color, -1)

        # Calculate area and orientation
        shape_area = calculate_area(good_new)
        orientation = calculate_orientation(good_new)
        if shape_area > 0:
            final_velo(theta=orientation, sigma=shape_area**0.5, s=good_new)

        # Display area and orientation on frame
        text = (
            f"Area: {shape_area:.2f}, Angle: {orientation:.2f}"
            if orientation is not None
            else "Area: {:.2f}".format(shape_area)
        )
        cv2.putText(
            frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        output = cv2.add(frame, mask)
        cv2.imshow("Frame", output)
        old_gray = frame_gray.copy()
        p0 = good_new.reshape(-1, 1, 2)
    else:
        cv2.imshow("Frame", frame)

    if cv2.waitKey(30) & 0xFF == ord("q"):
        rospy.signal_shutdown("User exited")


# Subscribe to the camera topic
rospy.Subscriber("/camera_gripper_left/image_raw", Image, image_callback)
rospy.Subscriber("/image_depth",Float32,depth_callback)
select_point()

# Keep the program running
rospy.spin()
cv2.destroyAllWindows()

plt.plot(errors)
plt.title("Error over Time")
plt.xlabel("Time Step")
plt.ylabel("Error (Euclidean Distance)")
plt.show()
