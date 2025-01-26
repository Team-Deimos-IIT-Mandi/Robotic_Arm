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

class VisualServoing:
    def __init__(self):
        rospy.init_node("visual_servoing", anonymous=True)
        self.listener = tf.TransformListener()
        self.bridge = CvBridge()
        self.pub = rospy.Publisher("/body_controller/command", JointTrajectory, queue_size=10)

        self.q = np.zeros(8)
        self.errors = []

        self.old_gray = None
        self.mask = None
        self.points = []
        self.p0 = None
        self.colors = np.random.randint(0, 255, (100, 3))

        self.lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
        rospy.Subscriber("/camera_gripper/image_raw", Image, self.process_image)
        self.setup_transforms()

    def setup_transforms(self):
        try:
            self.listener.waitForTransform("camera_link2", "Link_6", rospy.Time(0), rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("camera_link2", "Link_6", rospy.Time(0))
            self.Re_c = tf.transformations.quaternion_matrix(rot)[:3, :3]
            self.de_c = trans
            self.S_de_c = np.matrix(
                [[0, -self.de_c[2], self.de_c[1]], [self.de_c[2], 0, -self.de_c[0]], [-self.de_c[1], self.de_c[0], 0]]
            )
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not get transform between camera_link2 and Link_6")

        try:
            self.listener.waitForTransform("Link_6", "base_link", rospy.Time(0), rospy.Duration(1.0))
            (transb, rotb) = self.listener.lookupTransform("Link_6", "base_link", rospy.Time(0))
            self.Rb_e = tf.transformations.quaternion_matrix(rotb)[:3, :3]
            self.transb = transb
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Could not get transform between Link_6 and base_link")

    def publish_joint_angles(self, joint_angles):
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
        self.pub.publish(trajectory_msg)

    def compute_jacobian(self, ee_b):
        pin.forwardKinematics(model, data, self.q)
        pin.updateFramePlacements(model, data)
        pin.computeJointJacobians(model, data, self.q)

        J = pin.computeFrameJacobian(
            model, data, self.q, end_effector_frame, pin.ReferenceFrame.WORLD
        )
        desired_twist = ee_b.A1

        H = J.T @ J + damping * np.eye(model.nv)
        g = J.T @ desired_twist

        theta_dot_max = 1.0 * np.ones(model.nv)
        theta_dot_min = -1.0 * np.ones(model.nv)

        q_upper_violation = (q_max - self.q) / dt
        q_lower_violation = (q_min - self.q) / dt

        C = np.vstack(
            [np.eye(model.nv), -np.eye(model.nv), np.eye(model.nv), -np.eye(model.nv)]
        )
        b = np.hstack(
            [theta_dot_min, -theta_dot_max, q_lower_violation, -q_upper_violation]
        )

        theta_dot = solve_qp(H, g, C.T, b)[0]
        self.q = pin.integrate(model, self.q, theta_dot * 0.05)
        self.publish_joint_angles(self.q)

    def compute_ee_velocity_base(self, ee_b, rotation_matrix):
        zero_matrix = np.zeros((3, 3))
        ee_b = np.block([[rotation_matrix, zero_matrix], [zero_matrix, rotation_matrix]]) @ ee_b
        self.compute_jacobian(ee_b)

    def compute_ee_velocity(self, camera_velocity, rotation_matrix, skew_matrix):
        zero_matrix = np.zeros((3, 3))
        m = np.block([[rotation_matrix, skew_matrix @ rotation_matrix], [zero_matrix, rotation_matrix]])
        ee_b = m @ camera_velocity
        self.compute_ee_velocity_base(ee_b, self.Rb_e)

    def compute_interaction_matrix(self, points, focal_length, depth):
        n = len(points)
        Le = []
        for i in range(n):
            u = points[i][0]
            v = points[i][1]
            Lp = np.matrix(
                [
                    [-focal_length / depth, 0, u / depth, u * v / focal_length, -(focal_length**2 + u**2) / focal_length, v],
                    [0, -focal_length / depth, v / depth, (focal_length**2 + v**2) / focal_length, -u * v / focal_length, -u],
                ]
            )
            Le.append(Lp)
        return np.vstack(Le)

    def calculate_area(self, points):
        if len(points) < 3:
            return 0
        x1, y1 = points[0]
        x2, y2 = points[1]
        x3, y3 = points[2]
        return 0.5 * abs(x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2))

    def calculate_orientation(self, points):
        if len(points) < 3:
            return None
        angle = np.arctan2((points[2][1] - points[1][1]), (points[2][0] - points[1][0]))
        return angle

    def process_image(self, image_msg):
        frame = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.old_gray is None:
            self.old_gray = frame_gray.copy()
            self.mask = np.zeros_like(frame)
            return

        if self.points:
            new_p0 = np.array(self.points, dtype=np.float32).reshape(-1, 1, 2)
            self.p0 = new_p0 if self.p0 is None else np.vstack((self.p0, new_p0))
            self.points.clear()

        if self.p0 is not None and len(self.p0) > 0:
            p1, st, err = cv2.calcOpticalFlowPyrLK(
                self.old_gray, frame_gray, self.p0, None, **self.lk_params
            )

            good_new = p1[st == 1]
            good_old = self.p0[st == 1]

            for i, (new, old) in enumerate(zip(good_new, good_old)):
                x_new, y_new = new.ravel()
                x_old, y_old = old.ravel()
                color = self.colors[i % len(self.colors)].tolist()
                cv2.line(self.mask, (int(x_new), int(y_new)), (int(x_old), int(y_old)), color, 2)
                cv2.circle(frame, (int(x_new), int(y_new)), 5, color, -1)

            shape_area = self.calculate_area(good_new)
            orientation = self.calculate_orientation(good_new)

            if shape_area > 0:
                self.final_velocity(theta=orientation, sigma=shape_area**0.5, points=good_new)

            text = (
                f"Area: {shape_area:.2f}, Angle: {orientation:.2f}"
                if orientation is not None
                else f"Area: {shape_area:.2f}"
            )
            cv2.putText(frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            output = cv2.add(frame, self.mask)
            cv2.imshow("Frame", output)
            self.old_gray = frame_gray.copy()
            self.p0 = good_new.reshape(-1, 1, 2)
        else:
            cv2.imshow("Frame", frame)

        if cv2.waitKey(30) & 0xFF == ord("q"):
            rospy.signal_shutdown("User exited")

    def final_velocity(self, theta, sigma, points):
        Kwz = 0.01
        Kvz = 0.01
        Kp = 0.01
        area = 500

        sd = np.array([[425, 478], [315, 750], [530, 750]])
        error = points - sd
        error = error.reshape(6, 1)
        wz = Kwz * (theta - 0)
        vz = Kvz * (sigma - area)
        zi_z = np.array([[vz], [wz]])

        width = 800
        fov = 1.3962634
        focal_length = width / (2 * np.tan(fov / 2))
        depth = self.transb[2] - 0.12

        Le = self.compute_interaction_matrix(points, focal_length, depth)

        Lz = np.hstack([Le[:, 2], Le[:, 5]])
        Lxy = np.hstack([Le[:, 0:2], Le[:, 3:5]])
        L_in = np.linalg.pinv(Lxy)
        z_mat = np.dot(Lz, zi_z)
        post = -(Kp * error) - z_mat
        zi_xy = L_in @ post
        zi_cam = np.vstack([zi_xy[0], zi_xy[1], zi_z[0], zi_xy[2], zi_xy[3], zi_z[1]])
        self.compute_ee_velocity(zi_cam, self.Re_c, self.S_de_c)
        self.errors.append(np.linalg.norm(sd - points))

    def select_points(self):
        for _ in range(3):
            x = int(input("Enter x coordinate: "))
            y = int(input("Enter y coordinate: "))
            self.points.append((x, y))
            print(f"Point selected: {x}, {y}")

    def run(self):
        self.setup_transforms()
        self.select_points()
        rospy.spin()
        cv2.destroyAllWindows()
        plt.plot(self.errors)
        plt.title("Error over Time")
        plt.xlabel("Time Step")
        plt.ylabel("Error (Euclidean Distance)")
        plt.show()

if __name__ == "__main__":
    visual_servoing = VisualServoing()
    visual_servoing.run()