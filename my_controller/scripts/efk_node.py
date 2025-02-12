#!/usr/bin/env python3
"""
ROS2 Node for Extended Kalman Filter Localization with 15-state model

State vector (15 states):
    x = [ p_x, p_y, p_z,       -- Position (3)
          roll, pitch, yaw,    -- Orientation in Euler angles (3)
          v_x, v_y, v_z,       -- Linear velocity (3)
          ω_x, ω_y, ω_z,       -- Angular velocity (3)
          a_x, a_y, a_z ]^T    -- Linear acceleration (3)

Prediction Model:
    pₖ₊₁ = pₖ + R(rₖ) * (vₖ Δt + ½ aₖ Δt²)
    rₖ₊₁ = rₖ + J(rₖ) * ωₖ Δt
    vₖ₊₁ = vₖ + aₖ Δt
    ωₖ₊₁ = ωₖ + uₖ^α Δt       (control input in angular acceleration)
    aₖ₊₁ = aₖ

Measurement Sources:
  - /odom: Odometry message providing:
         z_odom = [ p_x, p_y, p_z,  v_x, v_y, v_z ]^T   (6-dim)
  - /imu: Imu message providing:
         z_imu = [ roll, pitch, yaw,  ω_x, ω_y, ω_z,  a_x, a_y, a_z ]^T   (9-dim)

The EKF fuses these measurements to update the state estimate.
Estimated pose (position and orientation) is published to "/ekf_pose".

Author: [Your Name]
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math

from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from ament_index_python.packages import get_package_share_directory

# Timer period [s]
DT = 0.1

# Process noise covariance Q (15x15)
# Tuning parameters (example values)
Q = np.diag([
    0.05, 0.05, 0.05,            # position noise
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),  # orientation noise (rad)
    0.1, 0.1, 0.1,               # linear velocity noise
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),  # angular velocity noise (rad/s)
    0.2, 0.2, 0.2                # linear acceleration noise
]) ** 2

# Measurement noise covariance for odometry (6x6): [p (3), v (3)]
R_odom = np.diag([0.2, 0.2, 0.2, 0.1, 0.1, 0.1]) ** 2

# Measurement noise covariance for IMU (9x9): [orientation (3), angular velocity (3), linear acceleration (3)]
R_imu = np.diag([
    np.deg2rad(1.0), np.deg2rad(1.0), np.deg2rad(1.0),
    np.deg2rad(0.5), np.deg2rad(0.5), np.deg2rad(0.5),
    0.2, 0.2, 0.2
]) ** 2

class EKFFullNode(Node):
    def __init__(self):
        super().__init__('ekf_full_node')
        self.dt = DT
        self.last_time = self.get_clock().now()
        
        # Initialize 15-dim state vector: [p (3); r (3); v (3); ω (3); a (3)]
        self.xEst = np.zeros((15, 1))
        self.PEst = np.eye(15)
        
        # Latest measurements
        self.z_odom = None  # from /odom: 6-dim measurement
        self.new_odom = False
        
        self.z_imu = None   # from /imu: 9-dim measurement
        self.new_imu = False
        
        # Control input for angular acceleration (u^α); assume 0 if not provided.
        self.u_alpha = np.zeros((3,1))  # 3-dim (for ω update), can be extended if available
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )
        
        # Publisher: publish estimated pose (position and orientation)
        self.ekf_pub = self.create_publisher(PoseStamped, '/ekf_pose', 10)
        
        # Timer for periodic EKF update
        self.timer = self.create_timer(self.dt, self.timer_callback)
    
    # --- Helper functions for rotation and Euler rate transformation ---
    def R_from_euler(self, roll, pitch, yaw):
        """
        Compute rotation matrix R from Euler angles (roll, pitch, yaw).
        R = Rz(yaw)*Ry(pitch)*Rx(roll)
        """
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr          ]
        ])
        return R

    def J_from_euler(self, roll, pitch, yaw):
        """
        Compute the matrix J that relates angular velocity in body frame to Euler angle rates.
        Standard formula for roll-pitch-yaw:
        
           [ φ̇ ]   [ 1, sin(roll)*tan(pitch), cos(roll)*tan(pitch) ] [ ω_x ]
           [ θ̇ ] = [ 0, cos(roll),           -sin(roll)            ] [ ω_y ]
           [ ψ̇ ]   [ 0, sin(roll)/cos(pitch), cos(roll)/cos(pitch) ] [ ω_z ]
        """
        roll = float(roll)
        pitch = float(pitch)
        # Handle potential division by zero for pitch = ±pi/2
        cos_pitch = math.cos(pitch)
        if abs(cos_pitch) < 1e-4:
            cos_pitch = 1e-4
        tan_pitch = math.tan(pitch)
        J = np.array([
            [1, math.sin(roll)*tan_pitch, math.cos(roll)*tan_pitch],
            [0, math.cos(roll),          -math.sin(roll)],
            [0, math.sin(roll)/cos_pitch,  math.cos(roll)/cos_pitch]
        ])
        return J
    
    # --- End Helper functions ---
    
    def odom_callback(self, msg):
        """
        Callback for odometry messages.
        Extract measurement:
         - Position: [p_x, p_y, p_z]
         - Linear velocity: [v_x, v_y, v_z]
        """
        px = msg.pose.pose.position.x
        py = msg.pose.pose.position.y
        pz = msg.pose.pose.position.z
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        vz = msg.twist.twist.linear.z
        
        self.z_odom = np.array([[px], [py], [pz], [vx], [vy], [vz]])
        self.new_odom = True
    
    def imu_callback(self, msg):
        """
        Callback for IMU messages.
        Extract measurement:
         - Orientation: convert quaternion to Euler angles: [roll, pitch, yaw]
         - Angular velocity: [ω_x, ω_y, ω_z]
         - Linear acceleration: [a_x, a_y, a_z]
        """
        q = msg.orientation
        quat = [q.x, q.y, q.z, q.w]
        (roll, pitch, yaw) = euler_from_quaternion(quat)
        
        omega_x = msg.angular_velocity.x
        omega_y = msg.angular_velocity.y
        omega_z = msg.angular_velocity.z
        
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        self.z_imu = np.array([
            [roll], [pitch], [yaw],
            [omega_x], [omega_y], [omega_z],
            [ax], [ay], [az]
        ])
        self.new_imu = True
    
    def dynamic_model(self, x, dt):
        """
        Compute the predicted state xₖ₊₁ from state xₖ using the following model:
        
        pₖ₊₁ = pₖ + R(rₖ) * (vₖ Δt + ½ aₖ Δt²)
        rₖ₊₁ = rₖ + J(rₖ) * ωₖ Δt
        vₖ₊₁ = vₖ + aₖ Δt
        ωₖ₊₁ = ωₖ + u_α Δt   (u_α: control in angular acceleration; assumed zero if not provided)
        aₖ₊₁ = aₖ
        
        Note: rₖ (orientation) is given as [roll, pitch, yaw].
        """
        x_new = np.zeros((15,1))
        # Indices:
        # 0:3   -> p (position)
        # 3:6   -> r (orientation: roll, pitch, yaw)
        # 6:9   -> v (linear velocity)
        # 9:12  -> ω (angular velocity)
        # 12:15 -> a (linear acceleration)
        
        # Update position:
        # First, compute rotation matrix R from orientation (roll, pitch, yaw)
        roll = x[3,0]
        pitch = x[4,0]
        yaw = x[5,0]
        R_mat = self.R_from_euler(roll, pitch, yaw)  # 3x3
        # Compute term: b = v*dt + 0.5*a*dt^2, where v and a are 3x1 vectors (indices 6:9 and 12:15)
        b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
        x_new[0:3] = x[0:3] + R_mat @ b
        
        # Update orientation:
        # Use J: from Euler angles to Euler angle rates.
        J_mat = self.J_from_euler(roll, pitch, yaw)  # 3x3
        # Orientation update: r_new = r + J(r)*ω*dt
        x_new[3:6] = x[3:6] + J_mat @ x[9:12] * dt
        
        # Update linear velocity:
        x_new[6:9] = x[6:9] + x[12:15] * dt
        
        # Update angular velocity:
        # ω_new = ω + u_α*dt. Here we assume u_α is provided (if not, it is zero).
        x_new[9:12] = x[9:12] + self.u_alpha * dt
        
        # Update linear acceleration (assumed constant)
        x_new[12:15] = x[12:15]
        
        return x_new
    
    def jacobian_F(self, x, dt):
        """
        Compute the full Jacobian (15x15) of the dynamic model with respect to the state x,
        without any simplifications.
        
        State vector x = [ p; r; v; ω; a ] with dimensions:
        p: indices 0:3, r: indices 3:6, v: indices 6:9, ω: indices 9:12, a: indices 12:15.
        
        The dynamic model is:
        pₖ₊₁ = pₖ + R(rₖ) * ( vₖ * dt + 0.5 * aₖ * dt² )
        rₖ₊₁ = rₖ + J(rₖ) * ωₖ * dt
        vₖ₊₁ = vₖ + aₖ * dt
        ωₖ₊₁ = ωₖ
        aₖ₊₁ = aₖ
        
        The Jacobian F = ∂f/∂x is partitioned as:
        
        F = [ F_pp,  F_pr,    F_pv,    F_pω,    F_pa;
                0,     F_rr,    0,       F_rω,    0;
                0,     0,       I₃,      0,       I₃*dt;
                0,     0,       0,       I₃,      0;
                0,     0,       0,       0,       I₃ ]
                
        Where:
        - F_pp = I₃
        - F_pr = L, with L[:,i] = (dR/d(r_i)) * b, for i = roll, pitch, yaw, and b = v*dt + 0.5*a*dt².
        - F_pv = R(r) * dt
        - F_pω = 0
        - F_pa = R(r) * (0.5*dt²)
        - F_rr = I₃ + M*dt, where M = (dJ/droll)*ω_x + (dJ/dpitch)*ω_y + (dJ/dyaw)*ω_z.
        - F_rω = J(r)*dt
        
        The remaining blocks are as defined in the model.
        """
        F = np.eye(15)
        I3 = np.eye(3)
        
        # ----- Block for position update (indices 0:3) -----
        # pₖ₊₁ = pₖ + R(rₖ)*(vₖ*dt + 0.5*aₖ*dt²)
        # F_pp = I₃
        # F_pr = L = [ dR/droll*b, dR/dpitch*b, dR/dyaw*b ]
        roll = x[3,0]
        pitch = x[4,0]
        yaw = x[5,0]
        R_mat = self.R_from_euler(roll, pitch, yaw)  # 3x3
        # Compute derivatives of R
        dR_dr = np.zeros((3,3,3))
        dR_dr[:,:,0] = self.dR_droll(roll, pitch, yaw)
        dR_dr[:,:,1] = self.dR_dpitch(roll, pitch, yaw)
        dR_dr[:,:,2] = self.dR_dyaw(roll, pitch, yaw)
        
        # b = v*dt + 0.5*a*dt², where v and a are state indices 6:9 and 12:15
        b = x[6:9] * dt + 0.5 * x[12:15] * (dt**2)
        
        L = np.zeros((3,3))
        for i in range(3):
            L[:, i] = (dR_dr[:,:,i] @ b).flatten()
        F[0:3, 3:6] = L
        
        # F_pv = ∂p/∂v = R(r)*dt
        F[0:3, 6:9] = R_mat * dt
        
        # F_pω = 0 (no dependence)
        # F_pa = R(r)*0.5*dt²
        F[0:3, 12:15] = R_mat * (0.5 * dt**2)
        
        # ----- Block for orientation update (indices 3:6) -----
        # rₖ₊₁ = rₖ + J(rₖ)*ωₖ*dt
        J_mat = self.J_from_euler(roll, pitch, yaw)  # 3x3
        # Compute derivatives of J with respect to roll, pitch, yaw
        dJ_droll_mat = self.dJ_droll(roll, pitch, yaw)
        dJ_dpitch_mat = self.dJ_dpitch(roll, pitch, yaw)
        dJ_dyaw_mat = self.dJ_dyaw(roll, pitch, yaw)  # may be nonzero in general
        # Extract angular velocity ω from state indices 9:12
        omega = x[9:12]  # 3x1
        # Compute M = dJ/droll*ω_x + dJ/dpitch*ω_y + dJ/dyaw*ω_z
        M = dJ_droll_mat * omega[0,0] + dJ_dpitch_mat * omega[1,0] + dJ_dyaw_mat * omega[2,0]
        F[3:6, 3:6] = np.eye(3) + M * dt
        # F_rω = J(r)*dt
        F[3:6, 9:12] = J_mat * dt
        # Blocks F_rp, F_rv, F_ra remain zero.
        
        # ----- Block for linear velocity update (indices 6:9) -----
        # vₖ₊₁ = vₖ + aₖ*dt
        F[6:9, 6:9] = I3
        F[6:9, 12:15] = I3 * dt
        
        # ----- Block for angular velocity update (indices 9:12) -----
        # ωₖ₊₁ = ωₖ (assumed constant, or control input u_alpha added separately)
        F[9:12, 9:12] = I3
        
        # ----- Block for linear acceleration update (indices 12:15) -----
        # aₖ₊₁ = aₖ
        F[12:15, 12:15] = I3
        
        return F

    # ฟังก์ชันสำหรับ 𝑅 และอนุพันธ์ของมัน
    def R_from_euler(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr]
        ])
        return R

    def dR_droll(self, roll, pitch, yaw):
        # Partial derivative of R with respect to roll
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        Rz = np.array([
            [cy, -sy, 0],
            [sy, cy,  0],
            [0,  0,   1]
        ])
        Ry = np.array([
            [cp, 0, sp],
            [0, 1,  0],
            [-sp,0, cp]
        ])
        # derivative of Rx(roll)
        dRx = np.array([
            [0, 0, 0],
            [0, -sr, -cr],
            [0, cr, -sr]
        ])
        return Rz @ Ry @ dRx

    def dR_dpitch(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        Rz = np.array([
            [cy, -sy, 0],
            [sy, cy,  0],
            [0,  0,   1]
        ])
        # derivative of Ry(pitch)
        dRy = np.array([
            [-sp, 0, cp],
            [0, 0, 0],
            [-cp, 0, -sp]
        ])
        Rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr]
        ])
        return Rz @ dRy @ Rx

    def dR_dyaw(self, roll, pitch, yaw):
        cr = math.cos(roll)
        sr = math.sin(roll)
        cp = math.cos(pitch)
        sp = math.sin(pitch)
        cy = math.cos(yaw)
        sy = math.sin(yaw)
        # derivative of Rz(yaw)
        dRz = np.array([
            [-sy, -cy, 0],
            [cy, -sy, 0],
            [0, 0, 0]
        ])
        Ry = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp]
        ])
        Rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr]
        ])
        return dRz @ Ry @ Rx

    # ฟังก์ชันสำหรับ 𝐽 และอนุพันธ์ของมัน
    def J_from_euler(self, roll, pitch, yaw):
        # J converts body angular velocity to Euler angle rates for ZYX Euler angles.
        # J = [[1, sin(roll)*tan(pitch), cos(roll)*tan(pitch)],
        #      [0, cos(roll), -sin(roll)],
        #      [0, sin(roll)/cos(pitch), cos(roll)/cos(pitch)]]
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        tan_pitch = math.tan(pitch)
        J = np.array([
            [1, sin_roll*tan_pitch, cos_roll*tan_pitch],
            [0, cos_roll, -sin_roll],
            [0, sin_roll/cos_pitch, cos_roll/cos_pitch]
        ])
        return J

    def dJ_droll(self, roll, pitch, yaw):
        # Compute derivative of J with respect to roll.
        cos_roll = math.cos(roll)
        sin_roll = math.sin(roll)
        tan_pitch = math.tan(pitch)
        cos_pitch = math.cos(pitch)
        
        dJ = np.zeros((3,3))
        # Row 0:
        # d/droll (sin(roll)*tan_pitch) = cos_roll*tan_pitch
        dJ[0,1] = cos_roll * tan_pitch
        # d/droll (cos(roll)*tan_pitch) = -sin_roll*tan_pitch
        dJ[0,2] = -sin_roll * tan_pitch
        # Row 1:
        # d/droll (cos(roll)) = -sin_roll
        dJ[1,1] = -sin_roll
        # d/droll (-sin(roll)) = -cos_roll
        dJ[1,2] = -cos_roll
        # Row 2:
        # d/droll (sin(roll)/cos_pitch) = cos_roll/cos_pitch
        dJ[2,1] = cos_roll / cos_pitch
        # d/droll (cos(roll)/cos_pitch) = -sin_roll/cos_pitch
        dJ[2,2] = -sin_roll / cos_pitch
        return dJ

    def dJ_dpitch(self, roll, pitch, yaw):
        # Compute derivative of J with respect to pitch.
        sin_roll = math.sin(roll)
        cos_roll = math.cos(roll)
        cos_pitch = math.cos(pitch)
        sin_pitch = math.sin(pitch)
        sec_pitch2 = 1.0/(cos_pitch**2)
        
        dJ = np.zeros((3,3))
        # Row 0:
        # d/dpitch (sin(roll)*tan(pitch)) = sin(roll)*sec^2(pitch)
        dJ[0,1] = sin_roll * sec_pitch2
        # d/dpitch (cos(roll)*tan(pitch)) = cos(roll)*sec^2(pitch)
        dJ[0,2] = cos_roll * sec_pitch2
        # Row 1: derivatives are zero with respect to pitch because cos(roll) and -sin(roll) do not depend on pitch.
        # Row 2:
        # d/dpitch (sin(roll)/cos(pitch)) = sin(roll)*sin(pitch)/(cos(pitch)**2)
        dJ[2,1] = sin_roll * sin_pitch / (cos_pitch**2)
        # d/dpitch (cos(roll)/cos(pitch)) = cos(roll)*sin(pitch)/(cos(pitch)**2)
        dJ[2,2] = cos_roll * sin_pitch / (cos_pitch**2)
        return dJ

    def dJ_dyaw(self, roll, pitch, yaw):
        # For ZYX Euler angles, J does not depend explicitly on yaw.
        return np.zeros((3,3))

    def ekf_predict(self, dt):
        """
        Perform the prediction step: propagate state and covariance.
        """
        F = self.jacobian_F(self.xEst, dt)
        xPred = self.dynamic_model(self.xEst, dt)
        PPred = F @ self.PEst @ F.T + Q
        self.xEst = xPred
        self.PEst = PPred
    
    def ekf_update_odom(self, z):
        """
        Update step with odometry measurement.
        Measurement z (6-dim): [p_x, p_y, p_z, v_x, v_y, v_z]^T.
        Observation matrix H_odom (6x15):
          H_odom = [ [I₃, 0, I₃, 0, 0],
                     [0,  0, 0,  0, 0] ]
          More specifically, rows 0-2 pick state indices 0:3 (position),
          rows 3-5 pick state indices 6:9 (linear velocity).
        """
        H = np.zeros((6, 15))
        # H[0:3, 0:3] = np.eye(3)      # position
        H[3:6, 6:9] = np.eye(3)      # linear velocity
        
        zPred = H @ self.xEst
        y = z - zPred  # innovation
        S = H @ self.PEst @ H.T + R_odom
        K = self.PEst @ H.T @ np.linalg.inv(S)
        self.xEst = self.xEst + K @ y
        self.PEst = (np.eye(15) - K @ H) @ self.PEst
    
    def ekf_update_imu(self, z):
        """
        Update step with IMU measurement.
        Measurement z (9-dim): [roll, pitch, yaw, ω_x, ω_y, ω_z, a_x, a_y, a_z]^T.
        Observation matrix H_imu (9x15):
          Rows 0-2 pick orientation (state indices 3:6),
          Rows 3-5 pick angular velocity (state indices 9:12),
          Rows 6-8 pick linear acceleration (state indices 12:15).
        """
        H = np.zeros((9, 15))
        H[0:3, 3:6] = np.eye(3)      # orientation
        H[3:6, 9:12] = np.eye(3)     # angular velocity
        H[6:9, 12:15] = np.eye(3)    # linear acceleration
        H[8, 14] = 0
        zPred = H @ self.xEst
        y = z - zPred  # innovation
        S = H @ self.PEst @ H.T + R_imu
        K = self.PEst @ H.T @ np.linalg.inv(S)
        self.xEst = self.xEst + K @ y
        self.PEst = (np.eye(15) - K @ H) @ self.PEst
    
    def timer_callback(self):
        """
        Timer callback executed at a fixed rate.
        1. Compute dt.
        2. Perform the prediction step.
        3. If new odometry measurement is available, update using odom.
        4. If new IMU measurement is available, update using IMU.
        5. Publish the estimated pose.
        """
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time
        
        # Prediction step
        self.ekf_predict(dt)
        
        # Measurement update with odometry if available
        if self.new_odom and self.z_odom is not None:
            self.ekf_update_odom(self.z_odom)
            self.new_odom = False
        
        # Measurement update with IMU if available
        if self.new_imu and self.z_imu is not None:
            self.ekf_update_imu(self.z_imu)
            self.new_imu = False
        
        # Publish estimated pose (we publish position and orientation)
        self.publish_estimate()
    
    def publish_estimate(self):
        """
        Publish the estimated pose as a PoseStamped message.
        For visualization, we publish the position (state indices 0:3)
        and orientation (Euler angles from state indices 3:6, converted to quaternion).
        """
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"  # ปรับตาม frame ของระบบของคุณ
        
        # Position
        msg.pose.position.x = self.xEst[0, 0]
        msg.pose.position.y = self.xEst[1, 0]
        msg.pose.position.z = self.xEst[2, 0]
        
        # Orientation: use Euler angles from state indices 3:6
        roll = self.xEst[3, 0]
        pitch = self.xEst[4, 0]
        yaw = self.xEst[5, 0]
        q = quaternion_from_euler(roll, pitch, yaw)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        self.ekf_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    ekf_node = EKFFullNode()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
