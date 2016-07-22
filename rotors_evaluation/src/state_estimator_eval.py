#!/usr/bin/env python
# This is needed such that int / int gives a float instead of an int
from __future__ import division

import roslib
roslib.load_manifest('rotors_evaluation')

from math import *
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot

from rosbag_tools import analyze_bag, helpers


__author__ = "Fadri Furrer, Michael Burri, Markus Achtelik"
__copyright__ = ("Copyright 2015, Fadri Furrer & Michael Burri & "
                 "Markus Achtelik, ASL, ETH Zurich, Switzerland")
__credits__ = ["Fadri Furrer", "Michael Burri", "Markus Achtelik"]
__license__ = "ASL 2.0"
__version__ = "0.1"
__maintainer__ = "Pavel Vechersky"
__email__ = "pvechersky@gmail.com"
__status__ = "Development"


def main():
    [ab, plot, begin_time, total_end_time, rms_calc_time, settling_radius,
     min_settled_time, first_waypoint_evaluation_delay] = helpers.initialize()

    # Get the time when the bag recording started
    start_time = ab.bag_time_start.to_sec()

    # Get the relative begin and end time of estimated odometry messages
    begin_time = ab.odom[0].bag_time[0].to_sec() - start_time
    end_time = ab.odom[0].bag_time[-1].to_sec() - start_time

    # Get the first ground truth position
    first_gt_pos = analyze_bag.XYZWithTime()
    first_gt_pos.x = ab.pos[0].x[0]
    first_gt_pos.y = ab.pos[0].y[0]
    first_gt_pos.z = ab.pos[0].z[0]

    # Get the first estimated position
    first_est_pos = analyze_bag.XYZWithTime()
    first_est_pos.x = ab.odom[0].pos.x[0]
    first_est_pos.y = ab.odom[0].pos.y[0]
    first_est_pos.z = ab.odom[0].pos.z[0]

    # Get the first ground truth orienation
    first_gt_quat = analyze_bag.QuatWithTime()
    first_gt_quat.w = ab.quat[0].w[0]
    first_gt_quat.x = ab.quat[0].x[0]
    first_gt_quat.y = ab.quat[0].y[0]
    first_gt_quat.z = ab.quat[0].z[0]

    # Get the first estimated orienation
    first_est_quat = analyze_bag.QuatWithTime()
    first_est_quat.w = ab.odom[0].rot.w[0]
    first_est_quat.x = ab.odom[0].rot.x[0]
    first_est_quat.y = ab.odom[0].rot.y[0]
    first_est_quat.z = ab.odom[0].rot.z[0]

    # Extract the estimates
    position_estimates = ab.odom[0].pos
    orientation_estimates = ab.odom[0].rot

    # Initialize the error arrays
    position_errors = []
    orientation_errors = []

    # Compare each ground truth position to the estimated position
    for index in range(len(position_estimates.x)):
        # Shift the message time to be relative to when we started recording
        ab.odom[0].time[index] -= start_time

    	# Make the position objects for comparison relative to the first 
        # position
    	gt_pos = analyze_bag.XYZWithTime()
    	gt_pos.x = ab.pos[0].x[index] - first_gt_pos.x
    	gt_pos.y = ab.pos[0].y[index] - first_gt_pos.y
    	gt_pos.z = ab.pos[0].z[index] - first_gt_pos.z

    	est_pos = analyze_bag.XYZWithTime()
    	est_pos.x = ab.odom[0].pos.x[index] - first_est_pos.x
    	est_pos.y = ab.odom[0].pos.y[index] - first_est_pos.y
    	est_pos.z = ab.odom[0].pos.z[index] - first_est_pos.z

        # Make the quaternion objects for comparison relative to the first
        # orientation
        gt_quat = analyze_bag.QuatWithTime()
        gt_quat.w = ab.quat[0].w[index]
        gt_quat.x = ab.quat[0].x[index]
        gt_quat.y = ab.quat[0].y[index]
        gt_quat.z = ab.quat[0].z[index]
        rel_gt_quat = quat_relative_rotation(gt_quat, first_gt_quat)

        est_quat = analyze_bag.QuatWithTime()
        est_quat.w = ab.odom[0].rot.w[index]
        est_quat.x = ab.odom[0].rot.x[index]
        est_quat.y = ab.odom[0].rot.y[index]
        est_quat.z = ab.odom[0].rot.z[index]
        rel_est_quat = quat_relative_rotation(est_quat, first_est_quat)

        # Compute the error between the two positions
    	position_errors.append(xyz_dist_error(gt_pos, est_pos))

        # Compute the error between the two quaternions
        orientation_errors.append(quat_dist_error(rel_gt_quat, rel_est_quat))

    # Plot the position errors
    if plot and len(position_errors) > 0:
        x_range = [begin_time - 2, end_time + 2]

        """Plot the position errors."""
        fig = pyplot.figure()
        fig.suptitle("Position error")
        a_x = fig.add_subplot(111)
        a_x.plot(ab.odom[0].time, position_errors, 'b', label='pos_error')

        y_range = [0, max(position_errors)]

        pyplot.xlabel('time [s]')
        pyplot.ylabel('position error [m]')
        pyplot.xlim(x_range)
        pyplot.ylim(y_range)
        pyplot.grid(b=True, which='both')

        file_name = 'position_error.png'
        pyplot.savefig(file_name)

        """Plot the orientation errors."""
        fig = pyplot.figure()
        fig.suptitle("Orientation error")
        a_x = fig.add_subplot(111)
        a_x.plot(ab.odom[0].time, orientation_errors, 'b', 
            label='orientation_error')

        y_range = [0, max(orientation_errors)]

        pyplot.xlabel('time [s]')
        pyplot.ylabel('orientation error')
        pyplot.xlim(x_range)
        pyplot.ylim(y_range)
        pyplot.grid(b=True, which='both')

        file_name = 'orientation_error.png'
        pyplot.savefig(file_name)


def xyz_dist_error(xyz_one, xyz_two):
	x_error = xyz_one.x - xyz_two.y
	y_error = xyz_one.y + xyz_two.x
	z_error = xyz_one.z - xyz_two.z
	sum_of_squares = x_error ** 2 + y_error ** 2 + z_error ** 2
	dist = sum_of_squares ** 0.5
	return dist


def quat_dist_error(quat_one, quat_two):
    dist = 1 - fabs(quat_dot_product(quat_one, quat_two))
    return dist


def quat_dot_product(quat_one, quat_two):
    dot = quat_one.w * quat_two.w
    dot += quat_one.x * quat_two.x
    dot += quat_one.y * quat_two.y
    dot += quat_one.z * quat_two.z
    return dot


def quat_inverse(quat):
    inverse = analyze_bag.QuatWithTime()
    dot = quat_dot_product(quat, quat)
    inverse.w = quat.w / dot
    inverse.x = -quat.x / dot
    inverse.y = -quat.y / dot
    inverse.z = -quat.z / dot
    return inverse


def quat_multiply(q, r):
    product = analyze_bag.QuatWithTime()
    product.w = q.w * r.w - q.x * r.x - q.y * r.y - q.z * r.z
    product.x = q.x * r.w + q.w * r.x - q.z * r.y + q.y * r.z
    product.y = q.y * r.w + q.z * r.x + q.w * r.y - q.x * r.z
    product.z = q.z * r.w - q.y * r.x + q.x * r.y + q.w * r.z
    return product


def quat_relative_rotation(quat_one, quat_two):
    inv_quat_two = quat_inverse(quat_two)
    rel_quat = quat_multiply(quat_one, inv_quat_two)
    return rel_quat


def quat_to_euler(q):
    roll = atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
    pitch = asin(2 * (q.w * q.y - q.z * q.x))
    yaw = atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.x * q.x + q.y * q.y))
    return [roll, pitch, yaw]


if __name__ == "__main__":
    main()