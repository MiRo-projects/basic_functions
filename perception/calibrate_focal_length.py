import apriltag
import cv2
import math
import numpy as np
import miro_ros_interface as mri

# MiRo-E parameters
import constants as con

# USAGE: Set MiRo in front of an example AprilTag
# (See here: https://april.eecs.umich.edu/software/apriltag)
# Enter the actual size of the tag and the distance from the camera, and ensure focal_length == None
# Run the script and note down the focal length given
# Enter the focal length and re-run the script; adjust MiRo's distance from the tag and check for accuracy

miro_per = mri.MiRoPerception()

# Set the actual size and distance of the AprilTag being used for calibration
# (Units don't matter as long as you use the same for both)
tag_size = 5
tag_distance = 31

# Enter a value here when you obtain a focal length to test your distance measurements are accurate
focal_length = None

# TODO: Find out more about what these options do
options = apriltag.DetectorOptions(
	families='tag36h11',
	border=1,
	nthreads=4,
	quad_decimate=1.0,
	quad_blur=0.0,
	refine_edges=True,
	refine_decode=True,
	refine_pose=True,
	debug=False,
	quad_contours=True,
	)

detector = apriltag.Detector(options)

# TODO: Allow the user to exit this loop cleanly
for i in range(400):
	# Is undistortion is actually useful for AprilTags?
	# If you have your own MTX and DIST calibration values, use them here
	mod_l = cv2.undistort(np.array(miro_per.caml), con.MTX, con.DIST, None)
	gray_im = cv2.cvtColor(mod_l, cv2.COLOR_RGB2GRAY)

	# Get all detected tags
	result = detector.detect(gray_im)

	if result:
		# Adapted from https://www.pyimagesearch.com/2020/11/02/apriltag-with-python/
		for r in result:
			# Extract the bounding box (x, y)-coordinates for the AprilTag
			# and convert each of the (x, y)-coordinate pairs to integers
			(ptA, ptB, ptC, ptD) = r.corners
			ptB = (int(ptB[0]), int(ptB[1]))
			ptC = (int(ptC[0]), int(ptC[1]))
			ptD = (int(ptD[0]), int(ptD[1]))
			ptA = (int(ptA[0]), int(ptA[1]))

			# Draw the bounding box of the AprilTag detection
			cv2.line(mod_l, ptA, ptB, (255, 0, 0), 3)
			cv2.line(mod_l, ptB, ptC, (0, 255, 0), 3)
			cv2.line(mod_l, ptC, ptD, (0, 0, 255), 3)
			cv2.line(mod_l, ptD, ptA, (255, 0, 255), 3)

			# Draw the center (x, y)-coordinates of the AprilTag
			(cX, cY) = (int(r.center[0]), int(r.center[1]))
			cv2.circle(mod_l, (cX, cY), 5, (0, 0, 255), -1)

			# Draw the tag family on the image
			tagFamily = r.tag_family.decode("utf-8")
			cv2.putText(mod_l, tagFamily, (ptA[0], ptA[1] - 15),
			            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

			# Get the mean perceived size of the tag
			# TODO: Is there a better / more accurate way to derive this value?
			ab = math.hypot(ptB[0] - ptA[0], ptB[1] - ptA[1])
			bc = math.hypot(ptC[0] - ptB[0], ptC[1] - ptB[1])
			cd = math.hypot(ptD[0] - ptC[0], ptD[1] - ptC[1])
			da = math.hypot(ptA[0] - ptD[0], ptA[1] - ptD[1])
			tag_perceived = np.mean([ab, bc, cd, da])

			# Output either focal length or estimated distance on the image
			if focal_length is not None:
				est_distance = (tag_size * focal_length) / tag_perceived
				cv2.putText(mod_l, 'Distance: {0:.2f}cm'.format(est_distance), (ptA[0], ptA[1] - 30),
				            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
			else:
				est_focal_length = (tag_perceived * tag_distance) / tag_size
				cv2.putText(mod_l, 'Focal length: {0:.2f}'.format(est_focal_length), (ptA[0], ptA[1] - 30),
				            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

				print('Estimated focal length: {}'.format(est_focal_length))

			cv2.imshow('AprilTag calibration', mod_l)
			cv2.waitKey(5)

	else:
		print('No AprilTags in view!')

		cv2.imshow('AprilTag calibration', mod_l)
		cv2.waitKey(5)
