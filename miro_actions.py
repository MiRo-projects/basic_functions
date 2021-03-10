from colour import Color
from miro_ros_interface import MiRoPublishers

miro_pub = MiRoPublishers()


class IllumActions:
	@staticmethod
	def pulse(duration, colour, step: int = 5, speed: int = 100):
		# Convert colour name to full RGB hex value (removing # character)
		# TODO: Skip this if provided colour is already an RGB value
		rgb = Color(colour).hex_l[1:]

		def pub_colour(alpha):
			# Append colour to alpha value for complete ARGB word
			argb = int(hex(alpha) + rgb, 16)

			# TODO: Add code to specify individual lights
			miro_pub.pub_illum(all=argb)

			# Sleep time is just reciprocal of speed
			miro_pub.sleep(1 / speed)

		for _ in range(duration):
			for a in range(0, 255, step):
				pub_colour(a)
			for a in range(255, 0, -step):
				pub_colour(a)

	# TODO: Create 'sweep' action that sweeps a pulse forwards or backwards
