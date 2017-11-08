from styx_msgs.msg import TrafficLight

import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction
	
	# Apply blur to image

	bgr_image_blur = cv2.medianBlur(image, 3)

	# Convert input image to HSV
	hsv_image = cv2.cvtColor(bgr_image_blur, cv2.COLOR_BGR2HSV)

	# Threshold the HSV image, keep only the red pixels
	lower_red_hue_range = cv2.inRange(hsv_image, (0, 100, 100), (5, 255, 255))
	upper_red_hue_range = cv2.inRange(hsv_image, (160, 100, 100), (179, 255, 255))

	# Threshold the image, keep only the yellow pixels
	yellow_hue_image = cv2.inRange(hsv_image, (25, 100, 100), (32, 255, 255))

	# Combine the above two images
	red_hue_image_combined = cv2.addWeighted(lower_red_hue_range, 1.0, upper_red_hue_range, 1.0, 0.0)
	red_hue_image_blur = cv2.GaussianBlur(red_hue_image_combined, (9, 9), 2, 2)
	yellow_hue_image_blur = cv2.GaussianBlur(yellow_hue_image, (9, 9), 2, 2)

	# Use the Hough transform to detect circles in the images
	red_circles = cv2.HoughCircles(red_hue_image_blur, cv2.HOUGH_GRADIENT, 1, red_hue_image_blur.shape[0] / 8.0, 100, 20, 20, 1)
	yellow_circles = cv2.HoughCircles(yellow_hue_image_blur, cv2.HOUGH_GRADIENT, 1, yellow_hue_image_blur.shape[0] / 8.0, 100, 20, 20, 1)
	### second to last entry is the min circle size (20 is the best size for the sim, real life may be different)

	# Loop over all detected circles and outline them on the original image
	if red_circles is not None:
		print("red light found")
		return TrafficLight.RED

	elif yellow_circles is not None:
		print("yellow light found")
		return TrafficLight.YELLOW

	else:
		return TrafficLight.GREEN
		
	# Traffic light designations taken from styx_msgs/TrafficLight
	#uint8 UNKNOWN=4
	#uint8 GREEN=2
	#uint8 YELLOW=1
	#uint8 RED=0

	# END TODO
