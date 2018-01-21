import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np
import cv2
import time

DETECTION_LIMIT = 0.5

SIMULATOR_MODE = True

class TLClassifier(object):
    def __init__(self):
        rospy.logdebug("TLClassifier::__init__ started.")

        # Location of pre-trained inference model
        model_path = './light_classification/models/frozen_inference_graph.pb'
        self.inference_graph = tf.Graph()

        with self.inference_graph.as_default():
            current_graph_def = tf.GraphDef()

            # Load graph from file and imports it into the default graph
            with tf.gfile.GFile(model_path, 'rb') as file:
                file_graph = file.read()
                current_graph_def.ParseFromString(file_graph)
                tf.import_graph_def(current_graph_def, name='')

            self.image_tensor = self.inference_graph.get_tensor_by_name('image_tensor:0')
            self.d_boxes = self.inference_graph.get_tensor_by_name('detection_boxes:0')
            self.d_scores = self.inference_graph.get_tensor_by_name('detection_scores:0')
            self.d_classes = self.inference_graph.get_tensor_by_name('detection_classes:0')
            self.num_d = self.inference_graph.get_tensor_by_name('num_detections:0')

        self.sess = tf.Session(graph = self.inference_graph)

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        start_time = time.time()
        # Runs inference on one image on the loaded graph
        with self.inference_graph.as_default():
            # Image is expanded to 4 dims - 1st dim batch size (=1)
            image_4d = np.expand_dims(image, axis=0)
            (boxes, scores, classes, num_det) = self.sess.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d], feed_dict = {self.image_tensor: image_4d})

        end_time = time.time()

        rospy.logdebug("Time for classification: {0}s".format(end_time - start_time))

        # Inference returns a (fixed) total of self.num_d detections - even those with low probabilities
        r_boxes = []
        r_scores = []
        r_classes = []
        idx = 0

        # If the highest score is below detection probability, there is no traffic light visible or not clear enough, return unknown
        if scores[0][0] < DETECTION_LIMIT:
            rospy.logdebug("No traffic light detected: UNKNOWN")
            return TrafficLight.UNKNOWN

        # Keep all results above probability of DETECTION_LIMIT
        while (scores[0][idx] > DETECTION_LIMIT):
            r_scores.append(scores[0][idx])
            r_boxes.append(boxes[0][idx])
            r_classes.append(classes[0][idx])
            idx+=1

        # Classes for red, yellow and green lights
        red = 1
        yellow = 2
        green = 3
        yellow_or_not = False

        img_shape = image.shape
        height = img_shape[0]
        width = img_shape[1]

        # In simulator mode, feed each detection box to a color detector
        # and return the classification result
        if SIMULATOR_MODE:
            
            rectangles = [] # List of rectangles to cut out regions

            # Loops through all boundary boxes with found traffic lights and expands them to full image size (0..1 to 0..image_height and width)
            for idx, box in enumerate(r_boxes):
                box_norm = [int(box[0]*height), int(box[1]*width), int(box[2]*height), int(box[3]*width)]
                rectangles.append(box_norm)

            # Loops through all the boundary boxes and detects their dominant light color
            for rect in rectangles:
                crop_image = image[rect[0]:rect[2], rect[1]:rect[3]]
                classification = detect_color(crop_image)
                if classification == red:
                    rospy.logdebug("Red traffic light detected")
                    return TrafficLight.RED
                elif classification == yellow:
                    yellow_or_not = True

            if yellow_or_not:
                rospy.logdebug("Yellow traffic light detected")
                return TrafficLight.YELLOW
            else:
                rospy.logdebug("Green traffic light detected")
                return TrafficLight.GREEN

        # If not in simulator mode, use the detection result from the FRCNN classifier directly
        else:
            for classification in r_classes:
                if classification == red:
                    rospy.logdebug("Red traffic light detected")
                    return TrafficLight.RED
                elif classification == yellow:
                    yellow_or_not = True
            if yellow_or_not:
                rospy.logdebug("Yellow traffic light detected")
                return TrafficLight.YELLOW
            else:
                rospy.logdebug("Green traffic light detected")
                return TrafficLight.GREEN

# Detects the dominant color among red, green and yellow in an image
# Used to determine a traffic light color in a cropped image of the traffic light
def detect_color(crop_image):
    # Thresholds of Hue values in HSV colorspace for red, green and yellow
    thr_low_green = 41
    thr_high_green = 80
    thr_low_yellow = 30
    thr_high_yellow = 40
    thr_low_red = 170
    thr_high_red = 10

    shape = crop_image.shape
    # crop_size = float(shape[0] * shape[1])

    # Convert image to HSV colorspace
    tl_hsv = cv2.cvtColor(crop_image, cv2.COLOR_RGB2HSV)
    # Select only the Hue channel (channel 0)
    tl_hue = tl_hsv[:, :, 0]

    # Masks in the image with True for each pixel in the range for red, yellow and green, False otherwise
    green_threshold = ((tl_hue > thr_low_green) & (tl_hue < thr_high_green))
    red_threshold = ((tl_hue > thr_low_red) | (tl_hue < thr_high_red))
    yellow_threshold = ((tl_hue > thr_low_yellow) & (tl_hue < thr_high_yellow))
    
    # Calculates the ratio of red, green and yellow among all pixels in the image
    green_ratio = (green_threshold == True).sum()  # / crop_size
    red_ratio = (red_threshold == True).sum()  # / crop_size
    yellow_ratio = (yellow_threshold == True).sum()  # / crop_size

    # Finds the index with the highest ratio
    detection = np.argmax([red_ratio, yellow_ratio, green_ratio])

    # Index starts from zero, red from 1, therefore increase by 1
    return detection+1
