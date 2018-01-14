import rospy
from styx_msgs.msg import TrafficLight
import tensorflow as tf
import numpy as np

DETECTION_LIMIT = 0.5

class TLClassifier(object):
    def __init__(self):
        rospy.logdebug("TLClassifier::__init__ started.")

        # Location of pre-trained inference model
        model_path = './light_classification/model/frozen_inference_graph.pb' 
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
        # Runs inference on one image on the loaded graph
        with self.inference_graph.as_default():
            # Image is expanded to 4 dims - 1st dim batch size (=1)
            image_4d = np.expand_dims(img, axis=0)
            (boxes, scores, classes, num_det) = self.sess.run([self.d_boxes, self.d_scores, self.d_classes, self.num_d], feed_dict = {self.image_tensor: image_4d})
        
        # Inference returns a (fixed) total of self.num_d detections - even those with low probabilities
        r_boxes, r_scores, r_classes = []
        idx = 0

        # If the highest score is below detection probability, there is no traffic light visible or not clear enough, return unknown
        if scores[0][0] < DETECTION_LIMIT:
            return TrafficLight.UNKNOWN

        # Keep all results above probability of DETECTION_LIMIT
        while (scores[0][idx] > DETECTION_LIMIT):
            r_scores.append(scores[0][idx])
            r_boxes.append(boxes[0][idx])
            r_classes.append(classes[0][idx])
            idx+=1

        # Classes for red, yellow and green lights
        red = [1, 2, 3, 4, 5]
        yellow = [6]
        green = [7, 8, 9, 10, 11, 12]
        yellow_or_not = False
        
        for classification in r_classes:
            if classification in red:
                return TrafficLight.RED
            elif classification in yellow:
                yellow_or_not = True
        if yellow_or_not:
            return TrafficLight.YELLOW
        else:
            return TrafficLight.GREEN

