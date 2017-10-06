from styx_msgs.msg import TrafficLight
from keras.models import load_model
import numpy as np
import tensorflow as tf
import cv2


class TLClassifier(object):
    def __init__(self):
        #TODO load classifier

	self.model = load_model('light_classification/_tf_lights_classifier.h5') 
        self.model._make_predict_function()
        self.graph = tf.get_default_graph()

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction


   
	img=cv2.resize(image,(224,224))
	img=img/255.0
	img = np.expand_dims(img, axis=0)
        with self.graph.as_default():
	    pred=self.model.predict(img)
	pclass=np.argmax(pred)

   	tf_color=TrafficLight.UNKNOWN
        if (pclass==1):
	    tf_color=TrafficLight.RED
        elif (pclass==2):
	    tf_color=TrafficLight.GREEN


        return tf_color

