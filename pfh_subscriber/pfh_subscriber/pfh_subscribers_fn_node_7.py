import operator
import os

import numpy as np
import rclpy
import tensorflow as tf
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String

os.environ["CUDA_VISIBLE_DEVICES"] = "-1"

class HistogramSubscriber(Node):
    """
    Create a subscriber node
    """

    def __init__(self, offset, model):

        super().__init__('pfh_sub_7')

        self.declare_parameter('cycles',8)

        self.model = model
        self.offset = offset
        self.memory = []
        self.cycles = self.get_parameter('cycles').value
        self.start_time = self.get_clock()
        self.counter = 0

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'pf_histogram',
            self.histogram_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            String, f"pfh_publisher_node_{offset}", 1)

    def subscribe(self):
        self.subscription = self.create_subscription(
                    Float32MultiArray,
                    'pf_histogram',
                    self.histogram_callback,
                    1)


    def publish(self, msg):
        self.publisher.publish(msg)

    def histogram_callback(self, histogram):
        # use a counter to keep track how many times the subscriber has received a message

        self.counter = self.counter + 1

        self.memory.append(histogram.data)
        if(self.counter == self.offset):
            self.send_to_lstm(self.memory)
            self.memory.clear()

    def send_to_lstm(self, data):
        npdata = np.array(data)[np.newaxis, :, :]
        output = self.evaluate_prediction(npdata)

        self.publish(output)

    def evaluate_prediction(self, data):
        prediction_output = self.run_single_prediction(self.model, data)
        prediction_dict = prediction_output['prediction']

        # get the most confident Prediction from the dict
        result = max(prediction_dict.items(), key=operator.itemgetter(1))
        
        output = String()

        output.data = str(self.offset) + ":" + str(result[0]) + ":" + str(result[1])

        return output

    def run_single_prediction(self, model, data):

        if (self.counter - self.offset) % (self.cycles) == 0:
            model.reset_states()
            self.counter = 0

        prediction = model.predict(data).tolist()
        result = {
            'prediction': {
                'Thumbs Up': prediction[0][0],
                'Thumbs Down': prediction[0][1],
                'Swipe Left': prediction[0][2],
                'Swipe Right': prediction[0][3],
                'One Snap': prediction[0][4],
                'Two Snaps': prediction[0][5]
            }
        }

        return result


def main(args=None):

    dirname = os.path.dirname(__file__)
    path = os.path.join(dirname, 'savedModel/newhandGestureModel.h5')
    model = tf.keras.models.load_model(path)

    offset = 7

    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create a subscriber
    pfh_sub= HistogramSubscriber(offset, model)

    # Spin the node so the callback function is called.
    # Pull messages from any topics this node is subscribed to.
    rclpy.spin(pfh_sub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    pfh_sub.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
