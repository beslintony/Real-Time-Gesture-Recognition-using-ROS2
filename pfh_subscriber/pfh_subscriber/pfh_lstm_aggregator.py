import statistics
from collections import Counter

import message_filters
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class LSTMAggregator(Node):
    """
    Create a subscriber node
    """

    def __init__(self):

        super().__init__(f'pfh_all_lstm_sub')

        self.n_subscribers = 8
        self.aggregate_block_dict = dict()
        self.aggregation_sub_dict = dict()
        self.n_nodes_block = []
        self.n_nodes_sub = []

        self.aggregate_with_filter()


    def aggregate_with_filter(self):
        msg_filter = []
        for i in range(1, self.n_subscribers + 1):
            msg_filter.append(message_filters.Subscriber(self, String, f"pfh_publisher_node_{i}",qos_profile=1))

        self.ts = message_filters.ApproximateTimeSynchronizer(msg_filter, 1, 0.5, allow_headerless=True)
        self.ts.registerCallback(self.callback_filters)

    def callback_filters(self,*msgs):
        for msg in msgs:
            key, class_cat, prediction = msg.data.split(":")
            self.aggregate_sub(key, class_cat, prediction)

    def lstm_callback(self, msg):
        # use a counter to keep track how many times the subscriber has received a message

        key, class_cat, prediction = msg.data.split(":")
        self.get_logger().info(key + " " + class_cat + " " + prediction)


    def aggregate_block(self, key, class_cat, prediction):
        if key not in self.aggregate_block_dict:
            self.aggregate_block_dict.update({int(key): (class_cat, float(prediction))})

        if len(self.aggregate_block_dict) == self.n_subscribers:
            prediction = threshold_prediction(list(self.aggregate_block_dict.values()))
            self.aggregate_block_dict.clear()

    def aggregate_sub(self, key, class_cat, prediction):        
        self.aggregation_sub_dict.update({int(key): (class_cat, float(prediction))})

        if len(self.aggregation_sub_dict) == self.n_subscribers:
            prediction = threshold_prediction(output_list=list(self.aggregation_sub_dict.values()), n_nodes=self.n_subscribers)


def threshold_prediction(output_list, threshold=0.65, n_nodes=0):
    freq_list = []

    threshold_output = ()
    arr_class = [i[0] for i in output_list[5:]]
    class_counts = Counter(arr_class)
    frequent_class = class_counts.most_common(1)

    x = 0
    for i in output_list:
        x = x + 1
        if i[0] == frequent_class[0][0]:
            freq_list.append(i[1])  # Append the prediction value
        if x > 5:
            print(f'At frame {x}, gesture is {i[0]}')
            # rclpy.get_logger().info(f'At frame {x}, gesture is {i[0]}')

    result = statistics.mean(freq_list)

    if result >= threshold:
        threshold_output = (frequent_class[0][0], result)
        print('prediction is "%s" \n' %threshold_output[0])

    else:
        print('prediction falls bellow threshold \n')
        threshold_output = (frequent_class[0][0], result)

    output_list.clear()
    freq_list.clear()
    return threshold_output


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create a subscriber
    lstm_aggregator = LSTMAggregator()

    rclpy.spin(lstm_aggregator)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    lstm_aggregator.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
