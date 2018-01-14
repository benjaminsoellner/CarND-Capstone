#!/usr/bin/env python
"""
Command line tool to inspect tensors in a tensorflow model file (*.pb) and to
run one image through the model (assuming hardcoded tensor names).

Usage:
python model_inspect_cli.py ops <model.pb>
  to inspect the model for operations
python model_inspect_cli.py test <model.pb> <image>
  to test model on image
"""

import tensorflow as tf
import sys
import pprint
import scipy.misc
import numpy


# Tensor names, modify based on neural network architecture
IMAGE_TENSOR = "image_tensor:0"
DETECTION_CLASSES_TENSOR = "detection_classes:0"
DETECTION_SCORES_TENSOR = "detection_scores:0"


def load_graph(model_pb, use_xla=False):
    """Loads neural network graph from *.pb file.
    See: https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/595f35e6-b940-400f-afb2-2015319aa640/lessons/b6de6f99-281f-44e8-a9df-196577204638/concepts/d2abd412-0ec6-436c-b961-a851b5eb35f8"""

    jit_level = 0
    config = tf.ConfigProto()
    if use_xla:
        jit_level = tf.OptimizerOptions.ON_1
        config.graph_options.optimizer_options.global_jit_level = jit_level

    with tf.Session(graph=tf.Graph(), config=config) as sess:
        gd = tf.GraphDef()
        with tf.gfile.Open(model_pb, 'rb') as f:
            data = f.read()
            print("... Loading tensorflow model ...")
            gd.ParseFromString(data)
        tf.import_graph_def(gd, name='')
        return sess


def run_ops(model_pb):
    """Outputs all named operations and the number of operations in a
    tensorflow model_pb file to the screen. Executes when calling the script
    with "ops" argument."""
    # Load graph from protobuf file
    sess = load_graph(model_pb)
    # Get all operations
    operations = sess.graph.get_operations()
    # Print
    pprint.pprint(operations)
    print("===========")
    print("# of Operations: " + str(len(operations)))


def run_test(model_pb, image):
    """Runs a single image file through the network."""
    # Load graph from protobuf file
    sess = load_graph(model_pb)
    # Load image and resize to batch array
    image_data = scipy.misc.imread(image)
    image_shape = image_data.shape
    image_reshaped = numpy.reshape(image_data,
            (1, image_shape[0], image_shape[1], image_shape[2]))
    # Get all tensors from the model that we need to fill
    image_tensor = sess.graph.get_tensor_by_name(IMAGE_TENSOR)
    classes_tensor = sess.graph.get_tensor_by_name(DETECTION_CLASSES_TENSOR)
    scores_tensor = sess.graph.get_tensor_by_name(DETECTION_SCORES_TENSOR)
    # Run inference
    scores_data, classes_data = sess.run(
            [scores_tensor, classes_tensor],
            feed_dict={ image_tensor: image_reshaped }
        )
    # Output results
    print("detection_scores: ")
    pprint.pprint(scores_data)
    print("detection_classes: ")
    pprint.pprint(classes_data)


if __name__ == '__main__':
    # Logic to handle script arguments
    if len(sys.argv) < 3 or len(sys.argv) > 4:
        print(__doc__)
    else:
        operation = sys.argv[1]
        model_pb = sys.argv[2]
        image = sys.argv[3] if len(sys.argv) >= 4 else None
        if operation == "ops":
            run_ops(model_pb)
        elif operation == "test":
            run_test(model_pb, image)
        else:
            print(__doc__)
