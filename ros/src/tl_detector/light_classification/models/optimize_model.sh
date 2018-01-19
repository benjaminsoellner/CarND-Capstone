#!/bin/bash

# see:
# https://github.com/tensorflow/tensorflow/tree/master/tensorflow/tools/graph_transforms/#introduction

# prequisite: install tensorflow-from-src
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")
TF_SRC_DIR="/var/team-herbie/tensorflow-src/tensorflow"
TF_MODEL_ORIGINAL="$SCRIPTPATH/frozen_inference_graph.pb"
TF_MODEL_OPTIMIZED="$SCRIPTPATH/frozen_inference_graph_opt.pb"
TF_MODEL_TENSOR_IN="image_tensor"
TF_MODEL_TENSOR_OUT="detection_boxes,detection_scores,detection_classes,num_detections"

cd $TF_SRC_DIR

bazel build tensorflow/tools/graph_transforms:transform_graph
bazel-bin/tensorflow/tools/graph_transforms/transform_graph \
--in_graph=$TF_MODEL_ORIGINAL \
--out_graph=$TF_MODEL_OPTIMIZED \
--inputs=$TF_MODEL_TENSOR_IN \
--outputs=$TF_MODEL_TENSOR_OUT \
--transforms='
  add_default_attributes
  strip_unused_nodes(type=float, shape="1,299,299,3")
  fold_constants(ignore_errors=true)
  fold_batch_norms
  fold_old_batch_norms
  fuse_resize_and_conv
  quantize_weights
  strip_unused_nodes
  sort_by_execution_order'
