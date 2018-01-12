# Traffic Light dataset conversion to Tensorflow data format

## Background
Tensorflow prefers to work with training and test data in its own file format tfrecord. tfrecord is a single file format containing all input data, such as images, as well as additional notations like labels or bounding boxes. The Tensorflow object detection library requires you to convert your dataset to tfrecord.

## Traffic light dataset
The following instructions and the conversion file are based on the Bosch small traffic light dataset which can be downloaded [here](https://hci.iwr.uni-heidelberg.de/node/6132). At the bottom of the page, you can request a download link from where to download the files. The dataset is available with both training and test set, as well as two different color maps (RGB and RIIB). After downloading and unpacking the dataset, please put the files in the following way relative to the convert_trafficlight_dataset.py file (please also note the label file)
```
data
| convert_trafficlight_dataset.py
| train.yaml (or test.yaml)
|--train
|  | rgb
|  | riib
|--test
   | rgb
   | riib
```

## Adapting the conversion script
If you have a dataset with other classes than in the training images from the Bosch dataset, you may have to adapt the classes at the top of the script in assign_label_id. Please change the labels in the trafficlight_label_map.pbtxt file accordingly so that they match each other 1:1.
You may also have to change the image format 
```
image_format = 'png'.encode('utf8') # b'jpeg' or b'png'
```
or the image size in the convert_dataset function.

## Running the conversion script
Simply type in the following command:
```
python convert_trafficlight_dataset.py --label_file=[yaml label file] --output_file=[path and filename to store tfrecord file]
```

## More on other conversion scripts
You can find more information to generate your own dataset from other data on the Tensorflow github page [here](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/preparing_inputs.md).
