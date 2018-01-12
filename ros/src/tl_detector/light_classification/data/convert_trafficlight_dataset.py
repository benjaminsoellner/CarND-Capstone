import tensorflow as tf
import dataset_util
import yaml
import os
import sys
import getopt

# Dictionary with IDs for each classifier
assign_label_id = {
        'Red': 1,
        'RedLeft': 2,
        'RedRight': 3,
        'RedStraight': 4,
        'RedStraightLeft': 5,
        'Yellow': 6,
        'Green': 7,
        'GreenLeft': 8,
        'GreenStraight': 9,
        'GreenStraightLeft': 10,
        'GreenStraightRight': 11,
        'GreenRight': 12,
        'off': 13
    }

def create_tf_record(image_file, height, width, xmins, ymins, xmaxs, ymaxs, classes_id, classes_text):
  '''
  param xmins: List of normalized left x coordinates in bounding box (1 per box)
  param xmaxs: List of normalized right x coordinates in bounding box (1 per box)
  param ymins: List of normalized top y coordinates in bounding box (1 per box)
  param ymaxs: List of normalized bottom y coordinates in bounding box (1 per box)
  param classes_text: List of string class name of bounding box (1 per box)
  param classes: List of integer class id of bounding box (1 per box)
  '''
  image_data = tf.gfile.FastGFile(image_file, 'rb').read()
  encoded_image_data = tf.compat.as_bytes(image_data) # Encoded image bytes
  filename = tf.compat.as_bytes(image_file)
  image_format = 'png'.encode('utf8') # b'jpeg' or b'png'

  tf_single_dataset = tf.train.Example(features=tf.train.Features(feature={
      'image/height': dataset_util.int64_feature(height),
      'image/width': dataset_util.int64_feature(width),
      'image/filename': dataset_util.bytes_feature(os.path.basename(filename)),
      'image/source_id': dataset_util.bytes_feature(filename),
      'image/encoded': dataset_util.bytes_feature(image_data),
      'image/format': dataset_util.bytes_feature(image_format),
      'image/object/bbox/xmin': dataset_util.float_list_feature(xmins),
      'image/object/bbox/xmax': dataset_util.float_list_feature(xmaxs),
      'image/object/bbox/ymin': dataset_util.float_list_feature(ymins),
      'image/object/bbox/ymax': dataset_util.float_list_feature(ymaxs),
      'image/object/class/text': dataset_util.bytes_list_feature(classes_text),
      'image/object/class/label': dataset_util.int64_list_feature(classes_id),
  }))
  return tf_single_dataset

def convert_dataset(label_file, output_path):

    # The following image properties must be set according to the dataset you are importing
    height = 720 # Image height
    width = 1280 # Image width

    writer = tf.python_io.TFRecordWriter(output_path)

    images = read_labels(label_file)
    total_images = len(images)
    
    for index, image in enumerate(images):
        image_file = image['path']
        if ((index % 100) == 0):
            print("Converting image {0} of {1}".format(index, total_images))
        ymins = []
        xmins = []
        ymaxs = []
        xmaxs = []
        classes_id = []
        classes_text = []
        for box in image['boxes']:
            ymins.append(float(box['y_min'] / height))
            xmins.append(float(box['x_min'] / width))
            ymaxs.append(float(box['y_max'] / height))
            xmaxs.append(float(box['x_max'] / width))
            label = box['label']
            classes_id.append(int(assign_label_id[label]))
            classes_text.append(label.encode())
        #print(xmins,ymins,xmaxs,ymaxs,classes_text,classes_id)
        tf_single_dataset = create_tf_record(image_file, height, width, xmins, ymins, xmaxs, ymaxs, classes_id, classes_text)
        writer.write(tf_single_dataset.SerializeToString())
    print("Dataset successfully converted!)
    print("Writing tfrecord file... (this may take some time)")
    writer.close()

# From https://github.com/bosch-ros-pkg/bstld/blob/master/read_label_file.py
def read_labels(label_file, riib=False):
    """ Gets all labels within label file
    Image size from Bosch dataset: RGB 1280x720, RIIB 1280x736
    :param label_file: Path to label file in yaml format
    :param riib: If True, change path to labeled pictures
    :return: images: Labels for traffic lights
    """
    print("Reading label file... (this may take some time)")
    images = yaml.load(open(label_file, 'rb').read())

    for i in range(len(images)):
        images[i]['path'] = os.path.abspath(os.path.join(os.path.dirname(label_file), images[i]['path']))
        if riib:
            images[i]['path'] = images[i]['path'].replace('.png', '.pgm')
            images[i]['path'] = images[i]['path'].replace('rgb/train', 'riib/train')
            images[i]['path'] = images[i]['path'].replace('rgb/test', 'riib/test')
            for box in images[i]['boxes']:
                box['y_max'] = box['y_max'] + 8
                box['y_min'] = box['y_min'] + 8
    return images

def main(argv):
    label = ''
    outfile = ''
    try:
        opts, args = getopt.getopt(argv, "", ["label_file=", "output_file="])
    except getopt.GetoptError:
        print("Syntax error. Correct syntax: python convert_trafficlight_dataset.py --label_file=[yaml label file] --output_file=[path and filename to store tfrecord file]")
        sys.exit(2)
    for opt, arg in opts:
        if opt == "--label_file":
            label = arg
        if opt == "--output_file":
            outfile = arg
    convert_dataset(label, outfile)

if __name__ == "__main__":
    main(sys.argv[1:])
