#! /usr/bin/env python

import numpy as np
import tensorflow as tf
import cv2

from os import path


class CluedoClassifier:

    CLUEDO_IMG_FILE = r"cluedo_character.png"
    MODEL_FILE = "final_graph.pb"
    LABEL_FILE = "cluedo_labels.txt"
    INPUT_HEIGHT = 224
    INPUT_WIDTH = 224
    INPUT_MEAN = 0
    INPUT_STD = 255
    INPUT_LAYER = "Placeholder"
    OUTPUT_LAYER = "final_result"

    def __init__(self):
        self._camera = cv2.imread(self.CLUEDO_IMG_FILE, 1)
        self._character = None

    def load_graph(self, model_file):
        graph = tf.Graph()
        graph_def = tf.GraphDef()
        #graph_def = tf.compat.v1.GraphDef() #if tf version 2

        with open(model_file, "rb") as f:
            graph_def.ParseFromString(f.read())
        with graph.as_default():
            tf.import_graph_def(graph_def)

        return graph

    def load_labels(self, label_file):
        label = []
        proto_as_ascii_lines = tf.gfile.GFile(label_file).readlines()
        #proto_as_ascii_lines = tf.compat.v1.gfile.GFile(label_file).readlines() #tf version 2

        for l in proto_as_ascii_lines:
            label.append(l.rstrip())
        return label

    def grab_video_feed(self):
        #grabbed, frame = camera.read()
        #return frame if grabbed else None
        frame = self._camera
        return frame

    def classify(self):
        #load tensorflow graph and labels file
        graph = self.load_graph(self.MODEL_FILE)
        labels = self.load_labels(self.LABEL_FILE)

        #set the inputs and outputs for the
        input_name = "import/" + self.INPUT_LAYER
        output_name = "import/" + self.OUTPUT_LAYER
        input_operation = graph.get_operation_by_name(input_name)
        output_operation = graph.get_operation_by_name(output_name)
        #with tf.compat.v1.Session(graph=graph) as sess: #tf version 2
        with tf.Session(graph=graph) as sesh:
            while True:
                live_frame = self.grab_video_feed()
                if live_frame is None:
                    raise SystemError('Issue grabbing the frame')

                resized_frame = cv2.resize(live_frame, (self.INPUT_HEIGHT,self.INPUT_WIDTH), interpolation=cv2.INTER_CUBIC)
                numpy_frame = np.float32(resized_frame)

                #ensure frame have same mean and range
                normalised = cv2.normalize(numpy_frame, None, alpha=0, beta=1, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_32F)
                t = np.expand_dims(normalised, axis=0)

                #run forward pass of framne through net
                results = sesh.run(output_operation.outputs[0], {input_operation.outputs[0]: t})
                #find the highest output
                results = np.squeeze(results)
                top_k = results.argsort()[-7:][::-1]
                self._character = labels[top_k[0]]
                print("[FOUND]: " + self._character)
                break

        # Now we can write out the classified character to a file
        with open("cluedo_character.txt", "w") as fp:
            fp.write(self._character)


if __name__ == "__main__":
    cluedo_classifier = CluedoClassifier()
    cluedo_classifier.classify()
