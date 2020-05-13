"""
Note: #!pip2 install tensorflow-serving-api==1.12.0
"""
from .videoprocessinglayers import VideoProcessingLayer

import cv2

import tensorflow as tf
from tensorflow_serving.apis import predict_pb2
from tensorflow_serving.apis import prediction_service_pb2_grpc
import grpc
import numpy as np

from .LabelMap_pb2 import LabelMap
from google.protobuf import text_format
import random

DEFAULT_TIMEOUT_IN_SEC = 10.0

def load_input_tensor(input_image):
    #image_np = load_image_into_numpy_array(input_image)
    image_np_expanded = np.expand_dims(input_image, axis=0).astype(np.uint8)
    tensor = tf.contrib.util.make_tensor_proto(image_np_expanded)
    return tensor


class ObjectDetectionLayer(VideoProcessingLayer):
    def __init__(self, endpoint, model_spec_name, 
                 timeout=DEFAULT_TIMEOUT_IN_SEC ):
        VideoProcessingLayer.__init__(self)
        self.endpoint =  endpoint
        self.model_spec_name = model_spec_name
        self.timeout = timeout
        pass
    
    def setup(self, ctx):
        self.channel = grpc.insecure_channel(self.endpoint)
        pass
    
    def process(self, ctx):
        # Send request
        # See prediction_service.proto for gRPC request/response details.
        request = predict_pb2.PredictRequest()
        request.model_spec.name = self.model_spec_name
        request.model_spec.signature_name = 'serving_default'
        input_tensor = load_input_tensor(ctx["OUTPUT_FRAME"])
        request.inputs['inputs'].CopyFrom(input_tensor)

        stub = prediction_service_pb2_grpc.PredictionServiceStub(self.channel)
        result = stub.Predict(request, self.timeout)  # 10 secs timeout
    
        output_dict = {}
        output_dict["classes"] = np.squeeze( result.outputs["detection_classes"].float_val).astype(np.uint8)
        output_dict["detection_boxes"] = np.reshape(result.outputs["detection_boxes"].float_val, (-1, 4))
        output_dict["detection_scores"] = np.squeeze(result.outputs["detection_scores"].float_val)
        ctx["OBJECT_DETECTION_OUTPUT"] = output_dict
    
    def release(self, ctx):
        self.channel.close()
        pass



def load_class_names_from_prototxt(filename):
    label_map = LabelMap()
    class_names = {}
    try:
        with open(filename, 'rb') as fin:
            file_content = fin.read()
            text_format.Parse(file_content.decode('UTF-8'),label_map, allow_unknown_extension=True)
            print("Parse file [%s] with text format successfully." % (filename))
            for it in label_map.item:
                class_names[it.id] = it.display_name
    except text_format.ParseError as e:
        raise IOError("Cannot parse file %s: %s." % (filename, str(e)))
    return class_names


def generate_class_colors(class_names):
    class_colors = {}
    for k,v in class_names.items():
        class_colors[k] = (random.randint(100,255),random.randint(200,255),random.randint(100,255)) 
    return class_colors


class DrawBoundingBoxesLayer(VideoProcessingLayer):
    def __init__(self, class_names_filename):
        VideoProcessingLayer.__init__(self)
        self.class_names = load_class_names_from_prototxt(class_names_filename)
        self.class_colors = generate_class_colors(self.class_names)
            
    def setup(self, ctx):
        pass
    
    def process(self, ctx):        
        output_dict = ctx["OBJECT_DETECTION_OUTPUT"]   
        output_frame = ctx["OUTPUT_FRAME"]
        h,w = ctx["OUTPUT_FRAME"].shape[0], ctx["OUTPUT_FRAME"].shape[1]      
        max_detections = output_dict['classes'].shape[0]
    
        labeled_output_scores = {}
        labeled_output_boxes = {}
        labeled_output_es = {}
        

        for i in range(max_detections):
            score = output_dict['detection_scores'][i]
            if score >= 0.5:                
                class_index = output_dict['classes'][i]
                box = output_dict['detection_boxes'][i]        
                x0 = int(box[1] * w)
                y0 = int(box[0] * h)        
                x1 = int(box[3] * w)
                y1 = int(box[2] * h)        
                cv2.rectangle( output_frame, (x0,y0), (x1,y1), self.class_colors[class_index], 1 )
                text = "{}: {:.4f}".format(self.class_names[class_index], score)
                cv2.putText( output_frame, 
                             text, (x0, y0 - 5), 
                             cv2.FONT_HERSHEY_SIMPLEX, 
                             0.3, 
                             self.class_colors[class_index], 
                             1,
                             lineType=cv2.LINE_AA)

                # Scores
                if self.class_names[class_index] not in labeled_output_scores:
                    labeled_output_scores[self.class_names[class_index]] = []
                labeled_output_scores[self.class_names[class_index]].append(score)
                ctx["OBJECT_DETECTION_LABELED_OUTPUT_SCORES"] = labeled_output_scores
                
                # Boxes
                if self.class_names[class_index] not in labeled_output_boxes:
                    labeled_output_boxes[self.class_names[class_index]] = []
                labeled_output_boxes[self.class_names[class_index]].append((x0,y0,x1,y1))
                ctx["OBJECT_DETECTION_LABELED_OUTPUT_BOXES"] = labeled_output_boxes

                # ES
                if self.class_names[class_index] not in labeled_output_es:
                    labeled_output_es[self.class_names[class_index]] = []
                new_detection = {
                        "score": score,
                        "box": {
                            "x0": x0,
                            "y0": y0,
                            "x1": x1,
                            "y1": y1
                        }
                }
                labeled_output_es[self.class_names[class_index]].append(new_detection)                
                ctx["OBJECT_DETECTION_LABELED_OUTPUT_ES"] = labeled_output_es
        
    def release(self, ctx):
        pass        