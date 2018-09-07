#!/usr/bin/env python

from hlpr_perception_msgs.msg import ExtractedFeaturesArray
from colorsys import rgb_to_hsv, hsv_to_rgb
from yaml_include_loader.loader import *
import hsv_utils
import rospy
import yaml

def hue_filter(h_range):
    def f(msg, idxs=None):
        if idxs is None:
            idxs = range(len(msg.objects))

        keep_idxs = []

        for idx, obj in enumerate(msg.objects):
            if idx in idxs:
                h = obj.basicInfo.hue
                if hsv_utils.in_hue_range(h, h_range):
                    keep_idxs.append(idx)

        return keep_idxs
    return f 

def hsv_filter(h_range, s_range, v_range):
    def f(msg, idxs=None):
        if idxs is None:
            idxs = range(len(msg.objects))

        keep_idxs = []

        for idx, obj in enumerate(msg.objects):
            if idx in idxs:
                r = obj.basicInfo.rgba_color.r
                g = obj.basicInfo.rgba_color.g
                b = obj.basicInfo.rgba_color.b
                h, s, v = rgb_to_hsv(r, g, b)
                if hsv_utils.in_hsv_range(h, s, v, h_range, s_range, v_range):
                    keep_idxs.append(idx)

        return keep_idxs
    return f

def rgb_filter(r_range, g_range, b_range):
    def f(msg, idxs=None):
        if idxs is None:
            idxs = range(len(msg.objects))

        keep_idxs = []

        for idx, obj in enumerate(msg.objects):
            if idx in idxs:
                r = obj.basicInfo.rgba_color.r
                g = obj.basicInfo.rgba_color.g
                b = obj.basicInfo.rgba_color.b
                if hsv_utils.in_rgb_range(r, g, b, r_range, g_range, b_range):
                    keep_idxs.append(idx)

        return keep_idxs
    return f

def largest_filter(msg, idxs=None):
    if idxs is None:
        idxs = range(len(msg.objects))

    largest_idx = None
    largest_vol = -float('inf')

    for idx, obj in enumerate(msg.objects):
        if idx in idxs:
            vol = obj.obb.bb_dims.x*obj.obb.bb_dims.y*obj.obb.bb_dims.z
            if vol > largest_vol:
                largest_idx = idx
                largest_vol = vol

    if largest_idx is None:
        return []
    else:
        return [largest_idx] 

# ordered is only accepted kwarg
def compose_filters(*filters, **kwargs):
    ordered = kwargs.pop('ordered', True)

    def f(msg, idxs=None):
        if ordered:
            keep_idxs = filters[0](msg, idxs)
            for filt in filters[1:]:
                keep_idxs = filt(msg, keep_idxs)
        else:
            keep_idxs = list(reduce(lambda x, y: x.intersection(y), 
                                    [set(filt(msg, idxs)) for filt in filters], 
                                    []))
        return keep_idxs

    return f

def object_filter(params):
    h_range = (params['hue']['min'], params['hue']['max'])
    s_range = (params['saturation']['min'], params['saturation']['max'])
    v_range = (params['value']['min'], params['value']['max'])

    def f(msg):
        return compose_filters(hsv_filter(h_range, s_range, v_range), largest_filter)(msg)
 
    return f

def object_filter_from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        params = yaml.load(f, Loader=YAMLIncludelLoader)
    return object_filter(params)

def object_filters_from_yaml(yaml_file):
    with open(yaml_file, 'r') as f:
        objects = yaml.load(f, Loader=YAMLIncludeLoader)['objects']
    return {obj.keys()[0]: object_filter(obj[obj.keys()[0]]) for obj in objects} 

def load_tasks(yaml_file):
    with open(yaml_file, 'r') as f:
        tasks = yaml.load(f, Loader=YAMLIncludeLoader)['tasks']

    return {task.keys()[0]: {obj.keys()[0]: object_filter(obj[obj.keys()[0]]) for obj in task[task.keys()[0]]['objects']} for task in tasks}

