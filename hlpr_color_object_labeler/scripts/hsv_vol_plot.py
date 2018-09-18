#!/usr/bin/env python

from os import  walk
from os.path import join
from colorsys import rgb_to_hsv
import matplotlib.pyplot as plt
import numpy as np
import argparse
import rosbag

OR = lambda z: reduce(lambda x, y: x or y, z) if len(z) > 0 else True

def get_files_recursive(path, type=None, full=True):
    prepend_path = lambda path, files: [join(path, f) for f in files]   

    if full: 
        files = reduce(lambda x, y: x + y, [prepend_path(r, fs) for r, _, fs in walk(path)])
    else:
        files = reduce(lambda x, y: x + y, [f for r, _, fs in walk(path)])

    if not type is None:
        files = filter(lambda f: f.endswith('.'+type), files)

return files

def main():
    parser = argparse.ArgumentParser(description='Show hue/volume plot for task')
    parser.add_argument('--src', metavar='DIR', required=True, help='Path to directory containing bag files')

    args = parser.parse_args()
    src = args.src

    hue = []
    sat = []
    val = []
    vol = []
    
    bag_files = get_files_recursive(src, 'bag')
    n = len(bag_files)
    
    for i, bag_file in enumerate(bag_files):
        print 'Processing bag file ' + str(i+1) + ' of ' + str(n) + '...'
        with rosbag.Bag(bag_file) as bag:
            for _, msg, _, in bag.read_messages(topics=['/beliefs/features']):
                for obj in msg.objects:
                    h, s, v = rgb_to_hsv(obj.basicInfo.rgba_color.r,
                                         obj.basicInfo.rgba_color.g,
                                         obj.basicInfo.rgba_color.b)
                    hue.append(h)
                    sat.append(s)
                    val.append(v)
                    vol.append(obj.obb.bb_dims.x*obj.obb.bb_dims.y*obj.obb.bb_dims.z)

    fig, ax = plt.subplots(2, 3, sharex='col', sharey='row')

    ax[0,0].plot(hue, vol, 'r.')
    ax[0,0].set_ylabel('Volume')

    ax[1,0].hist(hue, bins=25, color='r')
    ax[1,0].set_xlabel('Hue')
    ax[1,0].set_ylabel('Counts')

    ax[0,1].plot(sat, vol, 'g.')

    ax[1,1].hist(sat, bins=25, color='g')
    ax[1,1].set_xlabel('Saturation')
    
    ax[0,2].plot(val, vol, 'b.')

    ax[1,2].hist(hue, bins=25, color='b')
    ax[1,2].set_xlabel('Value')

    plt.show()    

if __name__ == '__main__':
    main()

