#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
import pyaudio as pya
from std_msgs.msg import Float64MultiArray
import numpy as np
from scipy import stats
import struct
import threading
import argparse

VERBOSE=False

class AudioFeature():
    def __init__(self, print_only):
        self._pa = pya.PyAudio()
        d_info=None
        d_index=None

        self._channels = 2
        #self._rate = 48000
        self._rate = 16000
        fmt = pya.paInt16
        self._timestep = 0.01
        self._data = {}
        self._data["data"]=[]

        found = False

        for i in range(self._pa.get_device_count()):
            d_info = self._pa.get_device_info_by_index(i)
            name = d_info["name"]
            channels = d_info["maxInputChannels"]
            if channels >0:
                print i, ":", d_info["name"], channels
            if "miniDSP" in name:
                print d_info
                self._rate = int(d_info["defaultSampleRate"])
                found = True
                d_index=i
                break
            
        if not found or print_only:
            exit(-1)

        self._frames_per_block = int(self._timestep*self._rate)

        
        self._stream = self._pa.open(format=fmt, channels=self._channels, rate=self._rate, input=True, frames_per_buffer=self._frames_per_block, input_device_index=d_index)
        self._intensity_pub = rospy.Publisher("/features/audio/intensity",Float64MultiArray, queue_size=1)
        self._flatness_pub = rospy.Publisher("/features/audio/flatness",Float64MultiArray, queue_size=1)
        self._band_pub = rospy.Publisher("/features/audio/bands", Float64MultiArray, queue_size=1)


        self._listen_thread=threading.Thread(target=self.listen)
        self._listen_thread.start()
        
    def get_features(self,data):
        data = np.reshape(data, [self._channels,-1],"F")
        #print data
        band_features = [[] for i in range(5)]
        flatness_features = []
        amp_features = []
        for k in range(self._channels):
            fft = np.fft.fft(data[k])
            spectrum = abs(fft)**2
            freqs = abs(np.fft.fftfreq(len(data[k]),d=1.0/self._rate))
            
            bands = [0,300,1200,2100,3000,24000]
        
            band_energies = [0 for b in range(len(bands)-1)]
            total_energy = sum(spectrum)

            for j in range(len(freqs)):
                for i in range(len(bands)-1):
                    if freqs[j]>=bands[i] and freqs[j]<bands[i+1]:
                        band_energies[i]+=(spectrum[j]/total_energy)
        

            flatness = stats.mstats.gmean(spectrum)/np.mean(spectrum)
            amplitude = np.sqrt(np.mean(np.square(data[k])))
            amp_features.append(amplitude)
            flatness_features.append(flatness)

            for b in range(len(band_energies)):
                band_features[b].append(band_energies[b])
                
        bands_out = []
        for b in band_features:
            bands_out+=b

        return (Float64MultiArray(data=amp_features),Float64MultiArray(data=flatness_features),Float64MultiArray(data=bands_out))

    def analyze(self):
        while not rospy.is_shutdown():
            if len(self._data["data"])==0:
                rospy.logwarn("Waiting for audio initialization...")
                rospy.sleep(0.2)
                continue

            read_fmt="{}h".format(self._frames_per_block*self._channels)
            data=struct.unpack(read_fmt,self._data["data"])
            intensity,flatness,bands = self.get_features(data)
            self._intensity_pub.publish(intensity)
            self._flatness_pub.publish(flatness)
            self._band_pub.publish(bands)
            

    def listen(self):
        while not rospy.is_shutdown():
            try:
                self._data["data"]=self._stream.read(self._frames_per_block)
            except IOError as e:
                if VERBOSE:
                    rospy.logwarn("Error in audio feature node: {}".format(e))
            read_fmt="{}h".format(self._frames_per_block*self._channels)
            data=struct.unpack(read_fmt,self._data["data"])
            intensity,flatness,bands = self.get_features(data)
            self._intensity_pub.publish(intensity)
            self._flatness_pub.publish(flatness)
            self._band_pub.publish(bands)

if __name__=="__main__":
    rospy.init_node("audio_feature_publisher")
    
    p = argparse.ArgumentParser(description='Extract useful audio features from multi-channel audio data.')

    p.add_argument('-l','--list', action='store_true',help="List available audio capture devices and return.")

    args = p.parse_known_args()

    print args
    a = AudioFeature(args[0].list)
    a.analyze()
