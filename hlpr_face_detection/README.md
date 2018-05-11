# Install dlib from source (without GPU support/CUDA)
````
$ cd ~/
$ git clone https://github.com/davisking/dlib.git
$ cd dlib
$ mkdir build; cd build; cmake .. -DDLIB_USE_CUDA=0; cmake --build .
$ cd ..
$ sudo python setup.py install --no USE_AVX_INSTRUCTIONS --no DLIB_USE_CUDA
````

# Install dlib from source (with GPU support/CUDA)
````
$ cd ~/
$ git clone https://github.com/davisking/dlib.git
$ cd dlib
$ mkdir build; cd build; cmake .. -DDLIB_USE_CUDA=1; cmake --build .
$ cd ..
$ sudo python setup.py install --no USE_AVX_INSTRUCTIONS
````

# Install the face recognition package
````
sudo pip install --upgrade scipy
sudo pip install face_recognition
sudo pip install git+https://github.com/ageitgey/face_recognition_models
````

Check <https://github.com/ageitgey/face_recognition#face-recognition> for more details on the algorithm.

# Test the face recognition package
````
Without GPU: roslaunch hlpr_face_detection face_detector.launch 
With GPU (more accurate version): roslaunch hlpr_face_detection face_detector_gpu.launch
````


