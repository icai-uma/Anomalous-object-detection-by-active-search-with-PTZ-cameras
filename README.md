# Anomalous-object-detection-by-active-search-with-PTZ-cameras

## Abstract
Due to the large amount of visual information generated daily, proposals that automatically analyze and process data are becoming increasingly necessary. This work focuses on the detection of anomalous objects in video sequences captured by PTZ (pan-tilt-zoom) cameras, considering as anomalies the objects which belong to categories that should not appear in a specific scene (e.g. pedestrians on a highway). There is a lack in the previous literature of a principled approach for the control of PTZ cameras that takes advantage of the recent developments in deep learning-based object detection. Our proposal aims to fill this gap by offering a probabilistic framework where the guidance of PTZ cameras is accommodated. The proposed methodology involves three different modules. An object detection stage, where deep learning networks are used to detect the objects that appear in the scene; an anomalous detection module, where a mixture of Dirichlet distributions is considered to detect automatically, and without supervised training, those detected objects which are likely to be anomalous; and finally, a PTZ camera controller which allows to follow and focus on the object considered as the most probably anomalous in the scene. The experimental results show the performance and viability of our proposal, which outperforms several competitors from qualitative and quantitative points of view. 

## Repository

This repository contains the source code of the paper Anomalous object detection by active search with PTZ cameras. The code is developed in Matlab and C++, and it uses Caffe (Faster-RCNN) and the virtualptz library.

The dataset with the anomalous objects has been created from those that the virtualptz library incorporates. They have been edited by using a video editing application and then adding the groundtruth by employing the PTZ gtmaker application from the LITIV project.

https://bitbucket.org/pierre_luc_st_charles/virtualptz_standalone

https://github.com/plstcharles/litiv

## Citation

Please, cite this work as:
López-Rubio, E., Molina-Cabello, M. A., Castro, F. M., Luque-Baena, R. M., Marín-Jiménez, M. J., & Guil, N. (2021). Anomalous object detection by active search with PTZ cameras. Expert Systems with Applications, 181, 115150. 
DOI: 10.1016/j.eswa.2021.115150
https://www.sciencedirect.com/science/article/abs/pii/S0957417421005911
