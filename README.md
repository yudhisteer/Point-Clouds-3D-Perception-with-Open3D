# Point Clouds: 3D Perception

## Problem Statement

## Abstract

## Plan of Action

1. [Understanding LiDARs](#ul)
2. [Point Cloud Processing](#pcl)
3. [Downsampling with Voxel Grid](#dvg)
4. [Segmentation with RANSAC](#sr)
5. [Clustering with DBSCAN](#cd)
6. [3D Bounding Box with PCA](#3bb)

----------------
<a name="ul"></a>
## 1. Understanding LiDARs


### 1.1 What is a LiDAR?
LiDAR stands for ```Light Detection And Ranging```. It is a **active remote sensing** technology that uses laser pulses to measure **distances** and create precise **3D representations** of objects and environments. Note that an active system generates its **own energy**, in this case, light, to measure objects or features on the ground. By **emitting** laser beams and measuring their **reflections**, LiDAR captures detailed spatial data, making it valuable in various fields like mapping, environmental monitoring, forestry, and ```autonomous vehicles```. 

### 1.2 How LiDAR works?
LiDAR technology operates by emitting short laser **pulses** toward the target area. These pulses hit surrounding objects and return to the LiDAR sensor. By calculating the time it takes for the pulses to return (**time of flight**), the system determines the **distance** to each object with incredible accuracy. Additionally, LiDAR can measure the **intensity** of the returned laser light, providing information about the objects' **reflectivity** or **composition**.


<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/16ef46d1-7bc3-4912-a923-6790c8b1d1b5.png" alt="CodeCogsEqn" />
</p>


- **Waveform**: Distribution of energy that returns to the sensor 
- **Intensity**: Amount of energy that returned to the sensor
- **Peaks**: Areas where more photons or more light energy returns to the sensor

### 1.3 Types of LiDAR

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/1e2c8522-750e-43e6-a0c2-6830e0ba7d31" width="50%" />
</p>


### 1.4 Types of LiDAR Data



----------
<a name="pcl"></a>
## 2. Point Cloud Processing


<table>
  <tr>
    <td>
      <div align="center">
        <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/c02f7218-135e-4501-aa76-5a1a1980f0ce" alt="Image 1" style="height: 200px;">
      </div>
    </td>
    <td>
      <div align="center">
        <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/dc6979bb-ffa6-4095-ab6c-9e23f539e298" alt="Image 2" style="height: 200px;">
      </div>
    </td>
  </tr>
</table>

------------

<a name="dvg"></a>
## 2. Downsampling with Voxel Grid


----------
<a name="sr"></a>
## 3. Segmentation with RANSAC


------------
<a name="cd"></a>
## 4. Clustering with DBSCAN


----------
<a name="3bb"></a>
## 5. 3D Bounding Box with PCA


----------




<div align="center">
    <p>Image Source: <a href="https://www.researchgate.net/figure/Illustration-of-several-grouping-principles-Adapted-from-Perceptual-Organization-in_fig1_230587594">A Century of Gestalt Psychology in Visual Perception</a></p>
</div>






## References:
1. https://www.generationrobots.com/blog/en/what-is-lidar-technology/#:~:text=Scanning%20technology&text=Scanning%20LiDAR%20typically%20spin%20and,frequency%20between%201Hz%20and%20100Hz.
2. https://velodynelidar.com/blog/guide-to-lidar-wavelengths/
3. https://www.neonscience.org/resources/learning-hub/tutorials/lidar-basics
4. https://www.neonscience.org/resources/learning-hub/tutorials/plasio-view-pointclouds
5. 
