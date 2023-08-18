# Point Clouds: 3D Perception

## Problem Statement

## Abstract

## Plan of Action

1. [Understanding LiDARs](#ul)
2. [Point Cloud Processing](#pcl)
3. [Downsampling with Voxel Grid](#dvg)
4. [Segmentation with RANSAC](#swr)
5. [Clustering with DBSCAN](#cd)
6. [3D Bounding Box with PCA](#3bb)
7. [Surface Reconstruction](#sr)


----------------
<a name="ul"></a>
## 1. Understanding LiDARs


### 1.1 What is a LiDAR?
LiDAR stands for ```Light Detection And Ranging```. It is a **active remote sensing** technology that uses laser pulses to measure **distances** and create precise **3D representations** of objects and environments. Note that an active system generates its **own energy**, in this case, light, to measure objects or features on the ground. By **emitting** laser beams and measuring their **reflections**, LiDAR captures detailed spatial data, making it valuable in various fields like mapping, environmental monitoring, forestry, and ```autonomous vehicles```. 

### 1.2 How LiDAR works?
LiDAR technology operates by emitting short laser **pulses** toward the target area. These pulses hit surrounding objects and return to the LiDAR sensor. By calculating the time it takes for the pulses to return (**time of flight**), the system determines the **distance** to each object with incredible accuracy. Additionally, LiDAR can measure the **intensity** of the returned laser light, providing information about the objects' **reflectivity** or **composition**.


<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/16ef46d1-7bc3-4912-a923-6790c8b1d1b5.png"/>
</p>

Note that we divide by ```2``` in the equation because we only want  


- **Waveform**: Distribution of energy that returns to the sensor 
- **Intensity**: Amount of energy that returned to the sensor
- **Peaks**: Areas where more photons or more light energy returns to the sensor



### 1.3 Types of LiDAR

#### 1.3.1 Scanning

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/639ae999-2fdf-45ec-b4d7-f186527053c8" width="50%" />
</p>

**1. Mechanical LiDAR**: Employ moving parts, typically ```rotating mirrors``` or ```oscillating prisms```, to steer the laser beam and scan the surroundings, achieving a ```360-degree``` field of view.

**2. Solid-State LiDAR**: Use ```semiconductor``` components to generate and manipulate laser beams, eliminating the need for moving parts. This design enhances reliability and reduces ```size``` and ```weight```.

**- MEMs**: ```Micro-Electro-Mechanical Systems``` (MEMS) LiDAR leverages tiny, ```microfabricated mirrors``` to direct laser beams. MEMS LiDARs offer high-speed scanning capabilities and compact form factors.

**- OPA**: ```Optical Phased Array``` (OPA) LiDAR utilizes electronically controlled ```optical elements``` to steer laser beams without moving parts. OPA LiDARs provide rapid beam steering, allowing for real-time data acquisition and high-resolution mapping.


**3. Flash LiDAR**: Also known as ```single-shot``` or ```non-scanning``` LiDAR, illuminates the entire field of view ```simultaneously``` with a single pulse of laser light. This type of LiDAR offers high data acquisition rates, making it suitable for real-time applications.



<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/4cf1c007-dace-48fb-b532-56392e335cad" width="50%" />
</p>
<div align="center">
    <p>Image Source: <a href="https://www.semanticscholar.org/paper/Lidar-for-Autonomous-Driving%3A-The-Principles%2C-and-Li-Iba%C3%B1ez-Guzm%C3%A1n/586e90ae16e480217709e735f5aa0752aedc8e62">Lidar for Autonomous Driving: The Principles, Challenges, and Trends for Automotive Lidar and Perception Systems</a></p>
</div>




<div style="text-align: center;">
  <video src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/9bd0ccf2-fb23-4599-8b65-51f7440d8444" controls="controls" style="max-width: 730px;">
  </video>
</div>
<div align="center">
    <p>Video source: <a href="https://www.youtube.com/watch?v=fFEmWubsE8U&ab_channel=Ouster">How Ouster Digital Lidar works</a></p>
</div>




#### 1.3.2 Dimension

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/f136dc12-e71b-4636-9e7e-719eae12d538" width="50%" />
</p>


**1. 2D**: Measures distances in ```2D space```, typically along a ```single plane```. 2D LiDAR provides information about the ```distances``` and ```angles``` of objects in that plane but does not capture information about objects in the ```vertical``` direction.

**2. 3D**: Measures distances in ```3D space```, providing information about objects in both the ```horizontal``` and ```vertical``` directions. Each point in the point cloud contains the X, Y, and Z coordinates. By calculating the ```distance``` and ```angle``` of the laser beams, 3D LiDAR generates a ```3D point cloud``` that represents the surveyed scene.

**3. 4D**: In addition to measuring distances in 3D space (X, Y, and Z), 4D LiDAR also captures the ```time-of-flight``` for each laser pulse. This additional temporal data allows us to measure the ```speed``` and ```movement``` of objects over time.



<div style="text-align: center;">
  <video src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/e90b1a31-c6bb-4036-8338-0d389de7a5d0" controls="controls" style="max-width: 730px;">
  </video>
</div>
<div align="center">
    <p>Video source: <a href="https://www.aeva.com/">Aeva's Aeries II: The World’s First 4D LiDAR With Camera-Level Resolution</a></p>
</div>

#### 1.3.3 Modulation

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/d8dee0e4-9eb4-49c7-a49a-4b10f0345cab" width="30%" />
</p>

**1. Discrete**: Operates by emitting individual ```laser pulses``` to measure distances where each pulse illuminates a ```single point``` on the target surface. By calculating the ```time-of-flight```, the distance to the object can be determined.

**2. Continuous: Amplitude-Modulated Continuous Wave (AMCW)**: Emits a continuous laser beam with a ```modulated intensity```. The modulation allows the LiDAR system to differentiate between the emitted light and the reflected light from the target surface. By analyzing the ```modulation phase shift```, the system can calculate the ```distance``` to the object. 

**3. Continuous:  Frequency-Modulated Continuous Wave (FMCW)**: Emits a continuous laser beam with a ```varying frequency```. The ```frequency shift``` of the reflected light is used to determine the ```distance``` to the target. FMCW LiDAR is particularly useful for ```velocity``` measurements, as it can also capture the ```Doppler shift``` caused by moving objects. 

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/0528e774-45be-4461-854e-cb88c2d68d5b" width="60%" />
</p>




<div style="text-align: center;">
  <video src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/4227871f-d92c-4a03-adeb-9fb07df650c6" controls="controls" style="max-width: 730px;">
  </video>
</div>
<div align="center">
    <p>Video source: <a href="https://www.aeva.com/aeries-ii/">Aeva’s FMCW technology is resistant to interference from sunlight and other LiDAR sensors</a></p>
</div>


### 1.4 Other Parameters

#### 1.4.1 Wavelength
LiDARs operate using laser light to measure distances and create 3D representations of environments. The choice of the working wavelength, such as ```905 nm``` and ```1550 nm```, has implications for the system's performance, safety, and interaction with the environment. Here's an overview of these wavelengths as illustrated by Velodyne:

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/853f8b73-34f5-4bed-8bb2-4c2caa7a0709" width="70%" />
</p>
<div align="center">
    <p>Image source: <a href="https://velodynelidar.com/blog/guide-to-lidar-wavelengths/">A Guide to Lidar Wavelengths for Autonomous Vehicles and Driver Assistance</a></p>
</div>

Shorter wavelengths like ```905 nm``` are favored for high-precision applications where detailed and accurate measurements are crucial. Longer wavelengths like ```1550 nm``` are chosen for their potential to provide extended range and better performance in challenging environments while maintaining a higher level of eye safety.

#### 1.4.2 Resolution
The number of lasers in the vertical direction is called channels for the lidar with 360-degree spinning and 1D rotating mirror structure. In a general sense, a greater number of channels corresponds to an increased quantity of lasers, leading to an enhanced overall performance of LiDAR technology.

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/e96918e3-5cc8-4d14-874b-8dd1622989c0" width="70%" />
</p>
<div align="center">
    <p>Image source: <a href="https://www.hesaitech.com/things-you-need-to-know-about-lidar-the-more-lasers-the-better/">16 channels / 128 channels / 64 channels simulated point cloud</a></p>
</div>

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/128a26c2-8b08-4818-abbd-eda5a8ad449d9" width="70%" />
</p>
<div align="center">
    <p>Image source: <a href="https://velodynelidar.com/blog/guide-to-lidar-wavelengths/">A Guide to Lidar Wavelengths for Autonomous Vehicles and Driver Assistance</a></p>
</div>

#### 1.4.3 Wavelength


----------
<a name="pcl"></a>
## 2. Point Cloud Processing
In our analysis, we will employ the KITTI Dataset, which includes synchronous stereo images and LiDAR data. This dataset captures two pairs of images, taken at regular time intervals, using their stereo system. Additionally, their Velodyne LiDAR generates a corresponding point cloud. As a result, we can visualize the scene in 2D using the images and in 3D using the point cloud, as demonstrated below:

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

### 2.1 Visualization

In this project, there are several libraries and software available for point cloud processing. Among them, two commonly used libraries are **Open3D** and **PCL**. For this project, we will exclusively utilize **Open3D**. The reason for this choice is its user-friendly nature and the abundant literature available on it. Our point cloud has already been transformed into the ```.ply``` format, allowing us to employ the ```read_point_cloud function``` from Open3D as follows:

```python
point_cloud = open3d.io.read_point_cloud(point_cloud_path)
```

After reading the file, there are numerous ways to visualize the point cloud with Open3D:

```python
    o3d.visualization.draw_geometries([point_cloud])
```
Or:

```python
    vis = open3d.visualization.Visualizer()
    vis.create_window()
    vis.add_geometry(point_cloud)
    vis.get_render_option().background_color = np.asarray([0, 0, 0])
    vis.run()
    vis.destroy_window()
```
Or with ```get_plotly_fig``` from **plotly**. (However, I was having some error using ```get_plotly_fig```, hence I created my custom function). The function below allows you to encode either the **distance** with color or use the **reflectance** data from the point cloud itself. 

```python
    visualize_reflectance_distance(point_cloud, mode='distance', save=False, show=True) #mode = reflectance or distance
```

If we want to visualize the point cloud with the **distance** encoded with color:

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/35b51000-dbaf-4492-af3e-12de72ab6ed3" width="70%" />
</p>

Notice, that points further away are in the shades of yellow as shown with the color bar on the right. Now, if we want to visualize the point cloud with the **reflectance** encoded with color:

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/35e20921-d514-40fc-8c24-f550de8679b3" width="70%" />
</p>


Observe that the billboard on the right or the license plate of the car or even the lane lines have a higher reflectance and a darker shade of red. We will work more on this later.

### 2.2 Thresholding
Let's build a ```lane line detection``` using **reflectance** data from our point cloud. From the image above, when we look closely, we can see traces of the lane lines with darker shades of red. We will now filter the point cloud with higher reflectance values. The steps:

1. Get the reflectivity values
2. Create a mask of points that have reflectivity above the threshold
3. Filter points and reflectivities using the mask
4. Create a new point cloud with the filtered points and colors

```python
    # Thresholding
    filtered_point_cloud = reflectivity_threshold(point_cloud, threshold=0.5)
```

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/0525bebf-9bf0-45ea-a6ee-ffe4d7264135" width="70%" />
</p>

With the thresholding technique, we filtered out all point clouds having reflectivity values below ```0.5```. Observe, how the lane lines are more visible, but also the license plate and the guardrails on the side. We are interested only in the lane lines hence, we need to filter those out.


### 2.3 Region of Interest (ROI)
Now that we have the filtered reflectance values, we need to manually create an ROI that will encompass the lane lines. We need to set the **min** and **max** values for the ROI as such:

```python
    # Region of Interest
    roi_point_cloud = roi_filter(point_cloud, roi_min=(0, -3, -2), roi_max=(20, 3, 0))
```

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/743c49bb-0b0a-47a1-9a2d-0f5830270a03" width="70%" />
</p>

When we are satisfied with our selected ROI, we then need to create a **pipeline** using the ```reflectivity_threshold``` function and the ```roi_filter``` function. 

```python
    # Lane Line Detection
    def lane_line_detection(point_cloud):
        # Make a copy
        point_cloud_copy = copy.deepcopy(point_cloud)

        # Thresholding
        filtered_point_cloud = reflectivity_threshold(point_cloud_copy, threshold=0.45)
    
        # Region of Interest
        roi_point_cloud = roi_filter(filtered_point_cloud, roi_min=(0, -3, -2), roi_max=(20, 3, 0))
    
        return roi_point_cloud
```

Below is a video of the output:

https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/dcd64671-662e-4b8c-a5c9-adbd0d795dff

------------

<a name="dvg"></a>
## 3. Downsampling with Voxel Grid
Notice that when processing our point cloud, it is not necessary to have all the points. In fact, it can a better idea to downsample our point cloud to remove any potential noise, or for faster processing and visualization. The steps to Voxel Grid Downsampling are as follows:

1. **Define the Voxel Size**: Smaller voxel sizes will result in a more detailed point cloud, while larger voxel sizes will lead to more aggressive downsampling.
2. **Divide the Space**: The 3D space of the point cloud is divided into voxels by creating a 3D grid. Each point in the point cloud is assigned to the voxel it falls into based on its 3D coordinates.
3. **Subsampling**: For each voxel, one point is retained. This can be achieved by selecting the ```centroid``` of the points within the voxel or by using other sampling methods.
4. **Remove Redundant Points**: Points that fall into the same voxel are reduced to one point, effectively downsampling the point cloud.
   
<table>
  <tr>
    <th align="center">Voxel=0.1</th>
    <th align="center">Voxel=0.5</th>
    <th align="center">Voxel=1.0</th>
  </tr>
  <tr>
    <td><img width="802" alt="Image 1" src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/87648f00-cd30-4c54-b6c3-26d41b3327d1"></td>
    <td><img width="822" alt="Image 2" src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/d310bb6a-e4b5-42b9-a149-3949d993906f"></td>
    <td><img width="828" alt="Image 3" src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/838d0e14-8ba4-4678-bc47-5e2db5e5c343"></td>
  </tr>
  <tr>
    <td align="center">#points = 45803</td>
    <td align="center">#points = 13811</td>
    <td align="center">#points = 5922</td>
  </tr>
</table>

```python
    # Voxel Grid
    point_cloud_downsampled = point_cloud.voxel_down_sample(voxel_size=1)
```

The downsampling process results in a point cloud with a reduced number of points while still retaining its main features.

<p align="center">
  <img style="width: 80%; height: auto;" alt="Image 2" src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/95e1c621-2718-45e2-a7ec-1e2aff4c79fc">
</p>




----------
<a name="swr"></a>
## 4. Segmentation with RANSAC
RANSAC (```Random Sample Consensus```) was invented by Fischler and Bolles in ```1981``` as a solution to the problem of fitting models (such as lines, planes, circles, etc.) to noisy data with outliers. The algorithm is widely used in computer vision, image processing, and other fields where robust estimation of model parameters is required.

1. Randomly choose ```s``` samples which is the minimum number of samples to fit a model.
2. Fit the model to the randomly chosen samples.
3. Count the number ```M``` of datapoints which fit the model within a measure of error ```e```.
4. Repeat steps 1-3 ```N``` times.
5. Choose the model that has the highest number of ```M``` inliers.


In our scenario, we choose ```ransac_n``` equal to ```3```, which is the number of points to randomly sample for each iteration of RANSAC, and ```num_iterations``` equal to ```2000``` which is the number of RANSAC iterations to perform.


```python
    # Perform plane segmentation using RANSAC
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold, ransac_n=ransac_n, num_iterations=num_iterations)

    # Extract inlier and outlier point clouds
    inlier_cloud = point_cloud.select_by_index(inliers)
    outlier_cloud = point_cloud.select_by_index(inliers, invert=True)
```
Observe how the plane which is the road is segmented from all other vertical objects such as cars, trees, billboards, and so on. The inliers are the road segment and the outliers are the rest.

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/c3ce774b-e861-4de2-857b-f87e7fc30d38" width="70%" />
</p>

------------
<a name="cd"></a>
## 5. Clustering with DBSCAN
DBSCAN (Density-Based Spatial Clustering of Applications with Noise) is a density-based clustering algorithm proposed by Martin Ester, Hans-Peter Kriegel, Jörg Sander, and Xiaowei Xu in 1996. It is particularly suitable for datasets with complex structures and a need for automatic cluster detection. 

1. Choose parameters ```ε``` and ```MinPts``` that define the **neighborhood** of a data point and the **minimum points** for forming core points, respectively.
2. Mark data points as **core points** if they have at least MinPts points within ε-neighborhood.
3. A point A is directly **density-reachable** from B if B is core and A is within its ε-neighborhood.
4. Density-reachability is **transitive**, forming density-based clusters by linking mutually density-reachable points.
5. ```Noise points``` are **outliers**, not core, and not density-reachable from any core point.

In our scenario, we will use an eps value of ```1.5``` which is the maximum distance between two points for one to be considered as in the neighborhood of the other, and a ```min_points``` value of ```50``` which signifies the minimum number of points required to form a dense region (core points).

```python
    labels = np.array(outlier_cloud.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
```

Notice that the road is one cluster and all the other obstacles are separate clusters represented in different colors.


<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/746bf267-188c-45ce-bd71-a2ab9f2cfa8f" width="70%" />
</p>



----------
<a name="3bb"></a>
## 6. 3D Bounding Box with PCA
After successfully clustering our obstacles, the next step is to enclose them within three-dimensional bounding boxes. Given that our operations are conducted in a three-dimensional space, the bounding boxes we create are also in 3D. However, it's essential to recognize that in addition to generating bounding boxes, we require information about their **orientation**. As some obstacles might be positioned at various angles relative to the ego vehicle, we aim to construct bounding boxes that precisely encompass each obstacle and nothing beyond that scope. We will use ```PCA``` for that:

1. Compute the **centroid** of the point cloud subset, which is the center of mass.

2. Calculate the **covariance matrix** for the subset, summarizing dimension relationships.

3. Apply **PCA** to the covariance matrix, finding **eigenvectors** and **eigenvalues**.

4. The eigenvector with the **largest** eigenvalue signifies the longest axis and primary **spread direction**.

5. The eigenvector with the **smallest** eigenvalue indicates the shortest axis and **compression direction**.

6. Use these eigenvectors to define the **orientation** (```rotation```) of the **Oriented Bounding Box** (```OBB```). It aligns the OBB with the eigenvectors, so the box represents the point cloud's **principal directions**.

7. OBB **size** along each axis reflects point **spread** along the corresponding eigenvector.

```python
    obs = []
    # Group points by cluster label
    indexes = pd.Series(range(len(labels))).groupby(labels, sort=False).apply(list).tolist()

    # Iterate over clusters and perform PCA
    for i in range(0, len(indexes)):
        nb_points = len(outlier_cloud.select_by_index(indexes[i]).points)
        if nb_points > MIN_POINTS and nb_points < MAX_POINTS:
            sub_cloud = outlier_cloud.select_by_index(indexes[i])
            obb = sub_cloud.get_oriented_bounding_box()
```

Note that the ```get_oriented_bounding_box``` function from Open3D performs the PCA function for us behind the scenes. We use the outlier point cloud subset as these correspond to the cars and other objects whereas the inliers represent the road.

<p align="center">
  <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/11dc95f4-7052-4ab1-9f04-98518ff023ae" width="70%" />
</p>

    
----------
    
<a name="sr"></a>
## 7. Surface Reconstruction
For the last part, we will see how we can do surface reconstruction given a point cloud. In order to do that, we first need to compute the ```normals``` of each point in a point cloud. Normals refer to the direction vectors that are **perpendicular** to the surfaces of the points. For each point in the point cloud, a normal vector is calculated to indicate the **direction** of the surface at that point. Normals are used to estimate the **orientation** of surfaces and to **reconstruct** smooth surfaces from a point cloud. For this scenario, I chose to get the point cloud of my jug of water using the LiDAR of an iPhone as shown below:



We can then visualize the point cloud and the normals with the code below:

```python
    point_cloud_downsampled.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=100, max_nn=30))
```

<table align="center">
  <tr>
    <td>
      <p align="center">
        <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/702d47ef-15c5-4b31-98ac-6e8b4be95851" width="100%" />
      </p>
    </td>
    <td>
      <p align="center">
        <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/b567d2f5-4891-4c5b-90ee-ddcc286bd2ea" width="100%" />
      </p>
    </td>
  </tr>
</table>

There are several methods that we can do surface reconstruction such as:

#### 1. Alpha Shape
- Constructs a simplicial complex from a set of points in 2D or 3D space.
- The complex includes vertices, edges, and faces, which together form a piecewise linear approximation of the point cloud.
- The parameter "alpha" determines which points are considered part of the shape and which are considered outliers. 

#### 2. Poisson Reconstruction
- Used to create a smooth and continuous surface from an unorganized point cloud.
- Based on solving a partial differential equation (PDE) over the volumetric representation of the point cloud.
- It is particularly effective for capturing fine geometric details in the reconstructed surface.

#### 3. Ball Pivoting
- Works by "pivoting" a ball of a certain radius around each point in the point cloud.
- Constructs triangles by connecting the centers of the balls when three balls are in contact.
- It iteratively adds triangles to the mesh until a closed surface is formed.
- It is suitable for reconstructing complex and irregular shapes, and it often produces watertight surfaces.

Below is an example using Alpha Shape (left) and using Mesh Lab (right):


<table align="center">
  <tr>
    <td>
      <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/2b8bd144-8393-4742-984d-753c783b44aa" alt="Image 1" style="height: 300px;"></td>
    <td>
      <img src="https://github.com/yudhisteer/Point-Clouds-3D-Perception/assets/59663734/aa73b8b4-2cfc-4a71-8e3d-b20b4e684802" alt="Image 1" style="height: 300px;"></td>
   </tr>
</table>



-----------------



## References:
[1] Generation Robots. (n.d.). What is LiDAR Technology? Scanning technology. Webpage. [https://www.generationrobots.com/blog/en/what-is-lidar-technology/](https://www.generationrobots.com/blog/en/what-is-lidar-technology/)

[2] Velodyne LiDAR. (n.d.). A Guide to LiDAR Wavelengths. Webpage. [https://velodynelidar.com/blog/guide-to-lidar-wavelengths/](https://velodynelidar.com/blog/guide-to-lidar-wavelengths/)

[3] NEON Science. (n.d.). LiDAR Basics. Webpage. [https://www.neonscience.org/resources/learning-hub/tutorials/lidar-basics](https://www.neonscience.org/resources/learning-hub/tutorials/lidar-basics)

[4] NEON Science. (n.d.). Plasio: View Point Clouds. Webpage. [https://www.neonscience.org/resources/learning-hub/tutorials/plasio-view-pointclouds](https://www.neonscience.org/resources/learning-hub/tutorials/plasio-view-pointclouds)

[5] Towards Data Science. (n.d.). Random Sample Consensus Helps You Filter Those Pesky Outliers. Webpage. [https://towardsdatascience.com/random-sample-consensus-helps-you-filter-those-pesky-outliers-9dbfbb8b668f](https://towardsdatascience.com/random-sample-consensus-helps-you-filter-those-pesky-outliers-9dbfbb8b668f)

[6] First Principles of Computer Vision. (n.d.). LiDAR Sensor - Light Detection and Ranging. YouTube video. [https://www.youtube.com/watch?v=EkYXjmiolBg&ab_channel=FirstPrinciplesofComputerVision](https://www.youtube.com/watch?v=EkYXjmiolBg&ab_channel=FirstPrinciplesofComputerVision)

[7] https://www.aeva.com/aeries-ii/

