# Pathtracer-with-Ray-Triangle-Intersection
This project is based on Kevin Beason's SmallPT pathtracer. It extends the original project by adding:
-Möller-Trumbore ray-triangle intersection algorithm,
-.OFF file loader,
-Motion Blur on Global Shutter
-A rolling shutter

In order to compile the project -> "g++ -O3 -fopenmp cgrpt1.cpp -o cgrpt"
In order to execute the program ->  e.g.: ./cgrpt 500 800 (pass additional "-withMB" argument for motion blur or pass "-withRSMB" for rolling shutter) 

  After the rendering is done, a .PPM file will be generated. The generated file could be opened via Gimp, Photoshop, LibreOffice etc. However,
the easy way to inspect the result via PPM Viewer extension in Visual Studio Code.

Result without motion blur.(1200x800, 10K Samples)
![fe](https://user-images.githubusercontent.com/43638551/144505443-63bf9e9a-e7a5-407f-9385-9b305d8d0c76.png)


Result with motion blur in 10K Sample.

![10KMBObjects](https://user-images.githubusercontent.com/43638551/144202495-f889eda3-4469-4549-ad28-f59f8dd05ecd.png)
