# Pathtracer-with-Ray-Triangle-Intersection
This project is based on Kevin Beason's SmallPT pathtracer. It extends the original project by adding:
-Möller-Trumbore ray-triangle intersection algorithm,
-.OFF file loader,
-Motion Blur in Global Shutter

In order to compile the project -> "g++ -O3 -fopenmp cgrpt1.cpp -o cgrpt"
In order to execute the program -> ./cgrpt <samplesPerPixel> <y-resolution>, e.g.: ./cgrpt 500 800 (pass additional "-withMB" argument for motion blur) 

Result without motion blur.
![Preview](https://user-images.githubusercontent.com/43638551/143770271-cc3ce686-9a96-4783-b38f-848bde2d8e8b.png)

Result with motion blur in 10K Sample.

![10KMBObjects](https://user-images.githubusercontent.com/43638551/144202495-f889eda3-4469-4549-ad28-f59f8dd05ecd.png)
