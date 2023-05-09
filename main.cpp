#include <iostream>
#include <chrono>

#include "scene.cpp"
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

int main()
{
    const int W = 512;
    const int H = 512;
    const int RAY_PER_PIXEL = 1000;
    std::vector<unsigned char> myImage(W * H * 3, 0);
    const double FOV = 1.0472;
    const Vector myCameraPosition = Vector(0, 0, 55);
    const double MAX_RAY_DEPTH = 5;
    const double gamma = 2.2;
    const Vector myLightPosition(-10, 20, 40);
    const Vector mySphericalLightPosition(0, 20, 5);
    const double myLightRadius = 3.;
    const double myLightIntensity = 1e5;
    const double invertGamma = 1. / gamma;

    auto start = std::chrono::high_resolution_clock::now();

    Scene myScene = Scene(myLightPosition, myLightRadius, myLightIntensity);

    // // Spheres
    // Sphere mySpheres[] = {
    //     Sphere(Vector(-21, 0, 0), 10, Vector(0, 1, 1), true),
    //     Sphere(Vector(0, 0, 0), 10, Vector(1, 1, 1), false, 1.5),
    //     Sphere(Vector(21, 0, 0), 10, Vector(1, 0, 1), false, 1.5),
    //     Sphere(Vector(21, 0, 0), 9.5, Vector(1, 0, 1), false, 1. / 1.5),
    // };

    // for (Sphere &aSphere : mySpheres)
    // {
    //     myScene.addGeometry(&aSphere);
    // }

    // // Spherical Light Source
    // Sphere myLight = Sphere(mySphericalLightPosition, myLightRadius, Vector(1., 0., 0.), false, 1., false, true);
    // myScene.addGeometry(&myLight);

    // Cat
    TriangleMesh myCat = TriangleMesh(0.6, Vector(0, -10, 0), Vector(0.3, 0.2, 0.25), false);
    myCat.readOBJ("cat.obj");
    myScene.addGeometry(&myCat);

#pragma omp parallel for schedule(dynamic, 1)
    for (int i = 0; i < H; i++)
    {
        for (int j = 0; j < W; j++)
        {

            Vector color;
            double x, y;

            for (int k = 0; k < RAY_PER_PIXEL; k++)
            {
                boxMuller(0.5, x, y);
                Vector myVector;
                myVector[0] = (myCameraPosition[0] + (j + x) + 0.5 - W / 2);
                myVector[1] = (myCameraPosition[1] - (i + y) - 0.5 + H / 2);
                myVector[2] = myCameraPosition[2] - W / (2 * tan(FOV / 2));

                Ray myRay = Ray(myCameraPosition, normalize(myVector - myCameraPosition));
                color += myScene.getColor(myRay, MAX_RAY_DEPTH);
            }

            color /= RAY_PER_PIXEL;

            myImage[(i * W + j) * 3 + 0] = std::max(std::min(255., pow(color[0], invertGamma) * 255), 0.);
            myImage[(i * W + j) * 3 + 1] = std::max(std::min(255., pow(color[1], invertGamma) * 255), 0.);
            myImage[(i * W + j) * 3 + 2] = std::max(std::min(255., pow(color[2], invertGamma) * 255), 0.);
        }
    }

    stbi_write_png("ray_mesh_intersection_1000rpp.png", W, H, 3, &myImage[0], 0);

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::high_resolution_clock::now() - start);
    std::cout << "Duration: " << duration.count() / 1000. << "s" << std::endl;

    return 0;
}