#include <stdio.h>
#include <algorithm>
#include <list>
#include <math.h>

#include "vector.cpp"

static inline double square(double x)
{
    return x * x;
}

void boxMuller(double stdev, double &x, double &y)
{
    double r1 = ((double)rand() / (RAND_MAX));
    double r2 = ((double)rand() / (RAND_MAX));
    x = sqrt(-2 * log(r1)) * cos(2 * M_PI * r2) * stdev;
    y = sqrt(-2 * log(r1)) * sin(2 * M_PI * r2) * stdev;
}

Vector randomCos(const Vector &N)
{
    double r1 = ((double)rand() / (RAND_MAX));
    double r2 = ((double)rand() / (RAND_MAX));
    double x = cos(2 * M_PI * r1) * sqrt(1 - r2);
    double y = sin(2 * M_PI * r1) * sqrt(1 - r2);
    double z = sqrt(r2);

    int iMin = 0;
    double min = abs(N[0]);
    for (int i = 1; i < 3; i++)
    {
        if (abs(N[i]) < min)
        {
            min = abs(N[i]);
            iMin = i;
        }
    }

    Vector T1;
    if (iMin == 0)
        T1 = Vector(0, N[2], -N[1]);
    else if (iMin == 1)
        T1 = Vector(N[2], 0, -N[0]);
    else if (iMin == 2)
        T1 = Vector(N[1], -N[0], 0);
    T1 = normalize(T1);

    Vector T2 = cross(T1, N);

    return T1 * x + T2 * y + N * z;
}