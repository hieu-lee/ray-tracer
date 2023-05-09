#include "mesh.cpp"

class Scene
{
private:
    std::vector<Geometry *> theGeometries;
    Vector theLightPosition;
    double theLightRadius;
    double theLightIntensity;
    Sphere *theBackground[6];

public:
    ~Scene()
    {
        for (int i = 0; i < 6; i++)
        {
            delete theBackground[i];
        }
    }

    explicit Scene(
        const Vector &aLightPosition,
        double aLightRadius,
        double aLightIntensity)
    {
        Sphere *myBackground[6] = {
            new Sphere(Vector(0, 1000, 0), 940, Vector(0.93, 0.11, 0.14)),
            new Sphere(Vector(0, 0, -1000), 940, Vector(0.05, 0.58, 0.27)),
            new Sphere(Vector(0, -1000, 0), 990, Vector(0, 0.68, 0.94)),
            new Sphere(Vector(0, 0, 1000), 940, Vector(0.93, 0.01, 0.55)),
            new Sphere(Vector(1000, 0, 0), 940, Vector(1, 1, 0)),
            new Sphere(Vector(-1000, 0, 0), 940, Vector(0, 1, 1))};

        for (int i = 0; i < 6; i++)
        {
            theBackground[i] = myBackground[i];
            addGeometry(myBackground[i]);
        }

        theLightPosition = aLightPosition;
        theLightRadius = aLightRadius;
        theLightIntensity = aLightIntensity;
    }

    void addGeometry(Geometry *aGeometry)
    {
        theGeometries.push_back(aGeometry);
    }

    Intersection intersect(const Ray &aRay)
    {
        Intersection myIntersection = Intersection();
        double tMin = INFINITY;
        for (auto &aGeometry : theGeometries)
        {
            Intersection aIntersection = aGeometry->intersect(aRay);
            if (aIntersection.isIntersected() && aIntersection.getDistance() < tMin)
            {
                tMin = aIntersection.getDistance();
                myIntersection = aIntersection;
            }
        }
        return myIntersection;
    }

    Vector getColor(const Ray &aRay, int aRayDepth, bool aLastDiffuse = false)
    {
        if (aRayDepth < 0)
            return Vector(0., 0., 0.);
        Intersection myIntersection = intersect(aRay);
        Vector Lo(0., 0., 0.);

        if (myIntersection.isIntersected())
        {
            const double epsilon = 1e-5;
            Vector N = myIntersection.getNormal();
            Vector u = aRay.getDirection();
            Vector P = myIntersection.getPosition() + N * epsilon;

            if (myIntersection.isLight())
            {
                if (aLastDiffuse)
                    return Vector(0., 0., 0.);
                else
                    return Vector(1., 1., 1.) * theLightIntensity / square(2 * M_PI * theLightRadius);
            }

            double myDotValue = dot(N, u);

            if (myIntersection.isReflective())
            {
                Ray myReflectedRay = Ray(P, u - (2 * myDotValue) * N);
                return getColor(myReflectedRay, aRayDepth - 1);
            }
            else if (myIntersection.getRefractIndex() != 1.)
            {
                double n1, n2;
                if (myDotValue > 0)
                {
                    N = (-1.) * N;
                    myDotValue = dot(N, u);
                    n1 = myIntersection.getRefractIndex();
                    n2 = 1.;
                }
                else
                {
                    n1 = 1.;
                    n2 = myIntersection.getRefractIndex();
                }
                double k0 = square((n1 - n2) / (n1 + n2));
                P = myIntersection.getPosition() - N * epsilon;
                double d = 1. - square((n1 / n2)) * (1 - square(myDotValue));
                if (d > 0)
                {
                    Vector wT = (n1 / n2) * (u - myDotValue * N);
                    Vector wN = (-1.) * N * sqrt(d);
                    Vector w = wT + wN;
                    double x = ((double)rand() / (RAND_MAX));
                    if (x < k0 + (1 - k0) * pow(1 - abs(dot(N, w)), 5.))
                    {
                        Ray myReflectedRay = Ray(P, u - (2 * myDotValue) * N);
                        return getColor(myReflectedRay, aRayDepth - 1);
                    }
                    else
                    {
                        Ray myRefractedRay = Ray(P, w);
                        return getColor(myRefractedRay, aRayDepth - 1);
                    }
                }
                else
                {
                    Ray myInternalReflectedRay = Ray(P, u - (2 * myDotValue) * N);
                    return getColor(myInternalReflectedRay, aRayDepth - 1);
                }
            }
            else
            {
                double I = theLightIntensity;
                double R = theLightRadius;
                Vector x = myIntersection.getSphereCenter();

                Vector xprime = R * randomCos(normalize(x - theLightPosition)) + theLightPosition;
                Vector Nprime = normalize(xprime - theLightPosition);
                double d = norm(xprime - P);
                Vector omega = normalize(xprime - P);
                Ray myLightRay = Ray(P, omega);
                Intersection lightIntersection = intersect(myLightRay);
                double visibility = (lightIntersection.isIntersected() && lightIntersection.getDistance() <= d) ? 0. : 1.;
                double pdf = dot(Nprime, normalize(x - theLightPosition)) / (M_PI * R * R);
                Vector rho = myIntersection.getAlbedo();
                Lo = I / square(2 * M_PI * R) * rho / M_PI * visibility * std::max(dot(N, omega), 0.) * std::max(dot(Nprime, (-1.) * omega), 0.) / (square(norm(xprime - P)) * pdf);

                Ray myRandomRay = Ray(P, randomCos(N));
                Lo += rho * getColor(myRandomRay, aRayDepth - 1, true);
            }
        }

        return Lo;
    }
};