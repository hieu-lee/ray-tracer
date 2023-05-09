#include "objects.cpp"

class Sphere : public Geometry
{
    Vector theCenter;
    double theRadius;
    double theRefractIndex;
    bool theInvertNormal;
    bool theIsLightSource;

public:
    Sphere(
        const Vector &aCenter,
        const double &aRadius,
        const Vector &aAlbedo,
        bool aIsReflective = false,
        double aRefractIndex = 1.,
        bool aInvertNormal = false,
        bool aIsLightSource = false)
    {
        theCenter = aCenter;
        theRadius = aRadius;
        theAlbedo = aAlbedo;
        theIsReflective = aIsReflective;
        theRefractIndex = aRefractIndex;
        theInvertNormal = aInvertNormal;
        theIsLightSource = aIsLightSource;
    }

    Intersection intersect(const Ray &aRay) const override
    {
        Intersection myIntersection;
        bool myIsIntersected;
        double myDistance;
        Vector myPosition;
        Vector myNormal;

        Vector OC = aRay.getOrigin() - theCenter;
        double myDelta = square(dot(aRay.getDirection(), OC)) - (square(norm(OC)) - square(theRadius));
        myIsIntersected = myDelta >= 0;
        myDistance = 0.;
        if (myIsIntersected)
        {
            double myDotValue = dot(aRay.getDirection(), OC * (-1.));
            double myDeltaSqrt = sqrt(myDelta);
            double t1 = myDotValue - myDeltaSqrt;
            double t2 = myDotValue + myDeltaSqrt;
            if (t2 < 0)
                myIsIntersected = false;
            else
                myDistance = (t1 >= 0) ? t1 : t2;
        }
        myPosition = aRay.getOrigin() + aRay.getDirection() * myDistance;
        myNormal = normalize(myPosition - theCenter);
        return Intersection(
            myDistance,
            theCenter,
            myPosition,
            myNormal,
            theAlbedo,
            myIsIntersected,
            theIsReflective,
            theIsLightSource,
            theRefractIndex);
    }
};