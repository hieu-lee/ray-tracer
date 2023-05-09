#include "utils.cpp"

class Intersection
{
    bool theIsIntersected = false;
    bool theIsReflective = false;
    bool theIsLight = false;
    double theRefractIndex = 1.;
    double theDistance;
    Vector theSphereCenter;
    Vector thePosition;
    Vector theNormal;
    Vector theAlbedo;

public:
    Intersection() {}

    Intersection(
        double aDistance,
        Vector aSphereCenter,
        Vector aPosition,
        Vector aNormal,
        Vector aAlbedo,
        bool aIsIntersected = true,
        bool aIsReflective = true,
        bool aIsLight = false,
        double aRefractIndex = 1.)
        : theIsIntersected(aIsIntersected),
          theIsReflective(aIsReflective),
          theIsLight(aIsLight),
          theRefractIndex(aRefractIndex),
          theDistance(aDistance),
          theSphereCenter(aSphereCenter),
          thePosition(aPosition),
          theNormal(aNormal),
          theAlbedo(aAlbedo)
    {
    }

    bool isIntersected() const
    {
        return theIsIntersected;
    }

    bool isReflective() const
    {
        return theIsReflective;
    }

    bool isLight() const
    {
        return theIsLight;
    }

    double getRefractIndex() const
    {
        return theRefractIndex;
    }

    double getDistance() const
    {
        return theDistance;
    }

    Vector getSphereCenter() const
    {
        return theSphereCenter;
    }

    Vector getPosition() const
    {
        return thePosition;
    }

    Vector getNormal() const
    {
        return theNormal;
    }

    Vector getAlbedo() const
    {
        return theAlbedo;
    }
};

class Ray
{
    Vector theOrigin;
    Vector theDirection;

public:
    Ray(Vector aOrigin, Vector aDirection) : theOrigin(aOrigin), theDirection(aDirection) {}

    Vector getOrigin() const
    {
        return theOrigin;
    }

    Vector getDirection() const
    {
        return theDirection;
    }
};

class BoundingBox
{
    Vector theBmin;
    Vector theBmax;

public:
    BoundingBox() {}
    BoundingBox(Vector aBmin, Vector aBmax) : theBmin(aBmin), theBmax(aBmax) {}

    Vector getBmin() const
    {
        return theBmin;
    }

    Vector getBmax() const
    {
        return theBmax;
    }

    Vector getDiagonal() const
    {
        return theBmax - theBmin;
    }

    Vector getMiddleDiagonal() const
    {
        return (theBmax + theBmin) / 2.;
    }

    bool intersect(const Ray &aRay, double &t) const
    {
        double tx1, ty1, tz1;
        double tx0, ty0, tz0;
        double tBmin, tBmax;
        Vector N;

        Vector BminO = theBmin - aRay.getOrigin();
        Vector BmaxO = theBmax - aRay.getOrigin();
        Vector u = aRay.getDirection();

        N = Vector(1, 0, 0);
        tBmin = dot(BminO, N) / dot(u, N);
        tBmax = dot(BmaxO, N) / dot(u, N);
        tx0 = std::min(tBmin, tBmax);
        tx1 = std::max(tBmin, tBmax);

        N = Vector(0, 1, 0);
        tBmin = dot(BminO, N) / dot(u, N);
        tBmax = dot(BmaxO, N) / dot(u, N);
        ty0 = std::min(tBmin, tBmax);
        ty1 = std::max(tBmin, tBmax);

        N = Vector(0, 0, 1);
        tBmin = dot(BminO, N) / dot(u, N);
        tBmax = dot(BmaxO, N) / dot(u, N);
        tz0 = std::min(tBmin, tBmax);
        tz1 = std::max(tBmin, tBmax);

        double myFirstIntersectionT = std::max({tx0, ty0, tz0});
        if (std::min({tx1, ty1, tz1}) > myFirstIntersectionT > 0)
        {
            t = myFirstIntersectionT;
            return true;
        }
        return false;
    }
};

class Geometry
{
protected:
    bool theIsReflective;
    Vector theAlbedo;

public:
    virtual Intersection intersect(const Ray &aRay) const = 0;
    bool isReflective() const
    {
        return theIsReflective;
    }

    Vector getAlbedo() const
    {
        return theAlbedo;
    }
};
