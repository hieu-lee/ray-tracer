#include <stdio.h>
#include <string.h>
#include "sphere.cpp"

class TriangleIndices
{
public:
    TriangleIndices(
        int vtxi = -1, int vtxj = -1, int vtxk = -1,
        int ni = -1, int nj = -1, int nk = -1,
        int uvi = -1, int uvj = -1, int uvk = -1,
        int group = -1) : vtxi(vtxi), vtxj(vtxj), vtxk(vtxk),
                          uvi(uvi), uvj(uvj), uvk(uvk),
                          ni(ni), nj(nj), nk(nk), group(group){};
    int vtxi, vtxj, vtxk; // indices within the aVertex coordinates array
    int uvi, uvj, uvk;    // indices within the uv coordinates array
    int ni, nj, nk;       // indices within the normals array
    int group;            // face group
};

class Node
{
    Node *theLeftChild = nullptr;
    Node *theRightChild = nullptr;
    BoundingBox theBoundingBox;
    int theStartTriangle;
    int theEndTriangle;

public:
    ~Node()
    {
        if (hasChildren())
        {
            delete theLeftChild;
            delete theRightChild;
        }
    }
    Node() {}

    void setBoundingBox(BoundingBox aBoundingBox)
    {
        theBoundingBox = aBoundingBox;
    }

    void setTriangles(int aStartTriangle, int aEndTriangle)
    {
        theStartTriangle = aStartTriangle;
        theEndTriangle = aEndTriangle;
    }

    void setChildren(Node *aLeftChild, Node *aRightChild)
    {
        theLeftChild = aLeftChild;
        theRightChild = aRightChild;
    }

    bool hasChildren() const
    {
        return theLeftChild != nullptr;
    }

    Node *getLeftChild()
    {
        return theLeftChild;
    }

    Node *getRightChild()
    {
        return theRightChild;
    }

    BoundingBox getBoundingBox()
    {
        return theBoundingBox;
    }

    int getStartTriangle() const
    {
        return theStartTriangle;
    }

    int getEndTriangle() const
    {
        return theEndTriangle;
    }
};

class TriangleMesh : public Geometry
{
    std::vector<TriangleIndices> indices;
    std::vector<Vector> vertices;
    std::vector<Vector> normals;
    std::vector<Vector> uvs;
    std::vector<Vector> aVertexcolors;
    BoundingBox theBoundingBox;
    Node *theRoot;
    double theScale;
    Vector theOffset;

    void transformVertices()
    {
        for (Vector &aVertex : vertices)
        {
            aVertex *= theScale;
            aVertex += theOffset;
        }
    }

    void initBVH(Node *aNode, int aStartTriangle, int aEndTriangle)
    {
        BoundingBox myBoundingBox = computeBoundingBox(aStartTriangle, aEndTriangle);
        aNode->setBoundingBox(myBoundingBox);
        aNode->setTriangles(aStartTriangle, aEndTriangle);

        Vector myDiagonal = myBoundingBox.getDiagonal();
        Vector myMiddleDiagonal = myBoundingBox.getMiddleDiagonal();

        int myLongestAxis = 0;
        double myMaxAxisLength = abs(myDiagonal[0]);
        for (int i = 1; i < 3; i++)
        {
            if (abs(myDiagonal[i]) > myMaxAxisLength)
            {
                myMaxAxisLength = abs(myDiagonal[i]);
                myLongestAxis = i;
            }
        }

        int myPivotIndex = aStartTriangle;
        for (int i = aStartTriangle; i < aEndTriangle; i++)
        {
            Vector myBarycenter = (vertices[indices[i].vtxi] + vertices[indices[i].vtxj] + vertices[indices[i].vtxk]) / 3.;
            if (myBarycenter[myLongestAxis] < myMiddleDiagonal[myLongestAxis])
            {
                std::swap(indices[i], indices[myPivotIndex]);
                myPivotIndex++;
            }
        }

        if (
            myPivotIndex <= aStartTriangle ||
            myPivotIndex >= aEndTriangle - 5 ||
            aEndTriangle - aStartTriangle < 5)
            return;

        aNode->setChildren(new Node(), new Node());
        initBVH(aNode->getLeftChild(), aStartTriangle, myPivotIndex);
        initBVH(aNode->getRightChild(), myPivotIndex, aEndTriangle);
    }

    BoundingBox computeBoundingBox(int aStartTriangle, int aEndTriangle)
    {
        double xMin = INFINITY, yMin = INFINITY, zMin = INFINITY;
        double xMax = -INFINITY, yMax = -INFINITY, zMax = -INFINITY;
        for (int i = aStartTriangle; i < aEndTriangle; i++)
        {
            auto myVertices = {
                vertices[indices[i].vtxi],
                vertices[indices[i].vtxj],
                vertices[indices[i].vtxk]};
            for (auto const &aVertex : myVertices)
            {
                if (aVertex[0] < xMin)
                    xMin = aVertex[0];
                else if (aVertex[0] > xMax)
                    xMax = aVertex[0];
                if (aVertex[1] < yMin)
                    yMin = aVertex[1];
                else if (aVertex[1] > yMax)
                    yMax = aVertex[1];
                if (aVertex[2] < zMin)
                    zMin = aVertex[2];
                else if (aVertex[2] > zMax)
                    zMax = aVertex[2];
            }
        }
        return BoundingBox(Vector(xMin, yMin, zMin), Vector(xMax, yMax, zMax));
    }

public:
    ~TriangleMesh() {}
    TriangleMesh(
        double aScale,
        const Vector &aOffset,
        const Vector &aAlbedo,
        bool aIsReflective = false)
    {
        theRoot = new Node();
        theIsReflective = aIsReflective;
        theAlbedo = aAlbedo;
        theScale = aScale;
        theOffset = aOffset;
    }

    void readOBJ(const char *obj)
    {
        char matfile[255];
        char grp[255];

        FILE *f;
        f = fopen(obj, "r");
        int curGroup = -1;
        while (!feof(f))
        {
            char line[255];
            if (!fgets(line, 255, f))
                break;

            std::string linetrim(line);
            linetrim.erase(linetrim.find_last_not_of(" \r\t") + 1);
            strcpy(line, linetrim.c_str());

            if (line[0] == 'u' && line[1] == 's')
            {
                sscanf(line, "usemtl %[^\n]\n", grp);
                curGroup++;
            }

            if (line[0] == 'v' && line[1] == ' ')
            {
                Vector vec;

                Vector col;
                if (sscanf(line, "v %lf %lf %lf %lf %lf %lf\n", &vec[0], &vec[1], &vec[2], &col[0], &col[1], &col[2]) == 6)
                {
                    col[0] = std::min(1., std::max(0., col[0]));
                    col[1] = std::min(1., std::max(0., col[1]));
                    col[2] = std::min(1., std::max(0., col[2]));

                    vertices.push_back(vec);
                    aVertexcolors.push_back(col);
                }
                else
                {
                    sscanf(line, "v %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                    vertices.push_back(vec);
                }
            }
            if (line[0] == 'v' && line[1] == 'n')
            {
                Vector vec;
                sscanf(line, "vn %lf %lf %lf\n", &vec[0], &vec[1], &vec[2]);
                normals.push_back(vec);
            }
            if (line[0] == 'v' && line[1] == 't')
            {
                Vector vec;
                sscanf(line, "vt %lf %lf\n", &vec[0], &vec[1]);
                uvs.push_back(vec);
            }
            if (line[0] == 'f')
            {
                TriangleIndices t;
                int i0, i1, i2, i3;
                int j0, j1, j2, j3;
                int k0, k1, k2, k3;
                int nn;
                t.group = curGroup;

                char *consumedline = line + 1;
                int offset;

                nn = sscanf(consumedline, "%i/%i/%i %i/%i/%i %i/%i/%i%n", &i0, &j0, &k0, &i1, &j1, &k1, &i2, &j2, &k2, &offset);
                if (nn == 9)
                {
                    if (i0 < 0)
                        t.vtxi = vertices.size() + i0;
                    else
                        t.vtxi = i0 - 1;
                    if (i1 < 0)
                        t.vtxj = vertices.size() + i1;
                    else
                        t.vtxj = i1 - 1;
                    if (i2 < 0)
                        t.vtxk = vertices.size() + i2;
                    else
                        t.vtxk = i2 - 1;
                    if (j0 < 0)
                        t.uvi = uvs.size() + j0;
                    else
                        t.uvi = j0 - 1;
                    if (j1 < 0)
                        t.uvj = uvs.size() + j1;
                    else
                        t.uvj = j1 - 1;
                    if (j2 < 0)
                        t.uvk = uvs.size() + j2;
                    else
                        t.uvk = j2 - 1;
                    if (k0 < 0)
                        t.ni = normals.size() + k0;
                    else
                        t.ni = k0 - 1;
                    if (k1 < 0)
                        t.nj = normals.size() + k1;
                    else
                        t.nj = k1 - 1;
                    if (k2 < 0)
                        t.nk = normals.size() + k2;
                    else
                        t.nk = k2 - 1;
                    indices.push_back(t);
                }
                else
                {
                    nn = sscanf(consumedline, "%i/%i %i/%i %i/%i%n", &i0, &j0, &i1, &j1, &i2, &j2, &offset);
                    if (nn == 6)
                    {
                        if (i0 < 0)
                            t.vtxi = vertices.size() + i0;
                        else
                            t.vtxi = i0 - 1;
                        if (i1 < 0)
                            t.vtxj = vertices.size() + i1;
                        else
                            t.vtxj = i1 - 1;
                        if (i2 < 0)
                            t.vtxk = vertices.size() + i2;
                        else
                            t.vtxk = i2 - 1;
                        if (j0 < 0)
                            t.uvi = uvs.size() + j0;
                        else
                            t.uvi = j0 - 1;
                        if (j1 < 0)
                            t.uvj = uvs.size() + j1;
                        else
                            t.uvj = j1 - 1;
                        if (j2 < 0)
                            t.uvk = uvs.size() + j2;
                        else
                            t.uvk = j2 - 1;
                        indices.push_back(t);
                    }
                    else
                    {
                        nn = sscanf(consumedline, "%i %i %i%n", &i0, &i1, &i2, &offset);
                        if (nn == 3)
                        {
                            if (i0 < 0)
                                t.vtxi = vertices.size() + i0;
                            else
                                t.vtxi = i0 - 1;
                            if (i1 < 0)
                                t.vtxj = vertices.size() + i1;
                            else
                                t.vtxj = i1 - 1;
                            if (i2 < 0)
                                t.vtxk = vertices.size() + i2;
                            else
                                t.vtxk = i2 - 1;
                            indices.push_back(t);
                        }
                        else
                        {
                            nn = sscanf(consumedline, "%i//%i %i//%i %i//%i%n", &i0, &k0, &i1, &k1, &i2, &k2, &offset);
                            if (i0 < 0)
                                t.vtxi = vertices.size() + i0;
                            else
                                t.vtxi = i0 - 1;
                            if (i1 < 0)
                                t.vtxj = vertices.size() + i1;
                            else
                                t.vtxj = i1 - 1;
                            if (i2 < 0)
                                t.vtxk = vertices.size() + i2;
                            else
                                t.vtxk = i2 - 1;
                            if (k0 < 0)
                                t.ni = normals.size() + k0;
                            else
                                t.ni = k0 - 1;
                            if (k1 < 0)
                                t.nj = normals.size() + k1;
                            else
                                t.nj = k1 - 1;
                            if (k2 < 0)
                                t.nk = normals.size() + k2;
                            else
                                t.nk = k2 - 1;
                            indices.push_back(t);
                        }
                    }
                }

                consumedline = consumedline + offset;

                while (true)
                {
                    if (consumedline[0] == '\n')
                        break;
                    if (consumedline[0] == '\0')
                        break;
                    nn = sscanf(consumedline, "%i/%i/%i%n", &i3, &j3, &k3, &offset);
                    TriangleIndices t2;
                    t2.group = curGroup;
                    if (nn == 3)
                    {
                        if (i0 < 0)
                            t2.vtxi = vertices.size() + i0;
                        else
                            t2.vtxi = i0 - 1;
                        if (i2 < 0)
                            t2.vtxj = vertices.size() + i2;
                        else
                            t2.vtxj = i2 - 1;
                        if (i3 < 0)
                            t2.vtxk = vertices.size() + i3;
                        else
                            t2.vtxk = i3 - 1;
                        if (j0 < 0)
                            t2.uvi = uvs.size() + j0;
                        else
                            t2.uvi = j0 - 1;
                        if (j2 < 0)
                            t2.uvj = uvs.size() + j2;
                        else
                            t2.uvj = j2 - 1;
                        if (j3 < 0)
                            t2.uvk = uvs.size() + j3;
                        else
                            t2.uvk = j3 - 1;
                        if (k0 < 0)
                            t2.ni = normals.size() + k0;
                        else
                            t2.ni = k0 - 1;
                        if (k2 < 0)
                            t2.nj = normals.size() + k2;
                        else
                            t2.nj = k2 - 1;
                        if (k3 < 0)
                            t2.nk = normals.size() + k3;
                        else
                            t2.nk = k3 - 1;
                        indices.push_back(t2);
                        consumedline = consumedline + offset;
                        i2 = i3;
                        j2 = j3;
                        k2 = k3;
                    }
                    else
                    {
                        nn = sscanf(consumedline, "%i/%i%n", &i3, &j3, &offset);
                        if (nn == 2)
                        {
                            if (i0 < 0)
                                t2.vtxi = vertices.size() + i0;
                            else
                                t2.vtxi = i0 - 1;
                            if (i2 < 0)
                                t2.vtxj = vertices.size() + i2;
                            else
                                t2.vtxj = i2 - 1;
                            if (i3 < 0)
                                t2.vtxk = vertices.size() + i3;
                            else
                                t2.vtxk = i3 - 1;
                            if (j0 < 0)
                                t2.uvi = uvs.size() + j0;
                            else
                                t2.uvi = j0 - 1;
                            if (j2 < 0)
                                t2.uvj = uvs.size() + j2;
                            else
                                t2.uvj = j2 - 1;
                            if (j3 < 0)
                                t2.uvk = uvs.size() + j3;
                            else
                                t2.uvk = j3 - 1;
                            consumedline = consumedline + offset;
                            i2 = i3;
                            j2 = j3;
                            indices.push_back(t2);
                        }
                        else
                        {
                            nn = sscanf(consumedline, "%i//%i%n", &i3, &k3, &offset);
                            if (nn == 2)
                            {
                                if (i0 < 0)
                                    t2.vtxi = vertices.size() + i0;
                                else
                                    t2.vtxi = i0 - 1;
                                if (i2 < 0)
                                    t2.vtxj = vertices.size() + i2;
                                else
                                    t2.vtxj = i2 - 1;
                                if (i3 < 0)
                                    t2.vtxk = vertices.size() + i3;
                                else
                                    t2.vtxk = i3 - 1;
                                if (k0 < 0)
                                    t2.ni = normals.size() + k0;
                                else
                                    t2.ni = k0 - 1;
                                if (k2 < 0)
                                    t2.nj = normals.size() + k2;
                                else
                                    t2.nj = k2 - 1;
                                if (k3 < 0)
                                    t2.nk = normals.size() + k3;
                                else
                                    t2.nk = k3 - 1;
                                consumedline = consumedline + offset;
                                i2 = i3;
                                k2 = k3;
                                indices.push_back(t2);
                            }
                            else
                            {
                                nn = sscanf(consumedline, "%i%n", &i3, &offset);
                                if (nn == 1)
                                {
                                    if (i0 < 0)
                                        t2.vtxi = vertices.size() + i0;
                                    else
                                        t2.vtxi = i0 - 1;
                                    if (i2 < 0)
                                        t2.vtxj = vertices.size() + i2;
                                    else
                                        t2.vtxj = i2 - 1;
                                    if (i3 < 0)
                                        t2.vtxk = vertices.size() + i3;
                                    else
                                        t2.vtxk = i3 - 1;
                                    consumedline = consumedline + offset;
                                    i2 = i3;
                                    indices.push_back(t2);
                                }
                                else
                                {
                                    consumedline = consumedline + 1;
                                }
                            }
                        }
                    }
                }
            }
        }
        fclose(f);
        transformVertices();
        initBVH(theRoot, 0, indices.size());
    }

    Intersection intersect(const Ray &aRay) const override
    {
        Intersection myIntersection = Intersection();
        double t;
        double tMin = INFINITY;

        if (!theRoot->getBoundingBox().intersect(aRay, t))
            return myIntersection;

        std::list<Node *> myNodesToVisit;
        myNodesToVisit.push_front(theRoot);
        while (!myNodesToVisit.empty())
        {
            Node *myCurrentNode = myNodesToVisit.back();
            myNodesToVisit.pop_back();
            Node *myLeftChild = myCurrentNode->getLeftChild();
            Node *myRightChild = myCurrentNode->getRightChild();
            if (myCurrentNode->hasChildren())
            {
                if (myLeftChild->getBoundingBox().intersect(aRay, t) && t < tMin)
                {
                    myNodesToVisit.push_back(myLeftChild);
                }
                if (myRightChild->getBoundingBox().intersect(aRay, t) && t < tMin)
                {
                    myNodesToVisit.push_back(myRightChild);
                }
            }
            else
            {
                const double epsilon = 1e-5;
                Vector A, B, C, N, e1, e2;
                Vector O = aRay.getOrigin();
                Vector u = aRay.getDirection();
                for (int i = myCurrentNode->getStartTriangle(); i < myCurrentNode->getEndTriangle(); i++)
                {
                    TriangleIndices triangle = indices[i];
                    A = vertices[triangle.vtxi];
                    B = vertices[triangle.vtxj];
                    C = vertices[triangle.vtxk];
                    e1 = B - A;
                    e2 = C - A;
                    N = cross(e1, e2);
                    Vector OA = A - O;
                    Vector myCrossVector = cross(OA, u);
                    double myDotValue = dot(u, N);

                    double beta = dot(e2, myCrossVector) / myDotValue;
                    double gamma = -dot(e1, myCrossVector) / myDotValue;
                    double alpha = 1. - beta - gamma;

                    if (alpha > 0. && beta > 0. && gamma > 0.)
                    {
                        double t = dot(OA, N) / myDotValue;
                        if (epsilon < t && t < tMin)
                        {
                            tMin = t;
                            myIntersection = Intersection(
                                t,
                                Vector(0, 0, 0),
                                A + beta * e1 + gamma * e2,
                                N,
                                theAlbedo,
                                true,
                                theIsReflective);
                        }
                    }
                }
            }
        }
        return myIntersection;
    }
};