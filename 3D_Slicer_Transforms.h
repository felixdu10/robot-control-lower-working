#ifndef SLICER_TRANSFORMS
#define SLICER_TRANSFORMS

#include "Point.h"
#include "Transform.h"



class STL_OBJECT_TRANSFORM {
    public:
    STL_OBJECT_TRANSFORM();
    std::string name;
    Transform initial_transform;
    Transform kinematically_defined_transform;
    virtual void set_transform(Point, Point);

};

class STL_OBJECT_TRANSFORM_MANAGER {
    public:
    STL_OBJECT_TRANSFORM* objects;
    int number_of_objects = 0;
    void add_object(STL_OBJECT_TRANSFORM);
    void add_objects(STL_OBJECT_TRANSFORM[], int num_objects);
};

class STL_LINK : STL_OBJECT_TRANSFORM {
    STL_LINK();
    void set_transform(Point, Point);
};

#endif