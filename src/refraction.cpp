#include "refraction.h"

Refraction::Refraction(short width, short height) :Reflection(width, height)
{
}

Refraction::~Refraction()
{
}

Payload Refraction::Hit(const Ray& ray, const IntersectableData& data, const MaterialTriangle* triangle, const unsigned int max_raytrace_depth) const
{
}
