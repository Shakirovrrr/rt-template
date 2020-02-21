#pragma once

#include "reflection.h"

class Refraction : public Reflection
{
public:
	Refraction(short width, short height);
	virtual ~Refraction();
protected:
	virtual Payload Hit(const Ray& ray, const IntersectableData& data, const MaterialTriangle* triangle, const unsigned int max_raytrace_depth) const;
};