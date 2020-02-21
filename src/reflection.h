#pragma once

#include "shadow_rays.h"

class Reflection: public ShadowRays
{
public:
	Reflection(short width, short height);
	virtual ~Reflection();
protected:
	virtual Payload Hit(const Ray& ray, const IntersectableData& data, const MaterialTriangle* triangle, const unsigned int max_raytrace_depth) const;
};