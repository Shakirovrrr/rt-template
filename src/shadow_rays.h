#pragma once

#include "lighting.h"

class ShadowRays : public Lighting
{
public:
	ShadowRays(short width, short height);
	virtual ~ShadowRays();

protected:
	virtual Payload TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const;
	virtual Payload Hit(const Ray& ray, const IntersectableData& data, const MaterialTriangle* triangle, const unsigned int max_raytrace_depth) const;
	virtual float TraceShadowRay(const Ray& ray, const float max_t) const;
};