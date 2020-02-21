#pragma once

#include "anti_aliasing.h"

class Mesh
{
public:
	Mesh() { triangles.clear(); };
	virtual ~Mesh() { triangles.clear(); };

	void AddTriangle(const MaterialTriangle triangle);
	const std::vector<MaterialTriangle>& Triangles() const { return triangles; };
	bool AABBTest(const Ray& ray) const;

	float3 aabb_min;
	float3 aabb_max;
	float3 aabb_center() const { return aabb_min + (aabb_max - aabb_min) / 2.0f; };
protected:
	std::vector<MaterialTriangle> triangles;
};

class AABB : public AntiAliasing
{
public:
	AABB(short width, short height);
	virtual ~AABB();

	virtual int LoadGeometry(std::string filename);
	virtual Payload TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const;
	virtual float TraceShadowRay(const Ray& ray, const float max_t) const;

protected:
	std::vector<Mesh> meshes;
};