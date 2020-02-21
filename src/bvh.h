#pragma once

#include "aabb.h"

class TLAS
{
public:
	TLAS() { meshes.clear(); };
	virtual ~TLAS() { meshes.clear(); };

	bool AABBTest(const Ray& ray) const;
	void AddMesh(const Mesh mesh);

	float3 aabb_min;
	float3 aabb_max;
	float3 aabb_center() const { return aabb_min + (aabb_max - aabb_min) / 2.0f; };

	const std::vector<Mesh> GetMeshes() const { return meshes; };

protected:
	std::vector<Mesh> meshes;
};

class BVH : public AABB
{
public:
	BVH(short width, short height);
	virtual ~BVH();

	virtual void BuildBVH();

	virtual Payload TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const;
	virtual float TraceShadowRay(const Ray& ray, const float max_t) const;

protected:
	std::vector<TLAS> tlases;
};