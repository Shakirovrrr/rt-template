#pragma once

#include "ray_generation.h"

#include <vector>

class IntersectableData
{
public:
	IntersectableData(float t) : t(t), baricentric(float3{ 1, 0, 0 }) {};
	IntersectableData(float t, float3 baricentric) : t(t), baricentric(baricentric) {};
	~IntersectableData() {};
	float t;
	float3 baricentric;
};

class Intersectable
{
public:
	virtual IntersectableData Intersect(const Ray& ray) const = 0;
};

class Sphere : public Intersectable
{
public:
	Sphere(float3 center, float radius);
	~Sphere();
	IntersectableData Intersect(const Ray& ray) const;
protected:
	float3 center;
	float radius;
};

class Vertex
{
public:
	Vertex(float3 position) : position(position), normal(float3{ 0.0, 0.0, 0.0 }), tex(float3{ 0.0, 0.0, 0.0 }), color(float3{ 1.0, 1.0, 1.0 }) {};
	Vertex(float3 position, float3 normal) : position(position), tex(float3{ 0.0, 0.0, 0.0 }), color(float3{ 1.0, 1.0, 1.0 }) { this->normal = normalize(normal); };
	Vertex(float3 position, float3 normal, float3 tex) : position(position), tex(tex), color(float3{ 1.0, 1.0, 1.0 }) { this->normal = normalize(normal); };
	Vertex(float3 position, float3 normal, float3 tex, float3 color) : position(position), tex(tex), color(color) { this->normal = normalize(normal); };

	~Vertex() {};

	float3 position;
	float3 normal;
	float3 tex;

	float3 color;
	float t = 1000.f;

};

class Triangle : public Intersectable
{
public:
	Triangle(Vertex a, Vertex b, Vertex c);
	Triangle();
	~Triangle();
	IntersectableData Intersect(const Ray& ray) const;

	Vertex a;
	Vertex b;
	Vertex c;
protected:
	float3 ba;
	float3 ca;
};



class MTAlgorithm : public RayGenerationApp
{
public:
	MTAlgorithm(short width, short height);
	virtual ~MTAlgorithm();

	virtual int LoadGeometry(std::string filename);

protected:
	virtual Payload TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const;
	virtual Payload Hit(const Ray& ray, const IntersectableData& t) const;

	std::vector<Intersectable*> objects;

	const float t_min = 0.01f;
	const float t_max = 1000.f;
};