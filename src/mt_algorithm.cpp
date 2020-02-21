#include "mt_algorithm.h"


Sphere::Sphere(float3 center, float radius) :
	center(center), radius(radius) {}

Sphere::~Sphere() {}

IntersectableData Sphere::Intersect(const Ray &ray) const {
	float3 oc = center - ray.position;
	float a = linalg::dot(ray.direction, ray.direction);
	float b = -2.0f * linalg::dot(oc, ray.direction);
	float c = linalg::dot(oc, oc) - radius * radius;
	float disc = b * b - 4 * a * c;
	if (disc < 0) {
		return disc;
	}

	float x0 = (-b - sqrtf(disc)) / (2.0f * a);
	float x1 = (-b + sqrtf(disc)) / (2.0f * a);
	float t = std::min(x0, x1);

	if (t < 0.0f) {
		t = std::max(x0, x1);
	}

	return t;
}



MTAlgorithm::MTAlgorithm(short width, short height) : RayGenerationApp(width, height) {}

MTAlgorithm::~MTAlgorithm() {}

int MTAlgorithm::LoadGeometry(std::string filename) {
	objects.push_back(new Sphere(float3 {2, 0, -1}, 0.4f));

	Vertex a(float3 {-.5f, -.5f, -1.f});
	Vertex b(float3 {.5f, -.5f, -1.f});
	Vertex c(float3 {-0.f, .5f, -1.f});
	objects.push_back(new Triangle(a, b, c));

	return 0;
}

Payload MTAlgorithm::TraceRay(const Ray &ray, const unsigned int max_raytrace_depth) const {
	IntersectableData closestData(t_max);
	for (auto &object : objects) {
		IntersectableData data = object->Intersect(ray);
		if (data.t < closestData.t && data.t > t_min) {
			closestData = data;
		}
	}

	if (closestData.t < t_max) {
		return Hit(ray, closestData);
	}

	return Miss(ray);
}

Payload MTAlgorithm::Hit(const Ray &ray, const IntersectableData &data) const {
	Payload payload;
	payload.color = float3 {1, 0, 0};
	return payload;
}

Triangle::Triangle(Vertex a, Vertex b, Vertex c) :
	a(a), b(b), c(c) {
	ba = b.position - a.position;
	ca = c.position - a.position;
}

Triangle::Triangle() :
	a(float3 {0, 0 ,0}), b(float3 {0, 0 ,0}), c(float3 {0, 0 ,0}) {}

Triangle::~Triangle() = default;

IntersectableData Triangle::Intersect(const Ray &ray) const {
	float3 vP = cross(ray.direction, ca);
	float det = linalg::dot(ba, vP);

	if (det > -1e-8 && det < 1e-8) {
		return IntersectableData(-1.0f);
	}

	float3 vT = ray.position - a.position;
	float u = linalg::dot(vT, vP) / det;
	if (u < 0 || u>1) {
		return IntersectableData(-1.0f);
	}

	float3 vQ = linalg::cross(vT, ba);
	float v = dot(ray.direction, vQ) / det;
	if (v < 0 || u + v>1) {
		return IntersectableData(-1.0f);
	}

	float t = linalg::dot(ca, vQ) / det;
	return IntersectableData(t, float3 {1 - u - v, u, v});
}
