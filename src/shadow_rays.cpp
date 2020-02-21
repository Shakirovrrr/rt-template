#include "shadow_rays.h"

ShadowRays::ShadowRays(short width, short height) : Lighting(width, height) {}

ShadowRays::~ShadowRays() {}

Payload ShadowRays::TraceRay(const Ray &ray, const unsigned int max_raytrace_depth) const {
	if (max_raytrace_depth <= 0) {
		return Miss(ray);
	}
	
	IntersectableData closestData(t_max);
	MaterialTriangle *closestTriangle = nullptr;

	for (auto &object : material_objects) {
		IntersectableData data = object->Intersect(ray);
		if (data.t < closestData.t && data.t > t_min) {
			closestData = data;
			closestTriangle = object;
		}
	}

	if (closestData.t < t_max) {
		return Hit(ray, closestData, closestTriangle, max_raytrace_depth);
	}

	return Miss(ray);
}


Payload ShadowRays::Hit(const Ray &ray, const IntersectableData &data, const MaterialTriangle *triangle, const unsigned int max_raytrace_depth) const {
	if (max_raytrace_depth <= 0) {
		return Miss(ray);
	}
	
	if (triangle == nullptr) {
		return Miss(ray);
	}

	Payload payload;
	payload.color = triangle->emissive_color;

	float3 x = ray.position + ray.direction * data.t;
	float3 normal = triangle->GetNormal(data.baricentric);

	for (auto const &light : lights) {
		Ray toLight(x, light->position - x);
		float toLightDist = linalg::length(light->position - x);

		float traceShadow = TraceShadowRay(toLight, toLightDist);
		if (std::fabs(traceShadow - toLightDist) > 0.001f) {
			continue;
		}

		payload.color += light->color * triangle->diffuse_color
			* std::max(0.0f, linalg::dot(normal, toLight.direction));

		float3 reflectionDir = 2.0f * linalg::dot(normal, toLight.direction) * normal - toLight.direction;
		payload.color += light->color * triangle->specular_color
			* std::powf(std::max(0.0f, linalg::dot(ray.direction, reflectionDir)), triangle->specular_exponent);
	}

	return payload;
}

float ShadowRays::TraceShadowRay(const Ray &ray, const float max_t) const {
	IntersectableData closestData(max_t);
	for (auto &object : material_objects) {
		IntersectableData data = object->Intersect(ray);
		if (data.t < closestData.t && data.t > t_min) {
			return data.t;
		}
	}

	return max_t;
}

