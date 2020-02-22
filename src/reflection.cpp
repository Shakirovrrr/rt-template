#include "reflection.h"

Reflection::Reflection(short width, short height) :ShadowRays(width, height) {}

Reflection::~Reflection() {}

Payload Reflection::Hit(const Ray &ray, const IntersectableData &data, const MaterialTriangle *triangle, const unsigned int raytrace_depth) const {
	if (raytrace_depth <= 0) {
		return Miss(ray);
	}

	if (triangle == nullptr) {
		return Miss(ray);
	}

	Payload payload;
	payload.color = triangle->emissive_color;

	float3 x = ray.position + ray.direction * data.t;
	float3 normal = triangle->GetNormal(data.baricentric);

	if (triangle->reflectiveness) {
		float3 reflectionDir = ray.direction - 2.0f * linalg::dot(normal, ray.direction) * normal;
		Ray reflectionRay(x + reflectionDir * 0.001f, reflectionDir);
		return TraceRay(reflectionRay, raytrace_depth - 1);
	}

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
