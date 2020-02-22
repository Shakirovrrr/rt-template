#include "refraction.h"

Refraction::Refraction(short width, short height) :Reflection(width, height) {
	raytracing_depth = 5;
}

Refraction::~Refraction() {}

Payload Refraction::Hit(const Ray &ray, const IntersectableData &data, const MaterialTriangle *triangle, const unsigned int raytrace_depth) const {
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

	if (triangle->reflectiveness_and_transparency) {
		float kr;
		float cosIn = std::max(-1.0f, std::min(1.0f, linalg::dot(ray.direction, normal)));
		float etaIn = 1.0f;
		float etaTr = triangle->ior;

		if (cosIn > 0.0f) {
			std::swap(etaIn, etaTr);
		}

		float sinTr = etaIn / etaTr * std::sqrtf(std::max(0.0f, 1 - cosIn * cosIn));
		if (sinTr >= 1.0f) {
			kr = 1.0f;
		} else {
			float cosTr = std::sqrtf(std::max(0.0f, 1 - sinTr * sinTr));
			cosIn = std::fabs(cosIn);
			float Rs = ((etaTr * cosIn) - (etaIn * cosTr)) / ((etaTr * cosIn) + (etaIn * cosTr));
			float Rp = ((etaIn * cosIn) - (etaTr * cosTr)) / ((etaIn * cosIn) + (etaTr * cosTr));
			kr = (Rs * Rs + Rp * Rp) / 2.0f;
		}

		bool outside = (linalg::dot(ray.direction, normal) < 0.0f);
		float3 bias = 0.001f * normal;
		Payload refractionPayload;

		if (kr < 1.0f) {
			float cosIn = std::max(-1.0f, std::min(1.0f, linalg::dot(ray.direction, normal)));
			float etaIn = 1.0f;
			float etaTr = triangle->ior;

			if (cosIn < 0.0f) {
				cosIn = -cosIn;
			} else {
				std::swap(etaIn, etaTr);
			}

			float eta = etaIn / etaTr;
			float k = 1.0f - eta * eta * (1.0f - cosIn * cosIn);
			float3 refractionDir = {0, 0, 0};

			if (k >= 0.0f) {
				refractionDir = eta * ray.direction + (eta * cosIn - std::sqrtf(k)) * normal;
			}

			Ray refractionRay(outside ? x - bias : x + bias, refractionDir);
			refractionPayload = TraceRay(refractionRay, raytrace_depth - 1);
		}

		float3 reflectionDir = ray.direction - 2.0f * linalg::dot(normal, ray.direction) * normal;
		Ray reflectionRay(outside ? x + bias : x - bias, reflectionDir);
		Payload reflectionPayload = TraceRay(reflectionRay, raytrace_depth - 1);

		Payload combined;
		combined.color = reflectionPayload.color * kr + refractionPayload.color * (1.0f - kr);

		return combined;
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
