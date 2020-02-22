#include "denoising.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <time.h>
#include <omp.h>
#include <random>

Denoising::Denoising(short width, short height) : AABB(width, height) {
	raytracing_depth = 16;
}

Denoising::~Denoising() {}

void Denoising::Clear() {
	history_buffer.resize(width * height);
	frame_buffer.resize(width * height);
}

Payload Denoising::Hit(const Ray &ray, const IntersectableData &data, const MaterialTriangle *triangle, const unsigned int raytrace_depth) const {
	if (raytrace_depth <= 0) {
		return Miss(ray);
	}

	if (triangle == nullptr) {
		return Miss(ray);
	}

	Payload payload;
	payload.color = triangle->emissive_color;
	if (payload.color > float3 {0, 0, 0}) {
		return payload;
	}

	float3 x = ray.position + ray.direction * data.t;
	float3 normal = triangle->GetNormal(data.baricentric);

	if (triangle->reflectiveness) {
		float3 reflectionDir = ray.direction - 2.0f * linalg::dot(normal, ray.direction) * normal;
		Ray reflectionRay(x + reflectionDir * 0.001f, reflectionDir);
		return TraceRay(reflectionRay, raytrace_depth - 1);
	}

	const int nSecondaryRays = 1;
	float3 color;
	for (int i = 0; i < nSecondaryRays;i++) {
		float3 randomDir = blue_noise[GetRandom(omp_get_thread_num()+clock())];
		if (linalg::dot(randomDir, normal) <= 0.0f) {
			randomDir = -randomDir;
		}

		Ray toLight(x, randomDir);
		// float toLightDist = linalg::length(light->position - x);
		Payload lightPayload = TraceRay(toLight, raytrace_depth - 1);

		/*float traceShadow = TraceShadowRay(toLight, toLightDist);
		if (std::fabs(traceShadow - toLightDist) > 0.001f) {
			continue;
		}*/

		color += lightPayload.color * triangle->diffuse_color
			* std::max(0.0f, linalg::dot(normal, toLight.direction));

		/*float3 reflectionDir = 2.0f * linalg::dot(normal, toLight.direction) * normal - toLight.direction;
		payload.color += light->color * triangle->specular_color
			* std::powf(std::max(0.0f, linalg::dot(ray.direction, reflectionDir)), triangle->specular_exponent);*/
	}

	payload.color += color / nSecondaryRays;

	return payload;
}

void Denoising::SetHistory(unsigned short x, unsigned short y, float3 color) {
	history_buffer[static_cast<size_t>(y) *static_cast<size_t>(width) + static_cast<size_t>(x)] = color;
}

float3 Denoising::GetHistory(unsigned short x, unsigned short y) const {
	return history_buffer[static_cast<size_t>(y) *static_cast<size_t>(width) + static_cast<size_t>(x)];
}


Payload Denoising::Miss(const Ray &ray) const {
	return Payload();
}

int Denoising::GetRandom(const int thread_num) const {
	static std::default_random_engine generator(thread_num);
	static std::uniform_int_distribution<int> distribution(0, 512 * 512);
	return distribution(generator);
}

void Denoising::DrawScene(int max_frame_number) {
	camera.SetRenderTargetSize(width, height);

	for (int frameNumber = 0; frameNumber < max_frame_number; frameNumber++) {
		std::cout << "Frame " << (frameNumber + 1) << std::endl;
#pragma omp parallel for
		for (int x = 0; x < width; x++) {
#pragma omp parallel for
			for (int y = 0; y < height; y++) {
				Ray ray = camera.GetCameraRay(x, y);
				Payload payload = TraceRay(ray, raytracing_depth);
				SetPixel(x, y, payload.color);
				SetHistory(x, y, GetHistory(x, y) + payload.color);
			}
		}
	}

	for (short x = 0; x < width; x++) {
		for (short y = 0; y < height; y++) {
			SetPixel(x, y, GetHistory(x, y) / max_frame_number);
		}
	}
}

void Denoising::LoadBlueNoise(std::string file_name) {
	int width, height, channels;
	unsigned char *img = stbi_load(file_name.c_str(), &width, &height, &channels, 0);

	std::vector<byte3> reference;
	for (int i = 0; i < width * height; i++) {
		float3 pixel {
			(img[channels * i] - 128.0f) / 128.0f,
			(img[channels * i + 1] - 128.0f) / 128.0f,
			(img[channels * i + 2] - 128.0f) / 128.0f
		};
		blue_noise.push_back(pixel);
	}
}
