#include "ray_generation.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

RayGenerationApp::RayGenerationApp(short width, short height) :
	width(width),
	height(height) {}

RayGenerationApp::~RayGenerationApp() {}

void RayGenerationApp::SetCamera(float3 position, float3 direction, float3 approx_up) {
	camera.SetPosition(position);
	camera.SetDirection(direction);
	camera.SetUp(approx_up);
	camera.SetRenderTargetSize(width, height);
}

void RayGenerationApp::Clear() {
	frame_buffer.resize(static_cast<size_t>(width) *static_cast<size_t>(height));
}

void RayGenerationApp::DrawScene() {
	for (unsigned int x = 0; x < width; x++) {
#pragma omp parallel for
		for (int y = 0; y < height; y++) {
			Ray ray = camera.GetCameraRay(x, y);
			Payload payload = TraceRay(ray, raytracing_depth);
			SetPixel(x, y, payload.color);
		}
	}
}

int RayGenerationApp::Save(std::string filename) const {
	int result = stbi_write_png(filename.c_str(), width, height, 3, frame_buffer.data(), width * sizeof(uint8_t) * 3);

	if (result == 1) {
		std::system(std::string("start " + filename).c_str());
	}

	return result - 1;
}

Payload RayGenerationApp::TraceRay(const Ray &ray, const unsigned int max_raytrace_depth) const {
	return Miss(ray);
}

Payload RayGenerationApp::Miss(const Ray &ray) const {
	float t = 0.5f * (ray.direction.y + 1.0f);
	//float3 color = {ray.direction.x, ray.direction.y, 0.7f + 0.3f * t};
	float3 color = { 0.0f, 0.2f, 0.7f + 0.3f * t };
	return Payload(color);
}

void RayGenerationApp::SetPixel(unsigned short x, unsigned short y, float3 color) {
	unsigned int ix = y * width + x;

	if (ix >= 0 && ix < frame_buffer.size()) {
		frame_buffer[ix] = byte3(color * 255);
	}
}

Camera::Camera() {}

Camera::~Camera() {}

void Camera::SetPosition(float3 position) {
	this->position = position;
}

void Camera::SetDirection(float3 direction) {
	this->direction = linalg::normalize(direction - position);
}

void Camera::SetUp(float3 approx_up) {
	this->right = linalg::normalize(linalg::cross(direction, linalg::normalize(approx_up)));
	this->up = linalg::normalize(linalg::cross(right, direction));
}

void Camera::SetRenderTargetSize(short width, short height) {
	this->width = width;
	this->height = height;
}

Ray Camera::GetCameraRay(short x, short y) const {
	float aspectRatio = static_cast<float>(width) / static_cast<float>(height);

	float u = (2.0f * (static_cast<float>(x) + 0.5f) / static_cast<float>(width) - 1.0f) * aspectRatio;
	float v = (2.0f * (static_cast<float>(y) + 0.5f) / static_cast<float>(height) - 1.0f);

	float3 direction = this->direction + u * right - v * up;

	return Ray(this->position, direction);
}

Ray Camera::GetCameraRay(short x, short y, float3 jitter) const {
	return GetCameraRay(x, y);
}
