#include "ray_generation.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "stb_image_write.h"

RayGenerationApp::RayGenerationApp(short width, short height) :
	width(width),
	height(height)
{
}

RayGenerationApp::~RayGenerationApp()
{
}

void RayGenerationApp::SetCamera(float3 position, float3 direction, float3 approx_up)
{
}

void RayGenerationApp::Clear()
{
}

void RayGenerationApp::DrawScene()
{

}

int RayGenerationApp::Save(std::string filename) const
{

}

Payload RayGenerationApp::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
}

Payload RayGenerationApp::Miss(const Ray& ray) const
{

	//color = { 0.0f, 0.2f, 0.7f + 0.3f * t };
}

void RayGenerationApp::SetPixel(unsigned short x, unsigned short y, float3 color)
{
}

Camera::Camera()
{
}

Camera::~Camera()
{
}

void Camera::SetPosition(float3 position)
{
}

void Camera::SetDirection(float3 direction)
{
}

void Camera::SetUp(float3 approx_up)
{
}

void Camera::SetRenderTargetSize(short width, short height)
{
}

Ray Camera::GetCameraRay(short x, short y) const
{
}

Ray Camera::GetCameraRay(short x, short y, float3 jitter) const
{
}
