#pragma once

#include "linalg.h"
using namespace linalg::aliases;
using namespace linalg::ostream_overloads;

#include <string>
#include <vector>


#include <iostream>

class Ray
{
public:
	Ray(float3 position, float3 direction) : position(position) {
		this->direction = normalize(direction);
	};
	~Ray() {};
	float3 position;
	float3 direction;
};

class Payload
{
public:
	Payload() { color = float3{ 0, 0, 0 };};
	~Payload() {};
	float3 color;
};


class Camera
{
public:
	Camera();
	~Camera();

	void SetPosition(float3 position);
	void SetDirection(float3 direction);
	void SetUp(float3 approx_up);
	void SetRenderTargetSize(short width, short height);

	Ray GetCameraRay(short x, short y) const;
	Ray GetCameraRay(short x, short y, float3 jitter) const;

private:
	float3 position;
	float3 direction;
	float3 up;
	float3 right;

	short width;
	short height;
};



class RayGenerationApp
{
public:
	RayGenerationApp(short width, short height);
	virtual ~RayGenerationApp();

	void SetCamera(float3 position, float3 direction, float3 approx_up);
	void Clear();
	virtual void DrawScene();
	int Save(std::string filename) const;
	// Public method to compare the final image with a reference
	std::vector<byte3> GetFrameBuffer() const { return frame_buffer; }
protected:
	void SetPixel(const unsigned short x, const unsigned short y, const float3 color);
	virtual Payload TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const;

	virtual Payload Miss(const Ray& ray) const;

	short width;
	short height;

	unsigned int raytracing_depth = 10;

	std::vector<byte3> frame_buffer;
	Camera camera;
};