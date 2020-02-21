#pragma once

#include "aabb.h"

class Denoising: public AABB
{
public:
	Denoising(short width, short height);
	virtual ~Denoising();
	virtual void Clear();
	virtual void DrawScene(int max_frame_number);
	void LoadBlueNoise(std::string file_name);

protected:
	Payload Hit(const Ray& ray, const IntersectableData& data, const MaterialTriangle* triangle, const unsigned int max_raytrace_depth) const;
	void SetHistory(unsigned short x, unsigned short y, float3 color);
	float3 GetHistory(unsigned short x, unsigned short y) const;
	Payload Miss(const Ray& ray) const;
	std::vector<float3> history_buffer;
	std::vector<float3> blue_noise;

	int GetRandom(const int thread_num) const;
};