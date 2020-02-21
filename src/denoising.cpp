#include "denoising.h"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"
#include <time.h>
#include <omp.h>
#include <random>

Denoising::Denoising(short width, short height) : AABB(width, height)
{
}

Denoising::~Denoising()
{
}

void Denoising::Clear()
{
}

Payload Denoising::Hit(const Ray& ray, const IntersectableData& data, const MaterialTriangle* triangle, const unsigned int max_raytrace_depth) const
{
}

void Denoising::SetHistory(unsigned short x, unsigned short y, float3 color)
{
}

float3 Denoising::GetHistory(unsigned short x, unsigned short y) const
{
}


Payload Denoising::Miss(const Ray& ray) const
{
}

int Denoising::GetRandom(const int thread_num) const
{
}

void Denoising::DrawScene(int max_frame_number)
{
}

void Denoising::LoadBlueNoise(std::string file_name)
{
}
