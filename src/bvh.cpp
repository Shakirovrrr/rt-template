#include "bvh.h"


BVH::BVH(short width, short height) :AABB(width, height)
{
}

BVH::~BVH()
{
}

bool cmp(const Mesh& a, const Mesh& b)
{
}

void BVH::BuildBVH()
{
}

Payload BVH::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
}

float BVH::TraceShadowRay(const Ray& ray, const float max_t) const
{
}

bool TLAS::AABBTest(const Ray& ray) const
{
}

void TLAS::AddMesh(const Mesh mesh)
{
}
