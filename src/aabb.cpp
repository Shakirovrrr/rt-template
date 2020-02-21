#include "aabb.h"

//#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

AABB::AABB(short width, short height) :AntiAliasing(width, height)
{
}

AABB::~AABB()
{
}

int AABB::LoadGeometry(std::string filename)
{
}

Payload AABB::TraceRay(const Ray& ray, const unsigned int max_raytrace_depth) const
{
}

float AABB::TraceShadowRay(const Ray& ray, const float max_t) const
{
}

void Mesh::AddTriangle(const MaterialTriangle triangle)
{
}

bool Mesh::AABBTest(const Ray& ray) const
{
}
