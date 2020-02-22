#include "lighting.h"

#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

#include <algorithm>

Lighting::Lighting(short width, short height) : MTAlgorithm(width, height) {}

Lighting::~Lighting() {}

int Lighting::LoadGeometry(std::string filename) {
	tinyobj::attrib_t attrib;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string warn;
	std::string err;

	size_t delimeter = filename.find_last_of('//');
	std::string dir = filename.substr(0, delimeter);

	bool ret = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, filename.c_str(), dir.c_str());

	if (!warn.empty()) {
		std::cout << warn << std::endl;
	}

	if (!err.empty()) {
		std::cerr << err << std::endl;
	}

	if (!ret) {
		exit(1);
	}

	// Loop over shapes
	for (size_t s = 0; s < shapes.size(); s++) {
		// Loop over faces(polygon)
		size_t index_offset = 0;
		for (size_t f = 0; f < shapes[s].mesh.num_face_vertices.size(); f++) {
			int fv = shapes[s].mesh.num_face_vertices[f];

			// Loop over vertices in the face.
			std::vector <Vertex> vertices;
			for (size_t v = 0; v < fv; v++) {
				// access to vertex
				tinyobj::index_t idx = shapes[s].mesh.indices[index_offset + v];
				tinyobj::real_t vx = attrib.vertices[3 * idx.vertex_index + 0];
				tinyobj::real_t vy = attrib.vertices[3 * idx.vertex_index + 1];
				tinyobj::real_t vz = attrib.vertices[3 * idx.vertex_index + 2];

				// tinyobj::real_t tx = attrib.texcoords[2 * idx.texcoord_index + 0];
				// tinyobj::real_t ty = attrib.texcoords[2 * idx.texcoord_index + 1];

				if (idx.normal_index >= 0) {
					tinyobj::real_t nx = attrib.normals[3 * idx.normal_index + 0];
					tinyobj::real_t ny = attrib.normals[3 * idx.normal_index + 1];
					tinyobj::real_t nz = attrib.normals[3 * idx.normal_index + 2];
					vertices.push_back(Vertex(float3 {vx, vy, vz}, float3 {nx, ny, nz}));
				} else {
					vertices.push_back(Vertex(float3 {vx, vy, vz}));
				}

				// Optional: vertex colors
				// tinyobj::real_t red = attrib.colors[3*idx.vertex_index+0];
				// tinyobj::real_t green = attrib.colors[3*idx.vertex_index+1];
				// tinyobj::real_t blue = attrib.colors[3*idx.vertex_index+2];
			}
			index_offset += fv;

			MaterialTriangle *triangle = new MaterialTriangle(vertices[0], vertices[1], vertices[2]);

			int materialId = shapes[s].mesh.material_ids[f];
			tinyobj::material_t material = materials[materialId];
			triangle->SetEmisive(float3 {material.emission});
			triangle->SetAmbient(float3 {material.ambient});
			triangle->SetDiffuse(float3 {material.diffuse});
			triangle->SetSpecular(float3 {material.specular}, material.shininess);
			triangle->SetReflectiveness(material.illum == 5);
			triangle->SetReflectivenessAndTransparency(material.illum == 7);
			triangle->SetIor(material.ior);

			material_objects.push_back(triangle);

			// per-face material
			// shapes[s].mesh.material_ids[f];
		}
	}

	return 0;
}

void Lighting::AddLight(Light *light) {
	lights.push_back(light);
}

Payload Lighting::TraceRay(const Ray &ray, const unsigned int max_raytrace_depth) const {
	IntersectableData closestData(t_max);
	MaterialTriangle *closestTriangle = nullptr;
	for (auto &object : material_objects) {
		IntersectableData data = object->Intersect(ray);
		if (data.t < closestData.t && data.t > t_min) {
			closestData = data;
			closestTriangle = object;
		}
	}

	if (closestData.t < t_max) {
		return Hit(ray, closestData, closestTriangle);
	}

	return Miss(ray);
}


Payload Lighting::Hit(const Ray &ray, const IntersectableData &data, const MaterialTriangle *triangle) const {
	if (triangle == nullptr) {
		return Miss(ray);
	}

	Payload payload;
	payload.color = triangle->emissive_color;

	float3 x = ray.position + ray.direction * data.t;
	float3 normal = triangle->GetNormal(data.baricentric);

	for (auto const &light : lights) {
		Ray toLight(x, light->position - x);

		payload.color += light->color * triangle->diffuse_color
			* std::max(0.0f, linalg::dot(normal, toLight.direction));

		float3 reflectionDir = 2.0f * linalg::dot(normal, toLight.direction) * normal - toLight.direction;
		payload.color += light->color * triangle->specular_color
			* std::powf(std::max(0.0f, linalg::dot(ray.direction, reflectionDir)), triangle->specular_exponent);
	}

	return payload;
}

float3 MaterialTriangle::GetNormal(float3 barycentric) const {
	if (linalg::length(a.normal) > 0.0f && linalg::length(b.normal) > 0.0f && linalg::length(c.normal) > 0.0f) {
		return a.normal * barycentric.x
			+ b.normal * barycentric.y
			+ c.normal * barycentric.z;
	}

	return geo_normal;
}
