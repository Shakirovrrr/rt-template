#include "aabb.h"

//#define TINYOBJLOADER_IMPLEMENTATION
#include "tiny_obj_loader.h"

AABB::AABB(short width, short height) :AntiAliasing(width, height) {}

AABB::~AABB() {}

int AABB::LoadGeometry(std::string filename) {
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
		Mesh mesh;

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

			MaterialTriangle triangle(vertices[0], vertices[1], vertices[2]);

			int materialId = shapes[s].mesh.material_ids[f];
			tinyobj::material_t material = materials[materialId];
			triangle.SetEmisive(float3 {material.emission});
			triangle.SetAmbient(float3 {material.ambient});
			triangle.SetDiffuse(float3 {material.diffuse});
			triangle.SetSpecular(float3 {material.specular}, material.shininess);
			triangle.SetReflectiveness(material.illum == 5);
			triangle.SetReflectivenessAndTransparency(material.illum == 7);
			triangle.SetIor(material.ior);

			mesh.AddTriangle(triangle);

			// material_objects.push_back(triangle);

			// per-face material
			// shapes[s].mesh.material_ids[f];
		}

		meshes.push_back(mesh);
	}

	return 0;
}

Payload AABB::TraceRay(const Ray &ray, const unsigned int max_raytrace_depth) const {
	if (max_raytrace_depth <= 0) {
		return Miss(ray);
	}

	IntersectableData closestData(t_max);
	MaterialTriangle closestTriangle;

	for (auto &mesh : meshes) {
		if (!mesh.AABBTest(ray)) {
			continue;
		}

		for (auto &object : mesh.Triangles()) {
			IntersectableData data = object.Intersect(ray);
			if (data.t < closestData.t && data.t > t_min) {
				closestData = data;
				closestTriangle = object;
			}
		}
	}

	if (closestData.t < t_max) {
		return Hit(ray, closestData, &closestTriangle, max_raytrace_depth);
	}

	return Miss(ray);
}

float AABB::TraceShadowRay(const Ray &ray, const float max_t) const {
	IntersectableData closestData(max_t);
	for (auto &mesh : meshes) {
		if (!mesh.AABBTest(ray)) {
			continue;
		}

		for (auto &object : mesh.Triangles()) {
			IntersectableData data = object.Intersect(ray);
			if (data.t < closestData.t && data.t > t_min) {
				return data.t;
			}
		}
	}

	return max_t;
}

void Mesh::AddTriangle(const MaterialTriangle triangle) {
	if (triangles.empty()) {
		aabb_max = aabb_min = triangle.a.position;
	}
	triangles.push_back(triangle);

	aabb_max = linalg::max(triangle.a.position, aabb_max);
	aabb_max = linalg::max(triangle.b.position, aabb_max);
	aabb_max = linalg::max(triangle.c.position, aabb_max);

	aabb_min = linalg::min(triangle.a.position, aabb_min);
	aabb_min = linalg::min(triangle.b.position, aabb_min);
	aabb_min = linalg::min(triangle.c.position, aabb_min);
}

bool Mesh::AABBTest(const Ray &ray) const {
	float3 invRaydir = float3(1.0) / ray.direction;
	float3 t0 = (aabb_max - ray.position) * invRaydir;
	float3 t1 = (aabb_min - ray.position) * invRaydir;
	float3 tmin = linalg::min(t0, t1);
	float3 tmax = linalg::max(t0, t1);
	return linalg::maxelem(tmin) <= linalg::minelem(tmax);
}
