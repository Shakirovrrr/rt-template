#include "refraction.h"

int main(int argc, char* argv[])
{
	Refraction* render = new Refraction(1920, 1080);
	int result = render->LoadGeometry("models/CornellBox-Sphere.obj");
	if (result)
	{
		return result;
	}
	render->SetCamera(float3{ 0.0f, 0.795f, 1.6f }, float3{ 0, 0.795f, -1 }, float3{ 0, 1, 0 });
	render->AddLight(new Light(float3{ 0, 1.58f, -0.03f }, float3{ 0.78f, 0.78f, 0.78f }));
	render->Clear();
	render->DrawScene();
	result = render->Save("results/refraction.png");
	return result;
}