#include "anti_aliasing.h"

int main(int argc, char* argv[])
{
	AntiAliasing* render = new AntiAliasing(1920, 1080);
	int result = render->LoadGeometry("models/CornellBox-Mirror.obj");
	if (result)
	{
		return result;
	}
	render->SetCamera(float3{ -0.5f, 0.99f, 1.5f }, float3{ 0, 0.99f, -1 }, float3{ 0, 1, 0 });
	render->AddLight(new Light(float3{ 0, 1.98f, -0.06f }, float3{ 0.78f, 0.78f, 0.78f }));
	render->Clear();
	render->DrawScene();
	result = render->Save("results/anti_aliasing.png");
	return result;
}