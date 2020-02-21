#include "denoising.h"
#include "denoising_main.h"

int main(int argc, char* argv[])
{
	Denoising* render = new Denoising(1920, 1080);
	int result = render->LoadGeometry("models/CornellBox-Mirror.obj");
	if (result)
	{
		return result;
	}
	render->SetCamera(float3{ -0.5f, 0.99f, 1.5f }, float3{ 0, 0.99f, -1 }, float3{ 0, 1, 0 });
	render->LoadBlueNoise("textures/blue-noise.png");
	render->Clear();
	render->DrawScene(1024);
	result = render->Save("results/denoising.png");
	return result;
}