#include "ray_generation.h"

int main(int argc, char* argv[])
{
    RayGenerationApp* render = new RayGenerationApp(1920, 1080);
    render->SetCamera(float3{ 0, 0, 0 }, float3{ 0, 0, -5 }, float3{ 0, 1, 0 });
    render->Clear();
    render->DrawScene();
    int result = render->Save("results/ray_generation.png");
    return result;
}