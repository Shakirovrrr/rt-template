#include "mt_algorithm.h"

int main(int argc, char* argv[])
{
    MTAlgorithm* render = new MTAlgorithm(1920, 1080);
    render->LoadGeometry("");
    render->SetCamera(float3{ 1, 0, 1 }, float3{ 1, 0, 0 }, float3{ 0, 1, 0 });
    render->Clear();
    render->DrawScene();
    int result = render->Save("results/mt_algorithm.png");
    return result;
}