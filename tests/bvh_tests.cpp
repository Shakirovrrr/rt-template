#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include "catch.hpp"

#include "test_utils.h"

#include "bvh.h"

TEST_CASE("BVH test") {
    BVH* render = new BVH(1920, 1080);
    int result = render->LoadGeometry("models/CornellBox-Sphere.obj");
    REQUIRE(result == 0);
    render->SetCamera(float3{ 0.0f, 0.795f, 1.6f }, float3{ 0, 0.795f, -1 }, float3{ 0, 1, 0 });
    render->AddLight(new Light(float3{ 0, 1.58f, -0.03f }, float3{ 0.78f, 0.78f, 0.78f }));
    render->Clear();

    BENCHMARK("BVH scene")
    {
        render->DrawScene();
    };

    REQUIRE(validate_framebuffer("references/bvh.png", render->GetFrameBuffer()));
}