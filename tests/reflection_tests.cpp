#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include "catch.hpp"

#include "test_utils.h"

#include "reflection.h"

TEST_CASE("Reflection algorithm test") {
    Reflection* render = new Reflection(1920, 1080);
    int result = render->LoadGeometry("models/CornellBox-Mirror.obj");
    REQUIRE(result == 0);
    render->SetCamera(float3{ -0.5f, 0.99f, 1.5f }, float3{ 0, 0.99f, -1 }, float3{ 0, 1, 0 });
    render->AddLight(new Light(float3{ 0, 1.98f, -0.06f }, float3{ 0.78f, 0.78f, 0.78f }));
    render->Clear();

    BENCHMARK("Draw scene")
    {
        render->DrawScene();
    };

    REQUIRE(validate_framebuffer("references/reflection.png", render->GetFrameBuffer()));
}