#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include "catch.hpp"

#include "test_utils.h"

#include "ray_generation.h"

TEST_CASE("Camera tests") {
    Camera camera;
    camera.SetRenderTargetSize(2, 2);
    camera.SetPosition(float3{ 0, 0, 0 });
    camera.SetDirection(float3{ 0, 0, 2 });
    camera.SetUp(float3{ 0, 3, 0 });

    Ray ray = camera.GetCameraRay(0, 0);

    CHECK(ray.position == float3{ 0, 0, 0 });
    CHECK(ray.direction == normalize(float3{ 0.5, 0.5, 1 }));

    ray = camera.GetCameraRay(1, 0);

    CHECK(ray.position == float3{ 0, 0, 0 });
    CHECK(ray.direction == normalize(float3{ -0.5, 0.5, 1 }));

    ray = camera.GetCameraRay(0, 1);

    CHECK(ray.position == float3{ 0, 0, 0 });
    CHECK(ray.direction == normalize(float3{ 0.5, -0.5, 1 }));

    ray = camera.GetCameraRay(1, 1);

    CHECK(ray.position == float3{ 0, 0, 0 });
    CHECK(ray.direction == normalize(float3{ -0.5, -0.5, 1 }));
}

TEST_CASE("Ray generation test") {
    RayGenerationApp* render = new RayGenerationApp(1920, 1080);

    render->SetCamera(float3{ 0, 0, 0 }, float3{ 0, 0, -5 }, float3{ 0, 1, 0 });
    render->Clear();

    BENCHMARK("Draw scene")
    {
        render->DrawScene();
    };

    REQUIRE(validate_framebuffer("references/ray_generation.png", render->GetFrameBuffer()));
}