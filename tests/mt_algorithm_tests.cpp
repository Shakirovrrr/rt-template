#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include "catch.hpp"

#include "test_utils.h"

#include "mt_algorithm.h"

TEST_CASE("Moller-Trumbore algorithm test") {
    MTAlgorithm* render = new MTAlgorithm(1920, 1080);
    render->LoadGeometry("");
    render->SetCamera(float3{ 1, 0, 1 }, float3{ 1, 0, 0 }, float3{ 0, 1, 0 });
    render->Clear();

    BENCHMARK("Draw scene")
    {
        render->DrawScene();
    };

    REQUIRE(validate_framebuffer("references/mt_algorithm.png", render->GetFrameBuffer()));
}