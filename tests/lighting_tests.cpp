#define CATCH_CONFIG_MAIN
#define CATCH_CONFIG_ENABLE_BENCHMARKING
#include "catch.hpp"

#include "test_utils.h"

#include "lighting.h"

TEST_CASE("Lighting algorithm test") {
	Lighting* render = new Lighting(1920, 1080);
	int result = render->LoadGeometry("models/CornellBox-Original.obj");
	REQUIRE(result == 0);
	render->SetCamera(float3{ 0, 1.1f, 2 }, float3{ 0, 1, -1 }, float3{ 0, 1, 0 });
	render->AddLight(new Light(float3{ 0, 1.98f, -0.06f }, float3{ 0.78f, 0.78f, 0.78f }));
	render->Clear();

    BENCHMARK("Draw scene")
    {
        render->DrawScene();
    };

    REQUIRE(validate_framebuffer("references/lighting.png", render->GetFrameBuffer()));
}