workspace "Basics of ray tracing"
   configurations { "Debug", "Release" }
   language "C++"
   architecture "x64"
   systemversion "latest"
   toolset "v142"
   optimize "Speed"
   buildoptions { "/openmp" }
   filter "configurations:Debug"
      defines { "DEBUG" }
      symbols "On"

   filter "configurations:Release"
      defines { "NDEBUG" }
      optimize "On"

   targetdir ("bin/%{prj.name}/%{cfg.longname}")
   objdir ("obj/%{prj.name}/%{cfg.longname}")
    
group "01. Ray generation"
   project "Ray generation lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
   
   project "Ray generation app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "Ray generation lib"
      files { "src/ray_generation_main.cpp" }
   
   project "Ray generation tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "Ray generation lib"
      debugargs { "--benchmark-samples", "25" }
      files {"tests/ray_generation_tests.cpp"}


group "02. Moller-Trumbore algorithm"
   project "Moller-Trumbore algorithm lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
   
   project "Moller-Trumbore algorithm app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "Moller-Trumbore algorithm lib"
      files { "src/mt_algorithm_main.cpp" }
   
   project "Moller-Trumbore algorithm tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "Moller-Trumbore algorithm lib"
      debugargs { "--benchmark-samples", "25" }
      files {"tests/mt_algorithm_tests.cpp"}

group "03. Lighting"
   project "Lighting lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
   
   project "Lighting app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "Lighting lib"
      files { "src/lighting_main.cpp" }
   
   project "Lighting tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "Lighting lib"
      debugargs { "--benchmark-samples", "25" }
      files {"tests/lighting_tests.cpp"}

group "04. Shadow rays"
   project "ShadowRays lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
      files {"src/shadow_rays.h", "src/shadow_rays.cpp"}

   project "ShadowRays app"
     kind "ConsoleApp"
     includedirs { "lib/linalg" }
     includedirs { "src" }
     links "ShadowRays lib"
     files { "src/shadow_rays_main.cpp" }

   project "ShadowRays tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "ShadowRays lib"
      debugargs { "--benchmark-samples", "25" }
      files {"tests/shadow_rays_tests.cpp"}

group "05. Reflection"
   project "Reflection lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
      files {"src/shadow_rays.h", "src/shadow_rays.cpp"}
      files {"src/reflection.h", "src/reflection.cpp"}
   
   project "Reflection app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "Reflection lib"
      files { "src/reflection_main.cpp" }
   
   project "Reflection tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "Reflection lib"
      debugargs { "--benchmark-samples", "25" }
      files {"tests/reflection_tests.cpp"}

group "06. Refraction"
   project "Refraction lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
      files {"src/shadow_rays.h", "src/shadow_rays.cpp"}
      files {"src/reflection.h", "src/reflection.cpp"}
      files {"src/refraction.h", "src/refraction.cpp"}
      
   project "Refraction app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "Refraction lib"
      files { "src/refraction_main.cpp" }
   
   project "Refraction tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "Refraction lib"
      debugargs { "--benchmark-samples", "5" }
      files {"tests/refraction_tests.cpp"}

group "07. Anti-aliasing"
   project "AntiAliasing lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
      files {"src/shadow_rays.h", "src/shadow_rays.cpp"}
      files {"src/reflection.h", "src/reflection.cpp"}
      files {"src/refraction.h", "src/refraction.cpp"}
      files {"src/anti_aliasing.h", "src/anti_aliasing.cpp"}
      
   project "AntiAliasing app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "AntiAliasing lib"
      files { "src/anti_aliasing_main.cpp" }
   
   project "AntiAliasing tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "AntiAliasing lib"
      debugargs { "--benchmark-samples", "25" }
      files {"tests/anti_aliasing_tests.cpp"}

group "08. AABB"
   project "AABB lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
      files {"src/shadow_rays.h", "src/shadow_rays.cpp"}
      files {"src/reflection.h", "src/reflection.cpp"}
      files {"src/refraction.h", "src/refraction.cpp"}
      files {"src/anti_aliasing.h", "src/anti_aliasing.cpp"}
      files {"src/aabb.h", "src/aabb.cpp"}
      
   project "AABB app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "AABB lib"
      files { "src/aabb_main.cpp" }
   
   project "AABB tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "AABB lib"
      debugargs { "--benchmark-samples", "5" }
      files {"tests/aabb_tests.cpp"}

group "09. BVH"
   project "BVH lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
      files {"src/shadow_rays.h", "src/shadow_rays.cpp"}
      files {"src/reflection.h", "src/reflection.cpp"}
      files {"src/refraction.h", "src/refraction.cpp"}
      files {"src/anti_aliasing.h", "src/anti_aliasing.cpp"}
      files {"src/aabb.h", "src/aabb.cpp"}
      files {"src/bvh.h", "src/bvh.cpp"}
      
   project "BVH app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "BVH lib"
      files { "src/bvh_main.cpp" }
   
   project "BVH tests"
      kind "ConsoleApp"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/catch2/single_include/catch2" }
      includedirs { "src" }
      files { "tests/test_utils.h" }
      links "BVH lib"
      debugargs { "--benchmark-samples", "25" }
      files {"tests/bvh_tests.cpp"}

group "10. Denoising"
   project "Denoising lib"
      kind "StaticLib"
      includedirs { "lib/stb" }
      includedirs { "lib/linalg" }
      includedirs { "lib/tinyobjloader" }
      includedirs { "src/" }
      files {"src/ray_generation.h", "src/ray_generation.cpp" }
      files {"src/mt_algorithm.h", "src/mt_algorithm.cpp"}
      files {"src/lighting.h", "src/lighting.cpp"}
      files {"src/shadow_rays.h", "src/shadow_rays.cpp"}
      files {"src/reflection.h", "src/reflection.cpp"}
      files {"src/refraction.h", "src/refraction.cpp"}
      files {"src/anti_aliasing.h", "src/anti_aliasing.cpp"}
      files {"src/aabb.h", "src/aabb.cpp"}
      files {"src/bvh.h", "src/bvh.cpp"}
      files {"src/denoising.h", "src/denoising.cpp"}
      
   project "Denoising app"
      kind "ConsoleApp"
      includedirs { "lib/linalg" }
      includedirs { "src" }
      links "Denoising lib"
      files { "src/denoising_main.cpp" }