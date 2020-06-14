workspace "iray"
    language "C++"
    cppdialect "C++17"

    location "build"

    buildoptions { "-std=c++2a", "-march=native" }

    configurations { "debug", "release" }

    filter { "configurations:debug" }
        symbols "On"
        targetdir "build/debug/bin"
        objdir "build/debug/obj"

    filter { "configurations:release" }
        optimize "On"
        targetdir "build/release/bin"
        objdir "build/release/obj"
        buildoptions { "-O3", "-Werror", "-Wextra", "-Wall", "-Wpedantic" }

    filter { }

project "lodepng"
    kind "StaticLib"
    files { "ext/lodepng/lodepng.cpp", "ext/lodepng/lodepng.h" }

project "iray"
    kind "ConsoleApp"
    files { "src/**.cpp", "src/**.hpp" }

    links { "lodepng", "pthread" }

    includedirs "src/"
    includedirs "ext/glm"
    includedirs "ext/lodepng"
