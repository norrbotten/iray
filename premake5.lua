workspace "iray"

    language "C++"
    cppdialect "C++17"

    location "build"

    buildoptions { "-std=c++2a" }

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

function includeGLM()
    includedirs "ext/glm"
end

project "iray"
    kind "ConsoleApp"
    files { "src/**.cpp", "src/**.hpp" }
    includedirs "src/"

    includeGLM()
