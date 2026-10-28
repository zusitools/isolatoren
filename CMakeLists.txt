cmake_minimum_required(VERSION 3.20)
project(IsolatorFixer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(FetchContent)

FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG 1.0.1
)
FetchContent_MakeAvailable(glm)

FetchContent_Declare(
    pugixml
    GIT_REPOSITORY https://github.com/zeux/pugixml.git
    GIT_TAG v1.14
)
FetchContent_MakeAvailable(pugixml)

set(wxWidgets_USE_STATIC 1)
set(wxBUILD_SHARED OFF)

include(FetchContent)
FetchContent_Declare(
   wxWidgets
   GIT_REPOSITORY https://github.com/wxWidgets/wxWidgets.git
   GIT_TAG v3.2.1
)
FetchContent_MakeAvailable(wxwidgets)

add_executable(isolator_fixer main.cpp)
target_link_libraries(isolator_fixer
    PRIVATE
        glm::glm
        pugixml::pugixml
        wx::base wx::core
)
target_include_directories(isolator_fixer
    PRIVATE
        parser/include
        ${wxWidgets_SOURCE_DIR}/include)
target_compile_definitions(isolator_fixer
    PRIVATE
        GLM_ENABLE_EXPERIMENTAL)
