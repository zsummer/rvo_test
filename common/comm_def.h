#pragma once

#ifndef COMM_DEF_H
#define COMM_DEF_H
#pragma warning(disable:4996)
#pragma warning(disable:4819)


#define WIN32_LEAN_AND_MEAN
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include <limits>
#include <string>
#include <vector>
#include <map>
#include <memory>
#include <list>
#include <array>
#include "fn_log.h"



#include <WS2tcpip.h>
#include <WinSock2.h>
#include <windows.h>
#include <io.h>
#include <shlwapi.h>
#include <process.h>
#include <direct.h>
#include <glad/glad.h>
#include <gl/GL.h>


#include <iomanip>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <cstdint>

#include <iostream>
#include <fstream>
#include <sstream>
#include <utility>
#include <algorithm>
#include <limits>

#include <functional>
#include <memory>
#include <unordered_map>
#include <chrono>
#include <random>

#include <string>
#include <set>
#include <vector>
#include <list>
#include <map>
#include <array>



#include "fn_log.h"


#include <vector>
#include <glad/glad.h>
#include <gl/GL.h>
#include <gl/GLU.h>

#include "glfw3.h"
#include "glfw3native.h"
#include "linmath.h"

#include <stdlib.h>
#include <stdio.h>


#pragma comment(lib, "OpenGL32")
#pragma comment(lib, "GLu32")

#include "vector3.h"
namespace RVO
{
    class Vector2;
}

typedef char s8;
typedef unsigned char u8;
typedef short int s16;
typedef unsigned short int u16;
typedef int s32;
typedef unsigned int u32;
typedef long long s64;
typedef unsigned long long u64;
typedef unsigned long pointer;
typedef unsigned int qq_t;
typedef int BOOL;
typedef float f32;

inline void empty_test(...) {}

using Point3 = Vector3<float>;
static std::tuple<float, float, float> rgb(float r, float g, float b)
{
    return { r / 255.0f, g / 255.0f, b / 255.0f };
}

static void error_callback(int error, const char* description)
{
    fprintf(stderr, "Error: %s\n", description);
}

static void key_callback(GLFWwindow* window, int key, int scancode, int action, int mods)
{
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

//color 0~1.0  
//pos {-1.0f, -1.0f} -> {1.0f, 1.0f}
static void draw_circle(float wide, const std::tuple<float, float, float>& color, float pos_x, float pos_y)
{
    constexpr const float V_COUNT = 80.0;
    wide = wide > 1.0f ? 1.0 : wide;
    glColor3f(std::get<0>(color), std::get<1>(color), std::get<2>(color));
    glBegin(GL_POLYGON);
    for (int i = 0; i < V_COUNT; i++)
    {
        glVertex2f(pos_x + wide * cos(2 * NI_PI * i / V_COUNT), pos_y + wide * sin(2 * NI_PI * i / V_COUNT));
    }
    glEnd();
}

static void draw_circle(float wide, const std::tuple<float, float, float>& color,  std::tuple<float, float> pos)
{
    draw_circle(wide, color, std::get<0>(pos), std::get<1>(pos));
}


//color 0~1.0  
//pos {-1.0f, -1.0f} -> {1.0f, 1.0f}
static void draw_line(float wide, const std::tuple<float, float, float>& color, float begin_x, float begin_y, float end_x, float end_y)
{
    glLineWidth(wide);
    glBegin(GL_LINE_STRIP);
    glColor3f(std::get<0>(color), std::get<1>(color), std::get<2>(color));    // Red
    glVertex2f(begin_x, begin_y);
    glVertex2f(end_x, end_y);
    glEnd();
}

static void draw_line(float wide, const std::tuple<float, float, float>& color, std::tuple<float, float> begin, std::tuple<float, float> end)
{
    draw_line(wide, color, std::get<0>(begin), std::get<1>(begin), std::get<0>(end), std::get<1>(end));
}







#endif // COMM_DEF_H