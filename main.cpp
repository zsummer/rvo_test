/*
* breeze License
* Copyright (C) 2014-2017 YaweiZhang <yawei.zhang@foxmail.com>.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
* http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/


#pragma warning(disable:4996)
#pragma warning(disable:4819)


#define WIN32_LEAN_AND_MEAN
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
#include <RVO.h>
#include <Agent.h>
#include <Obstacle.h>

#define SCREEN_X 800
#define SCREEN_Y 800

Point3 rgb(float r, float g, float b)
{
    return Point3(r / 255.0f, g / 255.0f, b / 255.0f);
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

void draw_circle(float wide, const Point3& color, const RVO::Vector2& center)
{
    constexpr const float V_COUNT = 80.0;
    wide = wide > 1.0f ? 1.0 : wide;
    glColor3f(color.x, color.y, color.z);
    glBegin(GL_POLYGON);
    for (int i = 0; i < V_COUNT; i++)
    {
        glVertex2f(center.x() + wide * cos(2 * NI_PI * i / V_COUNT), center.y() + wide * sin(2 * NI_PI * i / V_COUNT));
    }
    glEnd();
}

void draw_line(float wide, const Point3&color, const RVO::Vector2 &begin, const RVO::Vector2&end)
{
	glLineWidth(wide);
	glBegin(GL_LINE_STRIP);
	glColor3f(color.x, color.y, color.z);    // Red
	glVertex2f(begin.x(), begin.y());
	glVertex2f(end.x(), end.y());
	glEnd();
}


#ifndef M_PI
const float M_PI = 3.14159265358979323846f;
#endif
/* Create a new simulator instance. */
#ifdef USE_RVO_2_0
RVO::RVOSimulator* sim = new RVO::RVOSimulator();
#else
RVO::RVOSimulator* sim = RVO::RVOSimulator::Instance();
#endif
/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

void setupScenario(RVO::RVOSimulator* sim)
{
    

#ifdef USE_RVO_2_0
    /* Specify the default parameters for agents that are subsequently added. */
    sim->setAgentDefaults(100.0f, 15, sim->getTimeStep() * 20, sim->getTimeStep() * 10, 5.0f, 10.0f);
#else
    /* Specify the default parameters for agents that are subsequently added. */
    sim->setAgentDefaults(15, 100.0f, 15, 5.0f, 5.0f, 10.0f, 10.0f, 1.0f);
#endif
    /* Specify the global time step of the simulation. */
    sim->setTimeStep(0.01f);


    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */
#ifdef USE_RVO_2_0
    for (int i = 0; i < 1000; ++i)
    {
        RVO::Vector2 target = 200.0f * RVO::Vector2(std::cos(i * 2.0f * M_PI / 30.0f), std::sin(i * 2.0f * M_PI / 30.0f));
        if (rand()%5 == 0)
        {
            int id = sim->addAgent(target);
            sim->setAgentPosition(id, sim->getAgentPosition(id) / 2.0f);
            sim->setAgentRadius(id, 5 + rand() % 100 / 10.0f);
            sim->setAgentMaxSpeed(id, 0.0f);
            goals.push_back(-sim->getAgentPosition(id));
        }
        else if (rand()%3 == 0)
        {
            continue;
        }
        else
        {
            int id = sim->addAgent(target);
            sim->setAgentRadius(id, 5 + rand() % 100 / 10.0f);
            sim->setAgentMaxSpeed(id, sim->getAgentRadius(id) * 10.0f);
            goals.push_back(-sim->getAgentPosition(id));
        }
    }

    sim->addObstacle({ {0, 20}, {-20, 20}, {-20, 0}, {0, 0} });
    sim->addObstacle({ {100, -50},  {80, -50} });
    sim->addObstacle({ {-100, 100},  {-100, 120} });
#else
    for (int i = 0; i < 100; ++i)
    {
        RVO::Vector2 target = 200.0f * RVO::Vector2(std::cos(i * 2.0f * M_PI / 30.0f), std::sin(i * 2.0f * M_PI / 30.0f));
        if (rand() % 5 == 0)
        {
            int goal = sim->addGoal(-target);
            int id = sim->addAgent(target, goal);
            sim->setAgentPosition(id, sim->getAgentPosition(id) / 2.0f);
            sim->setAgentRadius(id, 5 + rand() % 100 / 10.0f);
            sim->setAgentMaxSpeed(id, 0.0f);
            goals.push_back(-sim->getAgentPosition(id));
        }
        else if (rand() % 3 == 0)
        {
            continue;
        }
        else
        {
            int goal = sim->addGoal(-target);
            int id = sim->addAgent(target, goal);
            sim->setAgentRadius(id, 5 + rand() % 100 / 10.0f);
            sim->setAgentMaxSpeed(id, sim->getAgentRadius(id) * 10.0f);
            goals.push_back(-sim->getAgentPosition(id));
        }
    }
    sim->initSimulation();
#endif
}

void setPreferredVelocities(RVO::RVOSimulator* sim)
{
#ifdef USE_RVO_2_0
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
        RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);
        
        if (RVO::abs(goalVector) > sim->getTimeStep() * sim->getAgentMaxSpeed(i)) 
        {
            goalVector = RVO::normalize(goalVector);
            sim->setAgentPrefVelocity(i, goalVector * sim->getAgentMaxSpeed(i) / 2.0f);
        }
        else
        {
            sim->setAgentPrefVelocity(i, RVO::Vector2());
        }
    }
#else
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) 
    {
        RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);
        sim->setAgentPrefSpeed(i, sim->getAgentMaxSpeed(i));
    }

#endif
}

bool reachedGoal(RVO::RVOSimulator* sim)
{
    /* Check if all agents have reached their goals. */
    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
#ifdef USE_RVO_2_0
        if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
            return false;
        }
#else
        if (absSq(sim->getAgentPosition(i) - goals[i]) > sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
            return false;
        }
#endif

    }

    return true;
}

void drawAgent(RVO::RVOSimulator* sim, double now)
{
    static double last = now;

    float interval = now - last;
    if (!reachedGoal(sim))
    {
        if (interval >= sim->getTimeStep())
        {
            setPreferredVelocities(sim);
            sim->doStep();
            last = now;
        }
    }

    static const float MAP_SIZE = 300.0f;

#ifdef USE_RVO_2_2
    for (RVO::Obstacle* obstacle : sim->obstacles_)
    {
        RVO::Obstacle* head = obstacle;
        while (head && head->nextObstacle_)
        {
            draw_circle(5.0f / MAP_SIZE, rgb(241, 210, 202), (head->point_ + head->unitDir_ * 5.0f) / MAP_SIZE);
            draw_line(3.0f, rgb(241, 110, 202), head->point_ / MAP_SIZE, head->nextObstacle_->point_ / MAP_SIZE);
            head = head->nextObstacle_;
            if (head == obstacle)
            {
                break;
            }
        }
    }


    for (int i = 0; i < sim->getNumAgents(); i++)
    {
        auto pos = sim->getAgentPosition(i) / MAP_SIZE;
        auto target = goals[i] / MAP_SIZE;
        auto radius = sim->getAgentRadius(i) / MAP_SIZE;

        auto ho = sim->getAgentTimeHorizon(i) / MAP_SIZE;
        auto hobst = sim->getAgentTimeHorizonObst(i) / MAP_SIZE;


        if (abs(sim->getAgentVelocity(i)) > 0.001)
        {
            draw_line(0.5f, rgb(248, 180, 228), pos, target);
        }
        

        for (int j = 0; j < 30; j++)
        {
            draw_line(0.7f, rgb(180, 228, 248), pos, pos +
                RVO::Vector2(std::cos(j * 2.0f * M_PI / 20.0f),
                    std::sin(j * 2.0f * M_PI / 20.0f))
                * (sim->getAgentTimeHorizon(i) * sim->getAgentMaxSpeed(i) + sim->getAgentRadius(i)) / MAP_SIZE);
        }

        for (int j = 0; j < 20; j++)
        {
            draw_line(1.4f, rgb(110, 202, 241), pos, pos +
                RVO::Vector2(std::cos(j * 2.0f * M_PI / 20.0f),
                    std::sin(j * 2.0f * M_PI / 20.0f))
                * (sim->getAgentTimeHorizonObst(i) * sim->getAgentMaxSpeed(i) + sim->getAgentRadius(i)) / MAP_SIZE);
        }

    }
#endif

    for (int i = 0; i < sim->getNumAgents(); i++)
    {
        auto pos = sim->getAgentPosition(i) / MAP_SIZE;
        auto radius = sim->getAgentRadius(i) / MAP_SIZE;
#ifdef USE_RVO_2_2
        auto ho = sim->getAgentTimeHorizon(i) / MAP_SIZE;
        auto hobst = sim->getAgentTimeHorizonObst(i) / MAP_SIZE;
#endif
        auto target = goals[i] / MAP_SIZE;
        
        /*
        for (int j = 0; j < 10; j++)
        {
            draw_line(1.0f, rgb(1,1,1), pos, 
                RVO::Vector2(std::cos(j * 2.0f * M_PI / 10.0f),
                    std::sin(j * 2.0f * M_PI / 10.0f))
                * (sim->getAgentTimeHorizonObst(i) * sim->getAgentMaxSpeed(i) + sim->getAgentRadius(i)) / MAP_SIZE);
        }
        */


        RVO::Vector2 dir = sim->getAgentVelocity(i);
        if (abs(dir) > 0.0001)
        {
            draw_circle(radius, rgb(40, 177, 234), pos);
            RVO::Vector2 vel = dir;
            draw_line(3.0,
                rgb(241, 110, 137),
                sim->getAgentPosition(i) / MAP_SIZE,
                (sim->getAgentPosition(i) + vel) / MAP_SIZE);
        }
        else
        {
            draw_circle(radius, rgb(241, 110, 202), pos);
        }

    }
    
}


int main(void)
{
#ifndef _WIN32
	//! linux下需要屏蔽的一些信号
	signal(SIGHUP, SIG_IGN);
	signal(SIGALRM, SIG_IGN);
	signal(SIGPIPE, SIG_IGN);
	signal(SIGXCPU, SIG_IGN);
	signal(SIGXFSZ, SIG_IGN);
	signal(SIGPROF, SIG_IGN);
	signal(SIGVTALRM, SIG_IGN);
	signal(SIGQUIT, SIG_IGN);
	signal(SIGCHLD, SIG_IGN);
	setenv("TZ", "GMT-8", 1);
#else
	//system("chcp 65001");
#endif
	srand((unsigned int)time(NULL));

    FNLog::FastStartDefaultLogger();
    setupScenario(sim);
	
    GLFWwindow * window;


    glfwSetErrorCallback(error_callback);

    if (!glfwInit())
        exit(EXIT_FAILURE);

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    glfwWindowHint(GLFW_SAMPLES, 8);
    
    window = glfwCreateWindow(SCREEN_X, SCREEN_Y, "Simple example", NULL, NULL);
    if (!window)
    {
        glfwTerminate();
        exit(EXIT_FAILURE);
    }

    glfwSetKeyCallback(window, key_callback);

    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);
    glfwSwapInterval(1);
    glEnable(GL_MULTISAMPLE);
    LOGI() << "GL_VERSION:" << (const char*)glGetString(GL_VERSION);

    while (!glfwWindowShouldClose(window))
    {


		glClear(GL_COLOR_BUFFER_BIT);
		glShadeModel(GL_SMOOTH);

        double begin_time = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
        static double last_time = begin_time;
        static double frame_count = 0;
        static double last_hit_count = 0;
        static double cur_hit_count = 0;
        static double last_test_count = 0;
        static double cur_test_count = 0;
        frame_count++;
        Vector3 dir = { 0.0f, 0.0f, 0.0f };
        double radian = (float)fmod(begin_time, 2.0f) / 2.0f * 3.141592654*2.0f;
        dir.x = cos(radian);
        dir.y = sin(radian);

        glColor3f(1.0f, 1.0f, 1.0f);
        glBegin(GL_POLYGON);
        glVertex2f(1.0f, 1.0f);
        glVertex2f(1.0f, -1.0f);
        glVertex2f(-1.0f, -1.0f);
        glVertex2f(-1.0f, 1.0f);
        glEnd();

        double now = std::chrono::duration_cast<std::chrono::duration<double>>(std::chrono::steady_clock::now().time_since_epoch()).count();
        drawAgent(sim, now);

		glfwSwapBuffers(window);
		glfwPollEvents();



        if (now - last_time  > 1.0f)
        {

            char title[100] = { 0 };
            sprintf(title, "specify type:<%u>, fps:<%lf>  lapse:<%lf>, test:<%lf>, hit:<%lf>", 0,
                frame_count / (now - last_time), now - begin_time,
                (cur_test_count - last_test_count) / (now - last_time),
                (cur_hit_count -last_hit_count) / (now - last_time));
            glfwSetWindowTitle(window, title);
            LOGD() << title;
            last_time = now;
            last_test_count = cur_test_count;
            last_hit_count = cur_hit_count;
            frame_count = 0.0f;

        }
    }

    glfwDestroyWindow(window);
    glGetError();
    glfwTerminate();
    exit(EXIT_SUCCESS);
}

void example()
{
    /* Draw a triangle */
    glBegin(GL_TRIANGLES);

    glColor3f(1.0f, 0.0f, 0.0f);    // Red
    glVertex3f(0.0f, 1.0f, 0.0f);

    glColor3f(0.0f, 1.0f, 0.0f);    // Green
    glVertex3f(-1.0f, -1.0f, 0.0f);

    glColor3f(0.0f, 0.0f, 1.0);    // Blue
    glVertex3f(1.0f, -1.0f, 0.0f);

    glEnd();


    //glEnable(GL_LINE_STIPPLE);
    //glLineStipple(1, 1);
    glColor3f(1.0f, 1.0f, 1.0f);    // Red
    glLineWidth(1);


    glBegin(GL_LINES);
    glVertex2f(0.0f, 0.0f);
    glColor3f(1.0f, 0.0f, 1.0f);    // Red
    glVertex2f(1, 1);
    glEnd();


    glBegin(GL_LINES);
    glVertex3f(0.0f, 1.0f, 0.0f);
    glColor3f(0.0f, 0.0f, 1.0f);    // Red
    glVertex3f(1, 0.0f, 1.0f);
    glEnd();


    glBegin(GL_POINTS);
    glColor3f(0.0f, 1.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glEnd();
}