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

#include "comm_def.h"


#include <RVO.h>
#include <Agent.h>
#include <Obstacle.h>

#define SCREEN_X 800
#define SCREEN_Y 800


static void draw_line(float wide, const std::tuple<float, float, float>& color, RVO::Vector2 begin, RVO::Vector2 end)
{
    draw_line(wide, color, std::make_tuple(begin.x(), begin.y()), std::make_tuple(end.x(), end.y()));
}

static void draw_circle(float wide, const std::tuple<float, float, float>& color, RVO::Vector2 pos)
{
    draw_circle(wide, color, std::make_tuple(pos.x(), pos.y()));
}

/* Create a new simulator instance. */

RVO::RVOSimulator* sim = new RVO::RVOSimulator();

/* Store the goals of the agents. */
std::vector<RVO::Vector2> goals;

void setupScenario(RVO::RVOSimulator* sim)
{
    /* Specify the default parameters for agents that are subsequently added. */
    sim->setAgentDefaults(50.0f, 15, sim->getTimeStep() * 20, sim->getTimeStep() * 10, 5.0f, 10.0f);
    /* Specify the global time step of the simulation. */
    sim->setTimeStep(0.01f);


    /*
     * Add agents, specifying their start position, and store their goals on the
     * opposite side of the environment.
     */

    for (int i = 0; i < 10; ++i)
    {
        RVO::Vector2 src_pos = 200.0f * RVO::Vector2(std::cos(i * 2.0f * M_PI / 30.0f), std::sin(i * 2.0f * M_PI / 30.0f));
        if (rand()%5 == 0 && i!=0)
        {
            int id = sim->addAgent(src_pos /2.0f);
            sim->setAgentRadius(id, 5 + rand() % 100 / 10.0f);
            sim->setAgentMaxSpeed(id, 0.0f);
            goals.push_back(-sim->getAgentPosition(id));
        }
        else if (rand()%3 == 0 && i != 0)
        {
            continue;
        }
        else
        {
            int id = sim->addAgent(src_pos);
            sim->setAgentRadius(id, 5 + rand() % 100 / 10.0f);
            sim->setAgentMaxSpeed(id, sim->getAgentRadius(id) * 10.0f);
            goals.push_back(-sim->getAgentPosition(id));
        }
    }

    sim->addObstacle({ {0, 20}, {-20, 20}, {-20, 0}, {0, 0} });
    //sim->addObstacle({ {100, -50},  {80, -50} });
   // sim->addObstacle({ {-100, 100},  {-100, 120} });

}


void setPreferredVelocities(RVO::RVOSimulator* sim)
{
    for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i)
    {
        RVO::Vector2 goalVector = goals[i] - sim->getAgentPosition(i);
        if (RVO::abs(goalVector) > sim->getTimeStep() * sim->getAgentMaxSpeed(i))
        {
            goalVector = RVO::normalize(goalVector);
            sim->setAgentPrefVelocity(i, goalVector * sim->getAgentMaxSpeed(i) / 2.0f);
        }
        else
        {
            sim->setAgentPrefVelocity(i, { 0.0f, 0.0f });
        }
    }

}

bool reachedGoal(RVO::RVOSimulator* sim)
{
    /* Check if all agents have reached their goals. */
    for (size_t i = 0; i < sim->getNumAgents(); ++i)
    {
        if (RVO::absSq(sim->getAgentPosition(i) - goals[i]) > sim->getAgentRadius(i) * sim->getAgentRadius(i)) {
            return false;
        }
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

    //障碍 
    for (RVO::Obstacle* obstacle : sim->obstacles_)
    {
        RVO::Obstacle* head = obstacle;
        while (head && head->nextObstacle_)
        {
            draw_circle(5.0f / MAP_SIZE, rgb(241, 210, 202), (head->point_ + head->unitDir_ * 5.0f) / MAP_SIZE);
            draw_line(3.0f, rgb(8, 8, 8), head->point_ / MAP_SIZE, head->nextObstacle_->point_ / MAP_SIZE);
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


        for (int j = 0; j < 1; j++)
        {
            draw_line(0.3f, rgb(180, 228, 248), pos, pos +
                RVO::Vector2(std::cos(j * 2.0f * M_PI / 20.0f),
                    std::sin(j * 2.0f * M_PI / 20.0f))
                * (sim->getAgentTimeHorizon(i) * sim->getAgentMaxSpeed(i) + sim->getAgentRadius(i)) / MAP_SIZE);
        }

        for (int j = 0; j < 1; j++)
        {
            draw_line(0.4f, rgb(110, 202, 241), pos, pos +
                RVO::Vector2(std::cos(j * 2.0f * M_PI / 20.0f),
                    std::sin(j * 2.0f * M_PI / 20.0f))
                * (sim->getAgentTimeHorizonObst(i) * sim->getAgentMaxSpeed(i) + sim->getAgentRadius(i)) / MAP_SIZE);
        }

    }

    for (int i = 0; i < sim->getNumAgents(); i++)
    {
        auto pos = sim->getAgentPosition(i) / MAP_SIZE;
        auto radius = sim->getAgentRadius(i) / MAP_SIZE;

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
        if (i == 0)
        {
            draw_circle(radius / 3.0f, rgb(40, 77, 234), pos);
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
    LogInfo() << "GL_VERSION:" << (const char*)glGetString(GL_VERSION);

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
        Point3 dir = { 0.0f, 0.0f, 0.0f };
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
            LogDebug() << title;
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