/*
* kdtree License
* Copyright (C) 2019 YaweiZhang <yawei.zhang@foxmail.com>.
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


#pragma once

#include <vector3.h>
#include <functional>

using DrawLine = 
std::function<void(float /*line_wide*/, const Point3&/*color*/, const Point3&/*begin*/, const Point3&/*end*/)>;

struct Agent
{
	int id = 0;
	Vector3 pos;
	struct Agent * next = nullptr;
	struct Agent * front = nullptr;
};


struct KDNode
{
	Agent * agent = nullptr;
	int agent_count = 0;
	struct KDNode * parent = nullptr;
	struct KDNode * left = nullptr;
	struct KDNode * right = nullptr;
	int dim = 0;
	Vector3 pos;
	Vector3 vct;
};


class KDTree
{
public:
	using Node = KDNode;
	static const int SPLITE_THRESHOLD = 10;
public:
	Node* head_ = nullptr;
    int node_count_ = 0;
	int depth_ = 0;
	int agent_count_ = 0;

	Point3 world_pos_;
	Point3 world_vct_;
public:
	KDTree(const Point3& pos,  const Point3& vct):world_pos_(pos), world_vct_(vct)
	{
	}
	~KDTree()
	{
		clear();
	}
	void clear(Node* node, int depth);
	void clear()
	{
		LOGD() << "clear kd-tree";
		clear(head_, 0);
		head_ = nullptr;
		node_count_ = 0;
		depth_ = 0;
		agent_count_ = 0;
	}



	void view(Node* node, Point3 color, float line_wide, const DrawLine& drawline);

	void view(const DrawLine& drawline)
	{
		drawline(3, { 1.0f, 1.00f, 1.0f }, { -0.99, -0.99, 0.0f }, { 0.99, -0.99, 0.0f });
		drawline(3, { 1.0f, 1.00f, 1.0f }, { 0.99, -0.99, 0.0f }, { 0.99, 0.99, 0.0f });
		drawline(3, { 1.0f, 1.00f, 1.0f }, { 0.99, 0.99, 0.0f }, { -0.99, 0.99, 0.0f });
		drawline(3, { 1.0f, 1.00f, 1.0f }, { -0.99, 0.99, 0.0f }, { -0.99, -0.99, 0.0f });
		view(head_, { 1.0f,1.0f,1.0f }, 4.0f, drawline);
	}

	void agent_splitter(Node& parrent)
	{
		while (parrent.agent != NULL)
		{
			Agent* next = parrent.agent->next;
			Agent* agent = parrent.agent;

			//into left child
			if ((parrent.dim == 0 && agent->pos.x < parrent.right->pos.x)
				|| (parrent.dim != 0 && agent->pos.y < parrent.right->pos.y))
			{
				if (parrent.left->agent == NULL)
				{
					agent->next = nullptr;
				}
				else
				{
					agent->next = parrent.left->agent;
					agent->next->front = agent;
				}
				parrent.left->agent = agent;
				parrent.left->agent_count++;

				agent->front = nullptr;
				parrent.agent = next;
				parrent.agent_count--;
				continue;
			}
			//into right child  
			if (parrent.right->agent == NULL)
			{
				agent->next = nullptr;
			}
			else
			{
				agent->next = parrent.right->agent;
				agent->next->front = agent;
			}
			parrent.right->agent = agent;
			parrent.right->agent_count++;

			agent->front = nullptr;
			parrent.agent = next;
			parrent.agent_count--;
			continue;
		}
	}

	Node* get_node(const Vector3& pos, Node* node)
	{
		if (node->right == NULL && node->left == NULL)
		{
			return node;
		}
		if (node->right == NULL || node->left == NULL)
		{
			LOGE()<< "node only has one child. ";
			return NULL;
		}
		if (pos.x < node->pos.x || pos.x >= node->pos.x + node->vct.x)
		{
			LOGE() << "out node";
		}
		if (pos.y < node->pos.y || pos.y >= node->pos.y + node->vct.y)
		{
			LOGE() << "out node";
		}
		if (node->dim == 0)
		{
			if (pos.x >= node->left->pos.x && pos.x < node->right->pos.x)
			{
				return get_node(pos, node->left);
			}
			return get_node(pos, node->right);
		}

		if (pos.y >= node->left->pos.y && pos.y < node->right->pos.y)
		{
			return get_node(pos, node->left);
		}
		return get_node(pos, node->right);
	}

	Node* get_node(const Vector3& pos)
	{
		if (head_ == NULL)
		{
			head_ = new Node();
			head_->pos = world_pos_;
			head_->vct = world_vct_;
			node_count_++;
		}
		if (pos.x < head_->pos.x 
			|| pos.x > head_->pos.x + head_->vct.x
			|| pos.y < head_->pos.y 
			|| pos.y > head_->pos.y + head_->vct.y)
		{
			LOGE() << "out map.";
			return NULL;
		}

		return get_node(pos, head_); //递归实现 
	}


	Agent* insert(const Agent& in_agent)
	{
		Node* node = get_node(in_agent.pos);
		if (node == NULL)
		{
			LOGE() << "insert error";
			return NULL;
		}

		Agent* new_agent = new Agent(in_agent);
		if (node->agent == NULL)
		{
			node->agent = new_agent;
			node->agent_count = 1;
			agent_count_++;
			return node->agent;
		}

		new_agent->next = node->agent;
		new_agent->next->front = new_agent;
		node->agent = new_agent;
		node->agent_count++;
		agent_count_++;
		
		if (node->agent_count < SPLITE_THRESHOLD)
		{
			return new_agent;
		}

		float x_total = 0.0f;
		float y_total = 0.0f;
		Agent* agent = node->agent;
		while (agent)
		{
			x_total += agent->pos.x;
			y_total += agent->pos.y;
			agent = agent->next;
		}
		float x_avg = x_total / node->agent_count;
		float y_avg = y_total / node->agent_count;
		float x_variegated = 0.0f;
		float y_variegated = 0.0f;
		agent = node->agent;
		while (agent)
		{
			x_variegated += (agent->pos.x - x_avg)*(agent->pos.x - x_avg);
			y_variegated += (agent->pos.y - y_avg)*(agent->pos.y - y_avg);
			agent = agent->next;
		}

		if (x_avg < node->pos.x || x_avg >= node->pos.x + node->vct.x)
		{
			LOGE() << "out node";
		}

		if (y_avg < node->pos.y || y_avg >= node->pos.y + node->vct.y)
		{
			LOGE() << "out node";
		}

//		if (node->parent == NULL || node->parent->dim == 1)
//		{
//			float half_x = node->vct.x / 2.0f;
		if (x_variegated > y_variegated)
		{
			float half_x = x_avg - node->pos.x;
			

			node->dim = 0;
			node->left = new Node();
			node_count_++;
			node->left->parent = node;
			node->left->pos = node->pos;
			node->left->pos.x += 0;
			node->left->vct = node->vct;
			node->left->vct.x = half_x;

			node->right = new Node();
			node_count_++;
			node->right->parent = node;
			node->right->pos = node->pos;
			node->right->pos.x += half_x;
			node->right->vct = node->vct;
			node->right->vct.x = node->vct.x - half_x;
		}
		else
		{
			//			float half_y = node->vct.y / 2.0f;
			float half_y = y_avg - node->pos.y;


			node->dim = 1;
			node->left = new Node();
			node_count_++;
			node->left->parent = node;
			node->left->pos = node->pos;
			node->left->pos.y += 0;
			node->left->vct = node->vct;
			node->left->vct.y = half_y;

			node->right = new Node();
			node_count_++;
			node->right->parent = node;
			node->right->pos = node->pos;
			node->right->pos.y += half_y;
			node->right->vct = node->vct;
			node->right->vct.y = node->vct.y - half_y;
		}

		agent_splitter(*node);
		return new_agent;
	}

	float get_shortest_dist(Node* node, Point3 pos)
	{
		bool xin = pos.x > node->pos.x && pos.x < node->pos.x + node->vct.x;
		bool yin = pos.y > node->pos.y && pos.y < node->pos.y + node->vct.y;
		float xdist = min(fabsf(pos.x - node->pos.x), fabsf(pos.x - node->pos.x + node->vct.x));
		float ydist = min(fabsf(pos.y - node->pos.y), fabsf(pos.y - node->pos.y + node->vct.y));
		if (xin && yin)
		{
			return 0.0f;
		}
		if (xin)
		{
			return xdist * xdist;
		}
		if (yin)
		{
			return ydist * ydist;
		}
		return xdist * ydist;
	}

	void search_shortest(Node* source, Node* node, Point3 pos, Agent*& e, float& sq_dist)
	{
		if (node == NULL)
		{
			return;
		}

		Agent * ptr = node->agent;
		while (ptr != NULL)
		{
			float d = (ptr->pos - pos).SquareLength();
			if (d < sq_dist)
			{
				e = ptr;
				sq_dist = d;
			}
			ptr = ptr->next;
		}
		if (source != node->right && node->right && get_shortest_dist(node->right, pos) < sq_dist)
		{
//			LOGD("search right. dist:" << get_shortest_dist(node->right, pos) << ", now shortest:"
//				<< sq_dist);
			search_shortest(node, node->right, pos, e, sq_dist);
		}
		if (source != node->left && node->left && get_shortest_dist(node->left, pos) < sq_dist)
		{
//			LOGD("search left. dist:" << get_shortest_dist(node->left, pos) << ", now shortest:"
//				<< sq_dist);
			search_shortest(node, node->left, pos, e, sq_dist);
		}
		if (source != node->parent && node->parent)
		{
//			LOGD("search parent. dist:" << get_shortest_dist(node->parent, pos) << ", now shortest:"
//				<< sq_dist);
			search_shortest(node, node->parent, pos, e, sq_dist);
		}
	}

	Agent* get_shortest(Point3 pos)
	{
		Node* node = get_node(pos);
		if (!node)
		{
			return NULL;
		}

		Agent* e = nullptr;
		float dist = FLT_MAX;

		search_shortest(nullptr, node, pos, e, dist);
		if (dist < FLT_MAX)
		{
			return e;
		}
		return NULL;
	}
protected:

};