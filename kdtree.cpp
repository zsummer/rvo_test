#include "kdtree.h"


void KDTree::clear(Node* node, int depth)
{
	if (node == NULL)
	{
		return;
	}
	//LOGD("clear left:" << depth);
	clear(node->left, depth + 1);
	//LOGD("clear right:" << depth);
	clear(node->right, depth + 1);

	Agent* agent = node->agent;
	while (agent != nullptr)
	{
		Agent* temp = agent;
		agent = agent->next;
		delete temp;
	}
	delete node;
}

void KDTree::view(Node* node, Point3 color, float line_wide, const DrawLine& drawline)
{
	if (node == NULL)
	{
		return;
	}

	if (node->right)
	{
		if (node->dim == 0)
		{
			drawline(line_wide, { color.x, 1.0f, color.z },
				node->right->pos,
				{ node->right->pos.x, node->right->pos.y + node->vct.y, 0.0f });
		}
		else
		{
			drawline(line_wide, { color.x, 0.0f, color.z },
				node->right->pos,
				{ node->right->pos.x + node->right->vct.x, node->right->pos.y, 0.0f });
		}
	}

	Agent* eptr = node->agent;
	while (eptr != NULL)
	{
		drawline(5, { 0.0f, 1.0f, 1.0f },
			{ eptr->pos.x - 0.01f, eptr->pos.y, 0.0f },
			{ eptr->pos.x + 0.01f, eptr->pos.y, 0.0f });
		eptr = eptr->next;
	}

	color.x -= 0.1f;
	view(node->left, color, line_wide - 0.35f, drawline);
	view(node->right, color, line_wide - 0.35f, drawline);
}





