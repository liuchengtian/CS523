//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#include <vector>
#include <stack>
#include <set>
#include <map>
#include <iostream>
#include <algorithm> 
#include <functional>
#include <queue>
#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include "planning/AStarPlanner.h"


#define COLLISION_COST  1000
#define GRID_STEP  1
#define OBSTACLE_CLEARANCE 1
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))

#define method 0
static float epsilon = 1.5;
static float epsilon_ara = 1.5;


namespace SteerLib
{
	AStarPlanner::AStarPlanner(){}

	AStarPlanner::~AStarPlanner(){}

	bool AStarPlanner::canBeTraversed(int id)
	{
		double traversal_cost = 0;
		int current_id = id;
		unsigned int x, z;
		gSpatialDatabase->getGridCoordinatesFromIndex(current_id, x, z);
		int x_range_min, x_range_max, z_range_min, z_range_max;

		x_range_min = MAX(x - OBSTACLE_CLEARANCE, 0);
		x_range_max = MIN(x + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsX());

		z_range_min = MAX(z - OBSTACLE_CLEARANCE, 0);
		z_range_max = MIN(z + OBSTACLE_CLEARANCE, gSpatialDatabase->getNumCellsZ());


		for (int i = x_range_min; i <= x_range_max; i += GRID_STEP)
		{
			for (int j = z_range_min; j <= z_range_max; j += GRID_STEP)
			{
				int index = gSpatialDatabase->getCellIndexFromGridCoords(i, j);
				traversal_cost += gSpatialDatabase->getTraversalCost(index);

			}
		}

		if (traversal_cost > COLLISION_COST)
			return false;
		return true;
	}



	Util::Point AStarPlanner::getPointFromGridIndex(int id)
	{
		Util::Point p;
		gSpatialDatabase->getLocationFromIndex(id, p);
		return p;
	}

	float AStarPlanner::cal_hn(Util::Point p1, Util::Point p2)
	{
		if (method)
		{
			return fabs(p1.x - p2.x) + fabs(p1.z - p2.z);
		}
		else
		{
			return sqrt(pow((p1.x - p2.x), 2) + pow((p1.z - p2.z), 2));
		}
	}

	float AStarPlanner::cal_distance(Util::Point p1, Util::Point p2)
	{
		if (fabs(p1.x - p2.x) + fabs(p1.z - p2.z) == 2){
			return 1.414;
		}
		else return 1;
	}

	void AStarPlanner::find_successor_wAStar(std::vector<SteerLib::AStarPlannerNode*>& successors, SteerLib::AStarPlannerNode* p, Util::Point goal){
		/* return all AStarPlannerNodes */
		std::vector<Util::Point> points;

		int x = p->point.x;
		int y = p->point.y;
		int z = p->point.z;

		points.push_back(Util::Point(x - 1, y, z + 1));
		points.push_back(Util::Point(x, y, z + 1));
		points.push_back(Util::Point(x + 1, y, z + 1));
		points.push_back(Util::Point(x - 1, y, z));
		points.push_back(Util::Point(x + 1, y, z));
		points.push_back(Util::Point(x - 1, y, z - 1));
		points.push_back(Util::Point(x, y, z - 1));
		points.push_back(Util::Point(x + 1, y, z - 1));

		for (int i = 0; i < points.size(); i++){
			if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i]))){

				if (i == 1 || i == 3 || i == 4 || i == 6){
					AStarPlannerNode *temp = new AStarPlannerNode(points[i], p->g + 1, p->g + epsilon*cal_hn(goal, points[i]), p);
					successors.push_back(temp);
				}
				else{
					AStarPlannerNode *temp = new AStarPlannerNode(points[i], p->g + 1.414, p->g + epsilon*cal_hn(goal, points[i]), p);
					successors.push_back(temp);
				}

			}
		}
	}

		void AStarPlanner::push_neighbors(std::vector<Util::Point>& points, SteerLib::AStarPlannerNode* p){
		/* return all AStarPlannerNodes */

		int x = p->point.x;
		int y = p->point.y;
		int z = p->point.z;

		points.push_back(Util::Point(x - 1, y, z + 1));
		points.push_back(Util::Point(x, y, z + 1));
		points.push_back(Util::Point(x + 1, y, z + 1));
		points.push_back(Util::Point(x - 1, y, z));
		points.push_back(Util::Point(x + 1, y, z));
		points.push_back(Util::Point(x - 1, y, z - 1));
		points.push_back(Util::Point(x, y, z - 1));
		points.push_back(Util::Point(x + 1, y, z - 1));
	}

	void AStarPlanner::UpdateState(AStarPlannerNode* s, MyHeap<SteerLib::AStarPlannerNode*>& OPEN, MyHeap<SteerLib::AStarPlannerNode*>& CLOSED, MyHeap<SteerLib::AStarPlannerNode*>& INCONS, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight)
	{
		if (!OPEN.find(s) && !CLOSED.find(s) && !INCONS.find(s))
		{
			s->g = 1000000;
			s->f = s->g;
		}
		else if (INCONS.find(s))
		{
			*s = *INCONS.getValue(s);
		}
		else if (OPEN.find(s))
		{
			*s = *OPEN.getValue(s);
		}
		else if (CLOSED.find(s))
		{
			*s = *CLOSED.getValue(s);
		}

		if (*s != *goalNode)
		{
			std::vector<Util::Point> points;
			push_neighbors(points, s);
			float minSucc = FLT_MAX;
			for (int i = 0; i < points.size(); i++)
			{
				if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i])))
				{
					AStarPlannerNode* ss = new AStarPlannerNode(points[i], -1, -1, s);
					if (!OPEN.find(ss) && !CLOSED.find(ss) && !INCONS.find(ss))
					{
						ss->g = 999999999;
						ss->f = 999999999;
					}
					else if (INCONS.find(ss))
					{
						ss = INCONS.getValue(ss);
					}
					else if (OPEN.find(ss))
					{
						ss = OPEN.getValue(ss);
					}
					else if (CLOSED.find(ss))
					{
						ss = CLOSED.getValue(ss);
					}
					minSucc = min(minSucc, ss->g + cal_distance(ss->point, s->point));
				}
			}
		}

		if (OPEN.find(s))
		{
			OPEN.remove(s);
		}
	}

	bool AStarPlanner::weightedAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		// Setup
		MyHeap<SteerLib::AStarPlannerNode*> *OPEN = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *CLOSED = new MyHeap<AStarPlannerNode*>;

		SteerLib::AStarPlannerNode *startNode = new AStarPlannerNode(start, 0, 0, NULL);
		SteerLib::AStarPlannerNode *goalNode = new AStarPlannerNode(goal, 999999999, 999999999, NULL);

		OPEN->insert(&startNode);

		SteerLib::AStarPlannerNode* current = (OPEN->top());

		while (!OPEN->empty())
		{
			current = (OPEN->top());
			OPEN->pop();
			//Find path
			if (*current == *goalNode)
			{
				cout << "Path is found." << endl;
				cout << "Length is " << current->g << endl;
				cout << "Number of expanded nodes: " << CLOSED->size() << endl;
				cout << "Number of generated nodes: " << OPEN->size() + CLOSED->size() << endl;

				while (current->point != start){
					agent_path.push_back(current->point);
					current = (current->parent);
				}
				agent_path.push_back(start);
				reverse(agent_path.begin(), agent_path.end());
				return true;
			}

			CLOSED->insert(&current);

			std::vector<SteerLib::AStarPlannerNode*> successor;
			find_successor_wAStar(successor, current, goal);

			for (int i = 0; i < successor.size(); i++){
				/* if neighbour in CLOSED list continue*/
				if (CLOSED->find(successor[i])){
					continue;
				}
				else{
					float tentGScore = current->g + cal_distance(current->point, successor[i]->point);
					if (tentGScore < successor[i]->g){
						(successor[i]->parent) = current;
						successor[i]->g = tentGScore;
						successor[i]->f = tentGScore + epsilon*cal_hn(successor[i]->point, goal);
					}

					if (!OPEN->find(successor[i])){
						OPEN->insert(&successor[i]);
					}
					else
					{
						OPEN->updateState(successor[i]);
					}

				}
			}
		}

		std::cout << "no path with A*";
		return false;
	}

	void AStarPlanner::improvePath(MyHeap<SteerLib::AStarPlannerNode*>& OPEN, MyHeap<SteerLib::AStarPlannerNode*>& CLOSED,
		MyHeap<SteerLib::AStarPlannerNode*>& INCONS, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight)
	{
		// goal node's fvalue = gvalue + weight * 0
		while (goalNode->g > OPEN.top()->f)
		{
			AStarPlannerNode* s = OPEN.top();
			OPEN.pop();
			CLOSED.insert(&s);

			std::vector<Util::Point> points;
			findSuccessor(points, s);

			for (int i = 0; i < points.size(); i++)
			{
				if (canBeTraversed(gSpatialDatabase->getCellIndexFromLocation(points[i])))
				{
					// ss is s'
					AStarPlannerNode* ss = new AStarPlannerNode(points[i], -1, -1, s);
					if (!OPEN.find(ss) && !CLOSED.find(ss) && !INCONS.find(ss))
					{
						ss->g = 1000000;
						ss->f = ss->g;
					}
					else if (INCONS.find(ss))
					{
						ss = INCONS.getValue(ss);
					}
					else if (OPEN.find(ss))
					{
						ss = OPEN.getValue(ss);
					}
					else if (CLOSED.find(ss))
					{
						ss = CLOSED.getValue(ss);
					}

					float new_distance = s->g + calculate_dist(s->point, ss->point);
					if (ss->g > new_distance)
					{
						(ss->parent) = s;
						ss->g = new_distance;
						ss->f = new_distance + weight * calculate_h(ss->point, goalNode->point);

						if (!CLOSED.find(ss))
						{
							if (OPEN.find(ss))
							{
								OPEN.updateState(ss);
							}
							else
							{
								OPEN.insert(&ss);
							}
						}
						else
						{
							if (INCONS.find(ss))
							{
								INCONS.updateState(ss);
							}
							else
							{
								INCONS.insert(&ss);
							}
						}
					}
					if (*ss == *goalNode)
					{
						*goalNode = *ss;
						if (OPEN.find(goalNode))
							OPEN.remove(goalNode);
						if (INCONS.find(goalNode))
							INCONS.remove(goalNode);
					}
				}
			}
		}
	}

	bool AStarPlanner::computeARAstar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		clock_t startTimer, endTimer;
		double timeDuration = 50;
		startTimer = clock();

		gSpatialDatabase = _gSpatialDatabase;

		MyHeap<SteerLib::AStarPlannerNode*> *OPEN = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *CLOSED = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *INCONS = new MyHeap<AStarPlannerNode*>;

		SteerLib::AStarPlannerNode *startNode = new AStarPlannerNode(start, 0, calculate_h(start, goal), NULL);
		SteerLib::AStarPlannerNode *goalNode = new AStarPlannerNode(goal, 999999999, 999999999, NULL);

		OPEN->insert(&startNode);
		improvePath(*OPEN, *CLOSED, *INCONS, startNode, goalNode, epsilon_ara);

		AStarPlannerNode *current = goalNode;
		while (current->point != start){
			agent_path.push_back(current->point);
			current = (current->parent);
		}
		agent_path.push_back(start);
		reverse(agent_path.begin(), agent_path.end());

		float minSucc = FLT_MAX;
		if (!OPEN->empty())
			minSucc = OPEN->top()->g + calculate_h(OPEN->top()->point, goal);
		if (!INCONS->empty())
			minSucc = min(minSucc, INCONS->top()->g + calculate_h(INCONS->top()->point, goal));
		epsilon_ara2 = min(epsilon_ara, goalNode->g / minSucc);

		while (epsilon_ara2 > 1)
		{
			epsilon_ara -= 0.002;

			while (!INCONS->empty())
			{
				AStarPlannerNode* temp = INCONS->top();
				INCONS->pop();
				OPEN->insert(&temp);
			}

			for (int i = 1; i < OPEN->heapDataVec.size(); i++)
			{
				OPEN->heapDataVec[i]->f = OPEN->heapDataVec[i]->g + epsilon_ara*calculate_h(OPEN->heapDataVec[i]->point, goal);
			}
			OPEN->makeHeap();

			while (!CLOSED->empty())
			{
				CLOSED->pop();
			}
			improvePath(*OPEN, *CLOSED, *INCONS, startNode, goalNode, epsilon_ara);

			float minSucc = FLT_MAX;
			if (!OPEN->empty())
				minSucc = OPEN->top()->g + calculate_h(OPEN->top()->point, goal);
			if (!INCONS->empty())
				minSucc = min(minSucc, INCONS->top()->g + calculate_h(INCONS->top()->point, goal));
			epsilon_ara2 = min(epsilon_ara, goalNode->g / minSucc);

			agent_path.clear();
			AStarPlannerNode *current = goalNode;
			while (current->point != start){
				agent_path.push_back(current->point);
				current = (current->parent);
			}
			agent_path.push_back(start);
			reverse(agent_path.begin(), agent_path.end());

			endTimer = clock();
			if (endTimer - startTimer > timeDuration)
			{
				cout << "run time is " << timeDuration << ", forced to stop." << endl;
				break;
			}
		}

		cout << "Path is found." << endl;
		cout << "Length is " << goalNode->g << endl;
		cout << "Number of expanded nodes: " << CLOSED->size() << endl;
		cout << "Number of generated nodes: " << OPEN->size() + CLOSED->size() + INCONS->size();

		std::cout << "find path" << endl;
		return false;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout << "\nIn A* \n";
		clock_t startTime, endTime;
		startTime = clock();
		bool normalAstar = weightedAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
		endTime = clock();
		cout << "run time: " << (endTime - startTime)<< endl;

		cout << "path" << endl;
		return true;
	}
}