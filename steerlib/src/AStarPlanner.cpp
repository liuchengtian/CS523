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
#define WEIGHT 1.5

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
		if (method == 0)
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

		return fabs(p1.x - p2.x) + fabs(p1.z - p2.z) == 2 ? 1.414 : 1;
	}

	void AStarPlanner::findSuccessor(std::vector<SteerLib::AStarPlannerNode*>& successors, SteerLib::AStarPlannerNode* p, Util::Point goal){
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

				if (i == 1 || i == 3 || i == 4 || i == 6)
				{
					AStarPlannerNode *temp = new AStarPlannerNode(points[i], p->g + 1, p->g + WEIGHT*cal_hn(goal, points[i]), p);
					successors.push_back(temp);
				}

				else
				{
					AStarPlannerNode *temp = new AStarPlannerNode(points[i], p->g + 1.414, p->g + WEIGHT*cal_hn(goal, points[i]), p);
					successors.push_back(temp);
				}

			}
		}
	}

	bool AStarPlanner::computeWeightedAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
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
				cout << "Path length is " << current->g << endl;
				cout << "Number of expand nodes is " << CLOSED->size() << endl;
				cout << "Number of generated nodes is " << OPEN->size() + CLOSED->size();

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
			findSuccessor(successor, current, goal);

			for (int i = 0; i < successor.size(); i++){
				/* if neighbour in closed list continue*/
				if (CLOSED->find(successor[i])){
					continue;
				}
				else{
					float tentGScore = current->g + cal_distance(current->point, successor[i]->point);
					if (tentGScore < successor[i]->g){
						(successor[i]->parent) = current;
						successor[i]->g = tentGScore;
						successor[i]->f = tentGScore + WEIGHT*cal_hn(successor[i]->point, goal);
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

	void AStarPlanner::findSuccessor(std::vector<Util::Point>& points, SteerLib::AStarPlannerNode* p){
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

	void AStarPlanner::improvePath(MyHeap<SteerLib::AStarPlannerNode*>& OPEN, MyHeap<SteerLib::AStarPlannerNode*>& CLOSED,
		MyHeap<SteerLib::AStarPlannerNode*>& INCONS, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight)
	{
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

					float tentGScore = s->g + cal_distance(s->point, ss->point);
					if (tentGScore < ss->g)
					{
						(ss->parent) = s;
						ss->g = tentGScore;
						ss->f = tentGScore + cal_hn(ss->point, goalNode->point);

						if (!CLOSED.find(ss))
						{
							ss->f = tentGScore + weight*cal_hn(ss->point, goalNode->point);
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

	bool AStarPlanner::computeARAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		clock_t startTimer, endTimer;
		double timeDuration = 50;
		startTimer = clock();

		gSpatialDatabase = _gSpatialDatabase;
		// Setup
		MyHeap<SteerLib::AStarPlannerNode*> *OPEN = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *CLOSED = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *INCONS = new MyHeap<AStarPlannerNode*>;


		float weight = 10000;

		SteerLib::AStarPlannerNode *startNode = new AStarPlannerNode(start, 0, cal_hn(start, goal), NULL);
		SteerLib::AStarPlannerNode *goalNode = new AStarPlannerNode(goal, 999999999, 999999999, NULL);

		OPEN->insert(&startNode);

		improvePath(*OPEN, *CLOSED, *INCONS, startNode, goalNode, weight);


		AStarPlannerNode *current = goalNode;
		while (current->point != start){
			agent_path.push_back(current->point);
			current = (current->parent);
		}
		agent_path.push_back(start);
		reverse(agent_path.begin(), agent_path.end());


		float minList = 9999999999;
		if (!OPEN->empty())
			minList = OPEN->top()->g + cal_hn(OPEN->top()->point, goal);
		if (!INCONS->empty())
			minList = min(minList, INCONS->top()->g + cal_hn(OPEN->top()->point, goal));
		weight = min(weight, goalNode->g / minList);

		while (weight > 1)
		{
			weight -= 0.002;

			while (!INCONS->empty())
			{
				AStarPlannerNode* temp = INCONS->top();
				INCONS->pop();
				OPEN->insert(&temp);
			}


			for (int i = 1; i < OPEN->heapDataVec.size(); i++)
			{
				OPEN->heapDataVec[i]->f = OPEN->heapDataVec[i]->g + weight*cal_hn(OPEN->heapDataVec[i]->point, goal);
			}
			OPEN->makeHeap();


			while (!CLOSED->empty())
			{
				CLOSED->pop();
			}
			improvePath(*OPEN, *CLOSED, *INCONS, startNode, goalNode, weight);

			float minList = 9999999999;
			if (!OPEN->empty())
				minList = OPEN->top()->g + cal_hn(OPEN->top()->point, goal);
			if (!INCONS->empty())
				minList = min(minList, INCONS->top()->g + cal_hn(OPEN->top()->point, goal));
			weight = min(weight, goalNode->g / minList);

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
				cout << "run time is " << timeDuration << endl;
				break;
			}
		}

		cout << "Path is found." << endl;
		cout << "Path length is " << goalNode->g << endl;
		cout << "Number of generated nodes is " << OPEN->size() + CLOSED->size() + INCONS->size();

		std::cout << "find path" << endl;
		return false;
	}

	
	float AStarPlanner::key(AStarPlannerNode* s, Point start, float weight)
	{
		if (s->g > s->rhs)
		{
			return 1000 * (s->rhs + weight*cal_hn(s->point, start)) + s->rhs;
		}
		else
		{
			return 1000 * (s->g + weight*cal_hn(s->point, start)) + s->rhs;
		}
	}

	void AStarPlanner::UpdateState(AStarPlannerNode* s, MyHeap<SteerLib::AStarPlannerNode*>& OPEN, MyHeap<SteerLib::AStarPlannerNode*>& CLOSED, MyHeap<SteerLib::AStarPlannerNode*>& INCONS, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight)
	{
		if (!OPEN.find(s) && !CLOSED.find(s) && !INCONS.find(s))
		{
			s->g = 999999999;
			s->f = 999999999;
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
			findSuccessor(points, s);
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
			s->rhs = minSucc;
		}

		if (OPEN.find(s))
		{
			OPEN.remove(s);
		}

		if (s->g != s->rhs)
		{
			if (!CLOSED.find(s))
			{
				s->f = key(s, startNode->point, weight);
				if (OPEN.find(s))
				{
					OPEN.updateState(s);
				}
				else
				{
					OPEN.insert(&s);
				}
			}
			else
			{
				if (INCONS.find(s))
				{
					INCONS.updateState(s);
				}
				else
				{
					INCONS.insert(&s);
				}
			}
		}
	}

	void AStarPlanner::ComputeorImprovePath(MyHeap<SteerLib::AStarPlannerNode*>& OPEN, MyHeap<SteerLib::AStarPlannerNode*>& CLOSED,
		MyHeap<SteerLib::AStarPlannerNode*>& INCONS, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight)
	{
		int count = 0;
		while (key(OPEN.top(), startNode->point, weight) < key(startNode, startNode->point, weight) || startNode->rhs != startNode->g)
		{
			count++;
			if (count % 1000 == 0)
				count++;
			AStarPlannerNode* s = OPEN.top();
			OPEN.pop();

			

			if (s->g > s->rhs)
			{
				s->g = s->rhs;
				CLOSED.insert(&s);
				std::vector<Util::Point> points;
				findSuccessor(points, s);
				for (int i = 0; i < points.size(); i++)
				{
					AStarPlannerNode* ss = new AStarPlannerNode(points[i], -1, -1, s);
					UpdateState(ss, OPEN, CLOSED, INCONS, startNode, goalNode, weight);
				}
			}
			else
			{
				s->g = 999999999;
				std::vector<Util::Point> points;
				findSuccessor(points, s);
				UpdateState(s, OPEN, CLOSED, INCONS, startNode, goalNode, weight);
				for (int i = 0; i < points.size(); i++)
				{
					AStarPlannerNode* ss = new AStarPlannerNode(points[i], -1, -1, s);
					UpdateState(ss, OPEN, CLOSED, INCONS, startNode, goalNode, weight);
				}
			}
			if (*s == *startNode)
				*startNode = *s;
		}
	}

	bool AStarPlanner::computeDAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		clock_t startTimer, endTimer;
		double timeDuration = 50;
		startTimer = clock();

		gSpatialDatabase = _gSpatialDatabase;
		// Setup
		MyHeap<SteerLib::AStarPlannerNode*> *OPEN = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *CLOSED = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *INCONS = new MyHeap<AStarPlannerNode*>;

		float weight = 3;

		SteerLib::AStarPlannerNode *startNode = new AStarPlannerNode(start, 999999999, cal_hn(start, goal), NULL, 999999999);
		SteerLib::AStarPlannerNode *goalNode = new AStarPlannerNode(goal, 999999999, 999999999, NULL, 0);

		startNode->f = key(startNode, start, weight);
		OPEN->insert(&goalNode);

		ComputeorImprovePath(*OPEN, *CLOSED, *INCONS, startNode, goalNode, weight);

		AStarPlannerNode *current = startNode;
		while (current->point != goal)
		{
			agent_path.push_back(current->point);
			current = (current->parent);
		}
		agent_path.push_back(goal);
		//reverse(agent_path.begin(), agent_path.end());

		while (true)
		{
			if (weight > 1)
				weight -= 0.002;

			while (!INCONS->empty())
			{
				AStarPlannerNode* temp = INCONS->top();
				INCONS->pop();
				OPEN->insert(&temp);
			}


			for (int i = 1; i < OPEN->heapDataVec.size(); i++)
			{
				OPEN->heapDataVec[i]->f = OPEN->heapDataVec[i]->g + weight*cal_hn(OPEN->heapDataVec[i]->point, goal);
			}
			OPEN->makeHeap();


			while (!CLOSED->empty())
			{
				CLOSED->pop();
			}
			ComputeorImprovePath(*OPEN, *CLOSED, *INCONS, startNode, goalNode, weight);

			agent_path.clear();
			AStarPlannerNode *current = startNode;
			while (current->point != goal){
				agent_path.push_back(current->point);
				current = (current->parent);
			}
			agent_path.push_back(goal);
			//reverse(agent_path.begin(), agent_path.end());

			endTimer = clock();
			if (endTimer - startTimer > timeDuration)
			{
				//cout << "run time is " << timeDuration << endl;
				break;
			}

		}
		return false;
	}

	bool AStarPlanner::computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;

		//TODO
		std::cout << "\nIn A* \n";
		clock_t startTimer, endTimer;
		startTimer = clock();
		// bool weightedAStar = computeWeightedAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
		// bool ARAstar = computeARAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
		bool DAstar = computeDAStar(agent_path, start, goal, _gSpatialDatabase, append_to_path);
		endTimer = clock();
		cout << "run time: " << (endTimer - startTimer)<< endl;

		cout << "path" << endl;
		return true;
	}
}