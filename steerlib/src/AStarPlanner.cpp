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
#define epsilon 1.5

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

	bool AStarPlanner::weightedAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path)
	{
		gSpatialDatabase = _gSpatialDatabase;
		// Setup
		MyHeap<SteerLib::AStarPlannerNode*> *fringe = new MyHeap<AStarPlannerNode*>;
		MyHeap<SteerLib::AStarPlannerNode*> *expanded = new MyHeap<AStarPlannerNode*>;

		SteerLib::AStarPlannerNode *startNode = new AStarPlannerNode(start, 0, 0, NULL);
		SteerLib::AStarPlannerNode *goalNode = new AStarPlannerNode(goal, 999999999, 999999999, NULL);

		fringe->insert(&startNode);

		SteerLib::AStarPlannerNode* current = (fringe->top());

		while (!fringe->empty())
		{
			current = (fringe->top());
			fringe->pop();
			//Find path
			if (*current == *goalNode)
			{
				cout << "Path is found." << endl;
				cout << "Length is " << current->g << endl;
				cout << "Number of expanded nodes: " << expanded->size() << endl;
				cout << "Number of generated nodes: " << fringe->size() + expanded->size() << endl;

				while (current->point != start){
					agent_path.push_back(current->point);
					current = (current->parent);
				}
				agent_path.push_back(start);
				reverse(agent_path.begin(), agent_path.end());
				return true;
			}

			expanded->insert(&current);

			std::vector<SteerLib::AStarPlannerNode*> successor;
			find_successor_wAStar(successor, current, goal);

			for (int i = 0; i < successor.size(); i++){
				/* if neighbour in closed list continue*/
				if (expanded->find(successor[i])){
					continue;
				}
				else{
					float tentGScore = current->g + cal_distance(current->point, successor[i]->point);
					if (tentGScore < successor[i]->g){
						(successor[i]->parent) = current;
						successor[i]->g = tentGScore;
						successor[i]->f = tentGScore + epsilon*cal_hn(successor[i]->point, goal);
					}

					if (!fringe->find(successor[i])){
						fringe->insert(&successor[i]);
					}
					else
					{
						fringe->updateState(successor[i]);
					}

				}
			}
		}

		std::cout << "no path with A*";
		return false;
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

	void AStarPlanner::UpdateState(AStarPlannerNode* s, MyHeap<SteerLib::AStarPlannerNode*>& fringe, MyHeap<SteerLib::AStarPlannerNode*>& expanded, MyHeap<SteerLib::AStarPlannerNode*>& inconsList, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight)
	{
		if (!fringe.find(s) && !expanded.find(s) && !inconsList.find(s))
		{
			s->g = 1000000;
			s->f = s->g;
		}
		else if (inconsList.find(s))
		{
			*s = *inconsList.getValue(s);
		}
		else if (fringe.find(s))
		{
			*s = *fringe.getValue(s);
		}
		else if (expanded.find(s))
		{
			*s = *expanded.getValue(s);
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
					if (!fringe.find(ss) && !expanded.find(ss) && !inconsList.find(ss))
					{
						ss->g = 999999999;
						ss->f = 999999999;
					}
					else if (inconsList.find(ss))
					{
						ss = inconsList.getValue(ss);
					}
					else if (fringe.find(ss))
					{
						ss = fringe.getValue(ss);
					}
					else if (expanded.find(ss))
					{
						ss = expanded.getValue(ss);
					}
					minSucc = min(minSucc, ss->g + cal_distance(ss->point, s->point));
				}
			}
		}

		if (fringe.find(s))
		{
			fringe.remove(s);
		}
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