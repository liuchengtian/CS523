//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman, Rahul Shome
// See license.txt for complete license.
//


#ifndef __STEERLIB_A_STAR_PLANNER_H__
#define __STEERLIB_A_STAR_PLANNER_H__


#include <iostream>  
#include <functional>  

#include <vector>
#include <stack>
#include <set>
#include <map>
#include "SteerLib.h"


const int StartIndex = 1;
#define MIN_VALUE new AStarPlannerNode(Point(0.0f,0.0f,0.0f),-999999999,-999999999,NULL)

using namespace std;
using namespace SteerLib;


namespace SteerLib
{ 
	template <class ElemType>
	class MyHeap{
	private:
		int numCounts; 
		bool comp(ElemType n1, ElemType n2)
		{
			if (n1->f == n2->f)
				return n1->g > n2->g;
			return n1->f > n2->f;
		};

	public:
		MyHeap();

		vector<ElemType> heapDataVec; // data in heap

		void initHeap(ElemType *data, const int n);
		void printfHeap(); // print heap
		void makeHeap(); // make a new heap
		ElemType top();
		void insert(ElemType *elem); // insert a node to heap
		void pop();
		void adjustHeap(int childTree, ElemType adjustValue);
		void percolateUp(int holeIndex, ElemType adjustValue);
		bool empty(); // check if empty
		bool find(ElemType element); // find if in heap
		void remove(ElemType element); // remove value from heap
		int size(); // return numCounts
		void updateState(ElemType element); // update fn and gn in heap
		ElemType getValue(ElemType element);
	};

	template <class ElemType>
	MyHeap<ElemType>::MyHeap()
		:numCounts(0)
	{
		heapDataVec.push_back(MIN_VALUE);
	}


	template <class ElemType>
	void MyHeap<ElemType>::initHeap(ElemType *data, const int n)
	{
		for (int i = 0; i < n; ++i)
		{
			heapDataVec.push_back(*(data + i));
			++numCounts;
		}
	}

	template <class ElemType>
	void MyHeap<ElemType>::printfHeap()
	{
		cout << "Heap : ";
		for (int i = 1; i <= numCounts; ++i)
		{
			cout << heapDataVec[i]->f << " ";
		}
		cout << endl;
	}

	template <class ElemType>
	void MyHeap<ElemType>::makeHeap()
	{
		if (numCounts < 2)
			return; 
		int parent = numCounts / 2;
		while (1)
		{
			adjustHeap(parent, heapDataVec[parent]);
			if (parent == StartIndex)
				return;
			--parent;
		}
	}

	template <class ElemType>
	void MyHeap<ElemType>::insert(ElemType* elem)
	{
		heapDataVec.push_back(*elem);
		++numCounts;
		percolateUp(numCounts, heapDataVec[numCounts]);
	}

	template <class ElemType>
	ElemType MyHeap<ElemType>::top()
	{
		return heapDataVec[StartIndex];
	}

	template <class ElemType>
	void MyHeap<ElemType>::pop()
	{
		if (heapDataVec.size() <= 1)
		{
			std::cout << "The heap is empty." << std::endl;
			return;
		}
		//stage the last node, later put in the first position and adjust heap
		ElemType adjustValue = heapDataVec[numCounts];
		//put the first node in the last position and pop out  
		heapDataVec[numCounts] = heapDataVec[StartIndex]; 
		--numCounts;
		heapDataVec.pop_back();

		if (heapDataVec.size() == 1)
			return;
		adjustHeap(StartIndex, adjustValue);
	}

	template <class ElemType>
	void MyHeap<ElemType>::adjustHeap(int childTree, ElemType adjustValue)
	{ 
		int holeIndex = childTree;
		int secondChid = 2 * holeIndex + 1;
		while (secondChid <= numCounts)
		{
			if (heapDataVec[secondChid] > heapDataVec[secondChid - 1])
			{
				--secondChid;
			}

			heapDataVec[holeIndex] = heapDataVec[secondChid];
			holeIndex = secondChid;
			secondChid = 2 * secondChid + 1;
		}
		if (secondChid == numCounts + 1)
		{
			heapDataVec[holeIndex] = heapDataVec[secondChid - 1];
			holeIndex = secondChid - 1;
		}
		heapDataVec[holeIndex] = adjustValue;

		percolateUp(holeIndex, adjustValue);
	}

	template <class ElemType>
	void MyHeap<ElemType>::percolateUp(int holeIndex, ElemType adjustValue)
	{
		// switch while node is greater than its parent
		int parentIndex = holeIndex / 2;
		while (holeIndex > StartIndex && comp(heapDataVec[parentIndex], adjustValue))
		{
			heapDataVec[holeIndex] = heapDataVec[parentIndex];
			holeIndex = parentIndex;
			parentIndex /= 2;
		}
		heapDataVec[holeIndex] = adjustValue;
	}

	template <class ElemType>
	bool MyHeap<ElemType>::empty()
	{
		if (heapDataVec.size() <= 1)
			return true;
		else
			return false;
	}

	template <class ElemType>
	bool MyHeap<ElemType>::find(ElemType element)
	{
		for (int i = 1; i < heapDataVec.size(); i++)
		{
			if (heapDataVec[i]->point == element->point)
				return true;
		}
		return false;
	}

	template <class ElemType>
	void MyHeap<ElemType>::remove(ElemType element)
	{
		for (int i = 1; i < heapDataVec.size(); i++)
		{
			if (heapDataVec[i]->point == element->point)
			{
				heapDataVec.erase(heapDataVec.begin() + i);
				numCounts--;
				break;
			}
		}
		this->makeHeap();
	}

	template <class ElemType>
	int MyHeap<ElemType>::size()
	{
		return heapDataVec.size() - 1;
	}

	template <class ElemType>
	void MyHeap<ElemType>::updateState(ElemType element)
	{
		for (int i = 1; i < heapDataVec.size(); i++)
		{
			if (heapDataVec[i]->point == element->point)
			{
				if (element->f < heapDataVec[i]->f)
				{
					heapDataVec[i]->f = element->f;
				}
				if (element->g < heapDataVec[i]->g)
				{
					this->remove(element);
					this->insert(&element);
				}
				break;
			}
		}
		this->makeHeap();
	}

	template <class ElemType>
	ElemType MyHeap<ElemType>::getValue(ElemType element)
	{
		for (int i = 1; i < heapDataVec.size(); i++)
		{
			if (heapDataVec[i]->point == element->point)
			{
				return heapDataVec[i];
			}
		}
		return NULL;
	}

	/*
	@function The AStarPlannerNode class gives a suggested container to build your search tree nodes.
	@attributes
	f : the f value of the node
	g : the cost from the start, for the node
	point : the point in (x,0,z) space that corresponds to the current node
	parent : the pointer to the parent AStarPlannerNode, so that retracing the path is possible.
	@operators
	The greater than, less than and equals operator have been overloaded. This means that objects of this class can be used with these operators. Change the functionality of the operators depending upon your implementation

	*/

	class STEERLIB_API AStarPlannerNode{
		public:
			double f;
			double g;
			double rhs;
			Util::Point point;
			AStarPlannerNode* parent;
			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
			}

			AStarPlannerNode(Util::Point _point, double _g, double _f, AStarPlannerNode* _parent, double _rhs)
			{
				f = _f;
				point = _point;
				g = _g;
				parent = _parent;
				rhs = _rhs;
			}

			bool operator<(AStarPlannerNode other) const
			{
				if (this->f == other.f)
					return this->g < other.g;
				return this->f < other.f;
			}
			bool operator>(AStarPlannerNode other) const
			{
				if (this->f == other.f)
					return this->g > other.g;
				return this->f > other.f;
			}
			bool operator==(AStarPlannerNode other) const
			{
				return ((this->point.x == other.point.x) && (this->point.z == other.point.z));
			}

			bool operator!=(AStarPlannerNode other) const
			{
				return ((this->point.x != other.point.x) || (this->point.z != other.point.z));
			}
		};

	class STEERLIB_API AStarPlanner{
		public:
			AStarPlanner();
			~AStarPlanner();
			// NOTE: There are four indices that need to be disambiguated
			// -- Util::Points in 3D space(with Y=0)
			// -- (double X, double Z) Points with the X and Z coordinates of the actual points
			// -- (int X_GRID, int Z_GRID) Points with the row and column coordinates of the GridDatabase2D. The Grid database can start from any physical point(say -100,-100). So X_GRID and X need not match
			// -- int GridIndex  is the index of the GRID data structure. This is an unique id mapping to every cell.
			// When navigating the space or the Grid, do not mix the above up

			/*
				@function canBeTraversed checkes for a OBSTACLE_CLEARANCE area around the node index id for the presence of obstacles.
				The function finds the grid coordinates for the cell index  as (X_GRID, Z_GRID)
				and checks cells in bounding box area
				[[X_GRID-OBSTACLE_CLEARANCE, X_GRID+OBSTACLE_CLEARANCE],
				[Z_GRID-OBSTACLE_CLEARANCE, Z_GRID+OBSTACLE_CLEARANCE]]
				This function also contains the griddatabase call that gets traversal costs.
			*/
			bool canBeTraversed ( int id );
			/*
				@function getPointFromGridIndex accepts the grid index as input and returns an Util::Point corresponding to the center of that cell.
			*/
			Util::Point getPointFromGridIndex(int id);

			/*
				@function computePath
				DO NOT CHANGE THE DEFINITION OF THIS FUNCTION
				This function executes an A* query
				@parameters
				agent_path : The solution path that is populated by the A* search
				start : The start point
				goal : The goal point
				_gSpatialDatabase : The pointer to the GridDatabase2D from the agent
				append_to_path : An optional argument to append to agent_path instead of overwriting it.
			*/
			float AStarPlanner::cal_distance(Util::Point p1, Util::Point p2);

			float AStarPlanner::cal_hn(Util::Point p1, Util::Point p2);

			void AStarPlanner::findSuccessor(std::vector<SteerLib::AStarPlannerNode*>& successors, SteerLib::AStarPlannerNode* p, Util::Point goal);

			bool AStarPlanner::computeWeightedAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path);

			void AStarPlanner::findSuccessor(std::vector<Util::Point>& points, SteerLib::AStarPlannerNode* p);

			void AStarPlanner::improvePath(MyHeap<SteerLib::AStarPlannerNode*>& openList, MyHeap<SteerLib::AStarPlannerNode*>& closedSet,
				MyHeap<SteerLib::AStarPlannerNode*>& inconsList, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight);

			bool AStarPlanner::computeARAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path);

			float AStarPlanner::key(AStarPlannerNode* s, Point start, float weight);

			void AStarPlanner::UpdateState(AStarPlannerNode* s, MyHeap<SteerLib::AStarPlannerNode*>& openList, MyHeap<SteerLib::AStarPlannerNode*>& closedSet, MyHeap<SteerLib::AStarPlannerNode*>& inconsList, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight);

			void AStarPlanner::ComputeorImprovePath(MyHeap<SteerLib::AStarPlannerNode*>& openList, MyHeap<SteerLib::AStarPlannerNode*>& closedSet,
				MyHeap<SteerLib::AStarPlannerNode*>& inconsList, AStarPlannerNode *startNode, AStarPlannerNode *goalNode, float weight);

			bool AStarPlanner::computeDAStar(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path);

			bool computePath(std::vector<Util::Point>& agent_path, Util::Point start, Util::Point goal, SteerLib::SpatialDataBaseInterface * _gSpatialDatabase, bool append_to_path = false);

		private:
			SteerLib::SpatialDataBaseInterface * gSpatialDatabase;
	};


}


#endif
