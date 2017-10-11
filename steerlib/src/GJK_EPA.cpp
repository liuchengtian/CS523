/**/
#include "obstacles/GJK_EPA.h"


SteerLib::GJK_EPA::GJK_EPA()
{
}

// get the polygon centers
Util::Vector gerCenter(const std::vector<Util::Vector>& _shape)
{
	Util::Vector center(0,0,0);
	for (int i = 0; i < _shape.size(); i++)
	{
		center.x = center.x + _shape[i].x;
		center.y = center.y + _shape[i].y;
		center.z = center.z + _shape[i].z;
	}
	if (_shape.size() == 0)
		return center;
	center.x = center.x / _shape.size();
	center.y = center.y / _shape.size();
	center.z = center.z / _shape.size();
	return center;
}

// input shape and direction, return the point index with farthest distance
Util::Vector getFarthestPoint(const std::vector<Util::Vector>& _shape, Util::Vector d)
{
	int index = 0;
	float distance = _shape[0].x * d.x + _shape[0].y * d.y + _shape[0].z * d.z;
	for (int i = 0; i < _shape.size(); ++i) {
		float tmp_distance = _shape[i].x * d.x + _shape[i].y * d.y + _shape[i].z * d.z;
		if (tmp_distance > distance) {
			distance = tmp_distance;
			index = i;
		}
	}
	return _shape[index];
}

// input polygon A & B, return the Minkowski difference
Util::Vector support(const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, Util::Vector d)
{
	Util::Vector p1 = getFarthestPoint(_shapeA,d);
	Util::Vector p2 = getFarthestPoint(_shapeB, -1 * d);

	return (p1 - p2);
}

// 
Util::Vector getDirection(std::vector<Util::Vector>& simplexes)
{
	Util::Vector last = simplexes[simplexes.size() - 1];
	Util::Vector secondLast = simplexes[simplexes.size() - 2];

	Util::Vector AB = last - secondLast;
	Util::Vector AO = -1 * last;

	//a x b x c = b(a.c) - c(a.b)
	//AB x AO x AB = AO(AB^2) + AB(AB.AO)
	Util::Vector newDirection = AO * (AB * AB) - AB * (AB * AO);
	return newDirection;
}

struct Edge
{
	float closestDis;
	Util::Vector normal;
	int index;
};

Edge findCloestEdge(std::vector<Util::Vector>& simplexes)
{
	Edge closestEdge;
	float dist;
	Util::Vector normal;
	float shortestDist = 99999;
	int j = 0;

	for (int i = 0; i < simplexes.size(); i++)
	{
		j = i + 1;
		if (j == simplexes.size()) j = 0;
				
		Util::Vector A = simplexes[i];
		Util::Vector B = simplexes[j];

		Util::Vector AB = simplexes[j] - simplexes[i];
		Util::Vector AO = -1 * A;

		if (fabs(fabs(AB*AO) - AB.norm() * AO.norm()) >= 0.0001)
			normal = AO*(AB*AB) - AB*(AB*AO); //AB x AO x AB 
		else { //AB*norm = 0 
			normal.x = -1 * AB.z;
			normal.y = 0;
			normal.z = AB.x;
		}

		normal = Util::normalize(normal);
		dist = fabs(AO*normal);

		normal = Util::normalize(normal);

		if (shortestDist > dist) {
			closestEdge.normal = -1 * normal;
			closestEdge.index = j;
			closestEdge.closestDis = dist;
			shortestDist = dist;
		}
	}
	return closestEdge;
}

// input A & B and check if O is in AB
bool checkABO(Util::Vector A, Util::Vector B)
{
	Util::Vector center(0,0,0);
	if (A == center || B == center) {
		return true;
	}
	else if (A.x == 0 && B.x == 0) {
		if ((A.z * B.z) < 0) {
			return true;
		}
	}
	else if (A.z == 0 && B.z == 0) {
		if ((A.x * B.x) < 0) {
			return true;
		}
	}
	else if (A.x != 0 && B.x != 0) {
		float k1 = A.z / A.x;
		float k2 = B.z / B.x;
		if (k1 == k2 && A.z*B.z < 0) {
			return true;
		}
	}
	return false;
}

// check if O is in simplex
bool containsOrigin(std::vector<Util::Vector> &simplexes)
{
	Util::Vector A = simplexes[2];
	Util::Vector B = simplexes[1];
	Util::Vector C = simplexes[0];
	Util::Vector AO = -1 * A;
	Util::Vector AB = B - A;
	Util::Vector AC = C - A;

	Util::Vector ABperp = AC * (AB * AB) - AB* (AB * AC); // (AB X AC) X AB
	Util::Vector ACperp = AB * (AC * AC) - AC * (AC * AB); // (AC X AB) X AC

	//check if origin is on edge
	if (checkABO(A, B) || checkABO(A, C)) {
		return true;
	}

	//if the origin in area
	if (ABperp * AO <= 0) {
		simplexes.erase(simplexes.begin());
		return false;
	}
	else if (ACperp * AO <= 0) {
		simplexes.erase(simplexes.begin() + 1);
		return false;
	}
	else {
		return true;
	}
}

void EPA(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB, std::vector<Util::Vector> &simplex)
{
	Edge closestEdge;

	while (true)
	{
		closestEdge = findCloestEdge(simplex);
		Util::Vector newSupport = support(_shapeA, _shapeB, closestEdge.normal);

		float distance = fabs(newSupport*closestEdge.normal - closestEdge.closestDis);

		if (distance <= 0.001)
		{
			return_penetration_depth = closestEdge.closestDis;
			return_penetration_vector = closestEdge.normal;
			return;
		}
		else simplex.insert(simplex.begin() + closestEdge.index, newSupport);
	}
}

// input Polygon A & B, simplexes, return simplexes and is_colliding 
void GJK(bool& is_colliding, std::vector<Util::Vector> &simplexes, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB){
	//gets centers of Minkowski difference via difference of center of both shapes.
	Util::Vector direction = gerCenter(_shapeA) - gerCenter(_shapeB);

	//get 2 points from the simplex
	Util::Vector simplex = support(_shapeA, _shapeB, direction);
	simplexes.push_back(simplex);
	
	direction = direction*-1;
	simplex = support(_shapeA, _shapeB, direction);
	simplexes.push_back(simplex);

	//build the simplex
	while (true) {
		//get new direction
		direction = getDirection(simplexes);

		//if ABO is in the same line, for testcase 1, no.1 and no.5
		if (direction == Util::Vector(0, 0, 0)) {
			Util::Vector pt1 = simplexes[0];
			Util::Vector pt2 = simplexes[1];
			//origin on edge
			if (checkABO(pt1, pt2)) {
				is_colliding = true;
				return;
			}
			else {
				is_colliding = false;
				return;
			}
		}

		//add minkowski difference vertex
		simplex = support(_shapeA, _shapeB, direction);
		simplexes.push_back(simplex);

		//check simplex contains origin
		if (simplexes[2] * direction <= 0) {
			//simplex past origin
			is_colliding = false;
			return;
		}
		else {
			//Check the simplex area
			if (containsOrigin(simplexes))
			{
				is_colliding = true;
				return;
			}
		}

	}
}

//Look at the GJK_EPA.h header file for documentation and instructions
//input: polygon A and B
//output: bool ifCollide, penetration_depth, penetration_vector 
//see in page 56 of 04-discrete-collisions.pdf
bool SteerLib::GJK_EPA::intersect(float& return_penetration_depth, Util::Vector& return_penetration_vector, const std::vector<Util::Vector>& _shapeA, const std::vector<Util::Vector>& _shapeB)
{
	//Simplex to be build using points found via Minkowski differences
	std::vector<Util::Vector> simplexes;
	
	bool is_colliding = false;
	GJK(is_colliding, simplexes, _shapeA, _shapeB);
	
	if (is_colliding){
		EPA(return_penetration_depth, return_penetration_vector, _shapeA, _shapeB, simplexes);
		return true;
	}
	else return false;
}
