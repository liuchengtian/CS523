//
// Copyright (c) 2009-2015 Glen Berseth, Mubbasir Kapadia, Shawn Singh, Petros Faloutsos, Glenn Reinman
// See license.txt for complete license.
// Copyright (c) 2015 Mahyar Khayatkhoei
//

#include <algorithm>
#include <vector>
#include <util/Geometry.h>
#include <util/Curve.h>
#include <util/Color.h>
#include <util/DrawLib.h>
#include "Globals.h"

using namespace Util;

Curve::Curve(const CurvePoint& startPoint, int curveType) : type(curveType)
{
	controlPoints.push_back(startPoint);
}

Curve::Curve(const std::vector<CurvePoint>& inputPoints, int curveType) : type(curveType)
{
	controlPoints = inputPoints;
	sortControlPoints();
}

// Add one control point to the vector controlPoints
void Curve::addControlPoint(const CurvePoint& inputPoint)
{
	controlPoints.push_back(inputPoint);
	sortControlPoints();
}

// Add a vector of control points to the vector controlPoints
void Curve::addControlPoints(const std::vector<CurvePoint>& inputPoints)
{
	for (int i = 0; i < inputPoints.size(); i++)
		controlPoints.push_back(inputPoints[i]);
	sortControlPoints();
}

// Draw the curve shape on screen, usign window as step size (bigger window: less accurate shape)
void Curve::drawCurve(Color curveColor, float curveThickness, int window)
{
#ifdef ENABLE_GUI

	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust()) return;
	
	// Move on the curve from t=0 to t=finalPoint, using window as step size, and linearly interpolate the curve points
	// Note that you must draw the whole curve at each frame, that means connecting line segments between each two points on the curve

	float start_time = controlPoints.front().time;
	float end_time = controlPoints.back().time;
	float interval = end_time - start_time;

	int numberPoints = (int)(interval / window);

	float time = start_time + window;
	Point prev = controlPoints.front().position;

	for (int i = 0; i < numberPoints; i++) {
		Point curr;

		//if observing the last point, set time to end_time
		if (i == numberPoints - 1) {
			curr = controlPoints.back().position;
			time = end_time;
		}
		else if (calculatePoint(curr, time) == false) return;

		DrawLib::drawLine(prev, curr, curveColor, curveThickness);
		prev = curr;
		time = time + window;
	}
	return;
#endif
}

// Sort controlPoints vector in ascending order: min-first
void Curve::sortControlPoints()
{
	sort(controlPoints.begin(),controlPoints.end(),[](const  CurvePoint& x,const CurvePoint& y){if (x.time<y.time) {return true;}else {return false;}});
	return;
}

// Calculate the position on curve corresponding to the given time, outputPoint is the resulting position
// Note that this function should return false if the end of the curve is reached, or no next point can be found
bool Curve::calculatePoint(Point& outputPoint, float time)
{
	// Robustness: make sure there is at least two control point: start and end points
	if (!checkRobust())
		return false;

	// Define temporary parameters for calculation
	unsigned int nextPoint;

	// Find the current interval in time, supposing that controlPoints is sorted (sorting is done whenever control points are added)
	// Note that nextPoint is an integer containing the index of the next control point
	if (!findTimeInterval(nextPoint, time))
		return false;

	// Calculate position at t = time on curve given the next control point (nextPoint)
	if (type == hermiteCurve)
	{
		outputPoint = useHermiteCurve(nextPoint, time);
	}
	else if (type == catmullCurve)
	{
		outputPoint = useCatmullCurve(nextPoint, time);
	}

	// Return
	return true;
}

// Check Roboustness
bool Curve::checkRobust()
{
	if (controlPoints.size() >= 3 || (type == hermiteCurve && controlPoints.size() == 2)) return true;
	else return false;
}

// Find the current time interval (i.e. index of the next control point to follow according to current time)
bool Curve::findTimeInterval(unsigned int& nextPoint, float time)
{
	for(unsigned int i =1; i < controlPoints.size(); i++) 
	{
		if(controlPoints[i].time > time){
			nextPoint=i;
			return true;
		}
	}
	std::cerr <<"Error in : findTimeInterval";
	return false;
}

// Implement Hermite curve
Point Curve::useHermiteCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;
	float normalTime, intervalTime;

	Point p1,p2,s1,s2;Vector v1,v2;

	intervalTime = controlPoints[nextPoint].time - controlPoints[nextPoint-1].time;
	p1=controlPoints[nextPoint-1].position;p2=controlPoints[nextPoint].position;
	v1=controlPoints[nextPoint-1].tangent;v2=controlPoints[nextPoint].tangent;
	s1=Point(v1.x,v1.y,v1.z)*intervalTime;s2=Point(v2.x,v2.y,v2.z)*intervalTime;
	//velocities should change because time interval is normalized
	float t=(time-controlPoints[nextPoint-1].time)/intervalTime;
	float t3 = std::pow(t, 3);
	float t2 = std::pow(t, 2);
	newPosition=(2*t3-3*t2+1)*p1+(-2*t3+3*t2)*p2+(t3-2*t2+t)*s1+(t3-t2)*s2;

	// Return result
	return newPosition;
}

// Implement Catmull-Rom curve
Point Curve::useCatmullCurve(const unsigned int nextPoint, const float time)
{
	Point newPosition;


	// Return result
	return newPosition;
}