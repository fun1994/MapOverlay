#pragma once
#include <vector>

class Point {
public:
	double x;
	double y;
	int incidentVertex;
	std::vector<int> L;
	std::vector<int> U;
	std::vector<int> C;
};
