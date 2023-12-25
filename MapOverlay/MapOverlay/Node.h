#pragma once
#include <vector>

class Node {
public:
	int incidentEdge;
	bool isOuterBoundary;
	std::vector<int> incidentNodes;
};
