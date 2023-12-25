#pragma once
#include <vector>

class Vertex {
public:
	double x;
	double y;
	int incidentEdge;
	int leftEdge;
};

class HalfEdge {
public:
	int origin;
	int twin;
	int prev;
	int next;
	int incidentFace;
	bool visited;
	int incidentNode;
};

class Face {
public:
	int outerComponent;
	std::vector<int> innerComponents;
};

class DCEL {
public:
	std::vector<Vertex> V;
	std::vector<HalfEdge> E;
	std::vector<Face> F;
};
