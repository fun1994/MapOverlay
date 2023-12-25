#pragma once
#include <queue>
#include <unordered_set>
#include "DCEL.h"
#include "EventQueue.h"
#include "StatusStructure.h"
#include "Node.h"

class MapOverlay {
	double area2(Point& p, Point& q, Point& r) {
		return p.x * q.y - p.y * q.x + q.x * r.y - q.y * r.x + r.x * p.y - r.y * p.x;
	}
	double area2(Vertex& p, Vertex& q, Vertex& r) {
		return p.x * q.y - p.y * q.x + q.x * r.y - q.y * r.x + r.x * p.y - r.y * p.x;
	}
	bool toLeft(Point& p, Point& q, Point& r) {
		return area2(p, q, r) > 0;
	}
	bool toLeft(Vertex& p, Vertex& q, Vertex& r) {
		return area2(p, q, r) > 0;
	}
	bool toLeft(DCEL& D, HalfEdge& e, Vertex& v) {
		return toLeft(D.V[e.origin], D.V[D.E[e.twin].origin], v);
	}
	bool intersect(std::vector<Point>& P, std::vector<Segment>& S, int i, int j) {
		return toLeft(P[S[i].first], P[S[i].second], P[S[j].first]) != toLeft(P[S[i].first], P[S[i].second], P[S[j].second]) && toLeft(P[S[j].first], P[S[j].second], P[S[i].first]) != toLeft(P[S[j].first], P[S[j].second], P[S[i].second]);
	}
	Point intersectionPoint(std::vector<Point>& P, std::vector<Segment>& S, int i, int j) {
		double a1 = area2(P[S[i].first], P[S[j].first], P[S[j].second]);
		double a2 = area2(P[S[i].second], P[S[j].first], P[S[j].second]);
		Point p;
		p.x = a2 / (a2 - a1) * P[S[i].first].x + a1 / (a1 - a2) * P[S[i].second].x;
		p.y = a2 / (a2 - a1) * P[S[i].first].y + a1 / (a1 - a2) * P[S[i].second].y;
		return p;
	}
	void mergeDCEL(DCEL& D, DCEL& D1, DCEL& D2) {
		D.V.insert(D.V.end(), D1.V.begin(), D1.V.end());
		D.V.insert(D.V.end(), D2.V.begin(), D2.V.end());
		for (int i = D1.V.size(); i < D.V.size(); i++) {
			D.V[i].incidentEdge += D1.E.size();
		}
		D.E.insert(D.E.end(), D1.E.begin(), D1.E.end());
		D.E.insert(D.E.end(), D2.E.begin(), D2.E.end());
		for (int i = D1.E.size(); i < D.E.size(); i++) {
			D.E[i].origin += D1.V.size();
			D.E[i].twin += D1.E.size();
			D.E[i].prev += D1.E.size();
			D.E[i].next += D1.E.size();
		}
	}
	void generateSegments(DCEL& D, std::vector<Point>& P, std::vector<Segment>& S) {
		P.resize(D.V.size());
		S.resize(D.E.size() / 2);
		for (int i = 0; i < D.V.size(); i++) {
			P[i].x = D.V[i].x;
			P[i].y = D.V[i].y;
			P[i].incidentVertex = i;
		}
		for (int i = 0; i < D.E.size(); i++) {
			D.E[i].visited = false;
		}
		int i = 0;
		for (int j = 0; j < D.E.size(); j++) {
			if (!D.E[j].visited) {
				S[i].first = D.E[j].origin;
				S[i].second = D.E[D.E[j].twin].origin;
				if (D.V[D.E[j].origin].y < D.V[D.E[D.E[j].twin].origin].y) {
					S[i].incidentEdge = j;
				}
				else {
					S[i].incidentEdge = D.E[j].twin;
				}
				i++;
				D.E[j].visited = true;
				D.E[D.E[j].twin].visited = true;
			}
		}
	}
	void findIntersections(DCEL& D, std::vector<Point>& P, std::vector<Segment>& S) {
		EventQueue Q;
		initializeEventQueue(P, S, Q);
		StatusStructure T;
		while (!Q.empty()) {
			Point p = Q.top->data;
			Q.pop();
			handleEventPoint(D, P, S, Q, T, p);
		}
	}
	void initializeEventQueue(std::vector<Point>& P, std::vector<Segment>& S, EventQueue& Q) {
		for (int i = 0; i < S.size(); i++) {
			if (P[S[i].first].y > P[S[i].second].y) {
				P[S[i].first].U.push_back(i);
			}
			else {
				P[S[i].second].U.push_back(i);
			}
		}
		for (int i = 0; i < P.size(); i++) {
			Q.push(P[i]);
		}
	}
	void handleEventPoint(DCEL& D, std::vector<Point>& P, std::vector<Segment>& S, EventQueue& Q, StatusStructure& T, Point& p) {
		BinNode<int>* left = T.leftmost(P, S, p);
		BinNode<int>* right = T.rightmost(P, S, p);
		if (left) {
			right = right->succ();
			BinNode<int>* v = left;
			while (v != right) {
				if (P[S[v->data].first].y > P[S[v->data].second].y) {
					if (abs(p.x - P[S[v->data].second].x) < EPS && abs(p.y - P[S[v->data].second].y) < EPS) {
						p.L.push_back(v->data);
					}
					else {
						p.C.push_back(v->data);
					}
				}
				else {
					if (abs(p.x - P[S[v->data].first].x) < EPS && abs(p.y - P[S[v->data].first].y) < EPS) {
						p.L.push_back(v->data);
					}
					else {
						p.C.push_back(v->data);
					}
				}
				v = v->succ();
			}
		}
		if (!p.C.empty()) {
			splitEdge(D, S, p);
		}
		for (int i = 0; i < p.L.size(); i++) {
			T.remove(P, S, p.L[i], p.y + EPS);
		}
		for (int i = 0; i < p.C.size(); i++) {
			T.remove(P, S, p.C[i], p.y + EPS);
		}
		for (int i = 0; i < p.U.size(); i++) {
			T.insert(P, S, p.U[i], p.y - EPS);
		}
		for (int i = 0; i < p.C.size(); i++) {
			T.insert(P, S, p.C[i], p.y - EPS);
		}
		if (p.U.empty() && p.C.empty()) {
			left = T.left(P, S, p);
			right = T.right(P, S, p);
			if (left && right) {
				findNewEvent(P, S, Q, left->data, right->data, p);
			}
			if (left) {
				D.V[p.incidentVertex].leftEdge = D.E[S[left->data].incidentEdge].twin;
			}
			else {
				D.V[p.incidentVertex].leftEdge = -1;
			}
		}
		else {
			left = T.leftmost(P, S, p);
			BinNode<int>* prec = left->prec();
			if (prec) {
				findNewEvent(P, S, Q, prec->data, left->data, p);
			}
			right = T.rightmost(P, S, p);
			BinNode<int>* succ = right->succ();
			if (succ) {
				findNewEvent(P, S, Q, right->data, succ->data, p);
			}
			if (prec) {
				D.V[p.incidentVertex].leftEdge = D.E[S[prec->data].incidentEdge].twin;
			}
			else {
				D.V[p.incidentVertex].leftEdge = -1;
			}
		}
	}
	void findNewEvent(std::vector<Point>& P, std::vector<Segment>& S, EventQueue& Q, int l, int r, Point& p) {
		if (intersect(P, S, l, r)) {
			Point q = intersectionPoint(P, S, l, r);
			if (q.y < p.y) {
				Q.push(q);
			}
		}
	}
	void splitEdge(DCEL& D, std::vector<Segment>& S, Point& p) {
		D.V.resize(D.V.size() + 1);
		D.E.resize(D.E.size() + 4);
		p.incidentVertex = D.V.size() - 1;
		D.V[D.V.size() - 1].x = p.x;
		D.V[D.V.size() - 1].y = p.y;
		D.V[D.V.size() - 1].incidentEdge = D.E.size() - 4;
		for (int i = D.E.size() - 4; i < D.E.size(); i++) {
			D.E[i].origin = D.V.size() - 1;
		}
		D.E[D.E.size() - 4].twin = D.E[S[p.C[0]].incidentEdge].twin;
		D.E[D.E[S[p.C[0]].incidentEdge].twin].twin = D.E.size() - 4;
		D.E[D.E.size() - 3].twin = S[p.C[0]].incidentEdge;
		D.E[S[p.C[0]].incidentEdge].twin = D.E.size() - 3;
		D.E[D.E.size() - 2].twin = D.E[S[p.C[1]].incidentEdge].twin;
		D.E[D.E[S[p.C[1]].incidentEdge].twin].twin = D.E.size() - 2;
		D.E[D.E.size() - 1].twin = S[p.C[1]].incidentEdge;
		D.E[S[p.C[1]].incidentEdge].twin = D.E.size() - 1;
		D.E[D.E.size() - 4].next = D.E[D.E[D.E.size() - 3].twin].next;
		D.E[D.E[D.E[D.E.size() - 3].twin].next].prev = D.E.size() - 4;
		D.E[D.E.size() - 3].next = D.E[D.E[D.E.size() - 4].twin].next;
		D.E[D.E[D.E[D.E.size() - 4].twin].next].prev = D.E.size() - 3;
		D.E[D.E.size() - 2].next = D.E[D.E[D.E.size() - 1].twin].next;
		D.E[D.E[D.E[D.E.size() - 1].twin].next].prev = D.E.size() - 2;
		D.E[D.E.size() - 1].next = D.E[D.E[D.E.size() - 2].twin].next;
		D.E[D.E[D.E[D.E.size() - 2].twin].next].prev = D.E.size() - 1;
		if (toLeft(D, D.E[S[p.C[0]].incidentEdge], D.V[D.E[S[p.C[1]].incidentEdge].origin])) {
			D.E[D.E.size() - 4].prev = D.E[D.E.size() - 1].twin;
			D.E[D.E[D.E.size() - 1].twin].next = D.E.size() - 4;
			D.E[D.E.size() - 3].prev = D.E[D.E.size() - 2].twin;
			D.E[D.E[D.E.size() - 2].twin].next = D.E.size() - 3;
			D.E[D.E.size() - 2].prev = D.E[D.E.size() - 4].twin;
			D.E[D.E[D.E.size() - 4].twin].next = D.E.size() - 2;
			D.E[D.E.size() - 1].prev = D.E[D.E.size() - 3].twin;
			D.E[D.E[D.E.size() - 3].twin].next = D.E.size() - 1;
		}
		else {
			D.E[D.E.size() - 4].prev = D.E[D.E.size() - 2].twin;
			D.E[D.E[D.E.size() - 2].twin].next = D.E.size() - 4;
			D.E[D.E.size() - 3].prev = D.E[D.E.size() - 1].twin;
			D.E[D.E[D.E.size() - 1].twin].next = D.E.size() - 3;
			D.E[D.E.size() - 2].prev = D.E[D.E.size() - 3].twin;
			D.E[D.E[D.E.size() - 3].twin].next = D.E.size() - 2;
			D.E[D.E.size() - 1].prev = D.E[D.E.size() - 4].twin;
			D.E[D.E[D.E.size() - 4].twin].next = D.E.size() - 1;
		}
	}
	void generateFaces(DCEL& D) {
		std::vector<Node> G(1);
		G[0].incidentEdge = -1;
		G[0].isOuterBoundary = true;
		for (int i = 0; i < D.E.size(); i++) {
			D.E[i].incidentNode = -1;
		}
		for (int i = 0; i < D.E.size(); i++) {
			if (D.E[i].incidentNode < 0) {
				G.resize(G.size() + 1);
				int j = i;
				G[G.size() - 1].incidentEdge = j;
				do {
					D.E[j].incidentNode = G.size() - 1;
					if (D.V[D.E[j].origin].x < D.V[D.E[G[G.size() - 1].incidentEdge].origin].x) {
						G[G.size() - 1].incidentEdge = j;
					}
					j = D.E[j].next;
				} while (j != i);
			}
		}
		for (int i = 1; i < G.size(); i++) {
			G[i].isOuterBoundary = toLeft(D, D.E[D.E[G[i].incidentEdge].prev], D.V[D.E[D.E[G[i].incidentEdge].twin].origin]);
			if (!G[i].isOuterBoundary) {
				if (D.V[D.E[G[i].incidentEdge].origin].leftEdge < 0) {
					G[0].incidentNodes.push_back(i);
				}
				else {
					G[D.E[D.V[D.E[G[i].incidentEdge].origin].leftEdge].incidentNode].incidentNodes.push_back(i);
				}
			}
		}
		for (int i = 0; i < G.size(); i++) {
			if (G[i].isOuterBoundary) {
				D.F.resize(D.F.size() + 1);
				D.F[D.F.size() - 1].outerComponent = G[i].incidentEdge;
				if (i > 0) {
					int j = G[i].incidentEdge;
					do {
						D.E[j].incidentFace = D.F.size() - 1;
						j = D.E[j].next;
					} while (j != G[i].incidentEdge);
				}
				std::queue<int> Q;
				for (int j = 0; j < G[i].incidentNodes.size(); j++) {
					Q.push(G[i].incidentNodes[j]);
				}
				while (!Q.empty()) {
					for (int j = 0; j < G[Q.front()].incidentNodes.size(); j++) {
						Q.push(G[Q.front()].incidentNodes[j]);
					}
					D.F[D.F.size() - 1].innerComponents.push_back(G[Q.front()].incidentEdge);
					int j = G[Q.front()].incidentEdge;
					do {
						D.E[j].incidentFace = D.F.size() - 1;
						j = D.E[j].next;
					} while (j != G[Q.front()].incidentEdge);
					Q.pop();
				}
			}
		}
	}
public:
	void mapOverlay(DCEL& D, DCEL& D1, DCEL& D2) {
		mergeDCEL(D, D1, D2);
		std::vector<Point> P;
		std::vector<Segment> S;
		generateSegments(D, P, S);
		findIntersections(D, P, S);
		generateFaces(D);
	}
};
