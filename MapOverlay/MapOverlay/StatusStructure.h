#pragma once
#include "Point.h"
#include "Segment.h"
#include "removeAt.h"

class StatusStructure {
	int size;
	BinNode<int>* root;
	BinNode<int>* hot;
	double intersectionPoint(std::vector<Point>& P, std::vector<Segment>& S, int e, double y0) {
		double t = (y0 - P[S[e].second].y) / (P[S[e].first].y - P[S[e].second].y);
		double x0 = t * P[S[e].first].x + (1 - t) * P[S[e].second].x;
		return x0;
	}
	int updateHeight(BinNode<int>* x) {
		return x->height = IsBlack(x) + std::max(stature(x->lc), stature(x->rc));
	}
	BinNode<int>*& search(std::vector<Point>& P, std::vector<Segment>& S, int e, double y0) {
		if (!root || e == root->data) {
			hot = NULL;
			return root;
		}
		for (hot = root;;) {
			BinNode<int>*& v = (intersectionPoint(P, S, e, y0) < intersectionPoint(P, S, hot->data, y0)) ? hot->lc : hot->rc;
			if (!v || e == v->data) {
				return v;
			}
			hot = v;
		}
	}
	BinNode<int>* connect34(BinNode<int>* a, BinNode<int>* b, BinNode<int>* c, BinNode<int>* T0, BinNode<int>* T1, BinNode<int>* T2, BinNode<int>* T3) {
		a->lc = T0;
		if (T0) {
			T0->parent = a;
		}
		a->rc = T1;
		if (T1) {
			T1->parent = a;
		}
		updateHeight(a);
		c->lc = T2;
		if (T2) {
			T2->parent = c;
		}
		c->rc = T3;
		if (T3) {
			T3->parent = c;
		}
		updateHeight(c);
		b->lc = a;
		a->parent = b;
		b->rc = c;
		c->parent = b;
		updateHeight(b);
		return b;
	}
	BinNode<int>* rotateAt(BinNode<int>* v) {
		BinNode<int>* p = v->parent;
		BinNode<int>* g = p->parent;
		if (IsLChild(*p)) {
			if (IsLChild(*v)) {
				p->parent = g->parent;
				return connect34(v, p, g, v->lc, v->rc, p->rc, g->rc);
			}
			else {
				v->parent = g->parent;
				return connect34(p, v, g, p->lc, v->lc, v->rc, g->rc);
			}
		}
		else {
			if (IsRChild(*v)) {
				p->parent = g->parent;
				return connect34(g, p, v, g->lc, p->lc, v->lc, v->rc);
			}
			else {
				v->parent = g->parent;
				return connect34(g, v, p, g->lc, v->lc, v->rc, p->rc);
			}
		}
	}
	void solveDoubleRed(BinNode<int>* x) {
		if (IsRoot(*x)) {
			root->color = RB_BLACK;
			root->height++;
			return;
		}
		BinNode<int>* p = x->parent;
		if (IsBlack(p)) {
			return;
		}
		BinNode<int>* g = p->parent;
		BinNode<int>* u = uncle(x);
		if (IsBlack(u)) {
			if (IsLChild(*x) == IsLChild(*p)) {
				p->color = RB_BLACK;
			}
			else {
				x->color = RB_BLACK;
			}
			g->color = RB_RED;
			BinNode<int>* gg = g->parent;
			BinNode<int>* r = FromParentTo(*g) = rotateAt(x);
			r->parent = gg;
		}
		else {
			p->color = RB_BLACK;
			p->height++;
			u->color = RB_BLACK;
			u->height++;
			g->color = RB_RED;
			solveDoubleRed(g);
		}
	}
	void solveDoubleBlack(BinNode<int>* r) {
		BinNode<int>* p = r ? r->parent : hot;
		if (!p) {
			return;
		}
		BinNode<int>* s = (r == p->lc) ? p->rc : p->lc;
		if (IsBlack(s)) {
			BinNode<int>* t = NULL;
			if (IsRed(s->rc)) {
				t = s->rc;
			}
			if (IsRed(s->lc)) {
				t = s->lc;
			}
			if (t) {
				RBColor oldColor = p->color;
				BinNode<int>* b = FromParentTo(*p) = rotateAt(t);
				if (HasLChild(*b)) {
					b->lc->color = RB_BLACK;
					updateHeight(b->lc);
				}
				if (HasRChild(*b)) {
					b->rc->color = RB_BLACK;
					updateHeight(b->rc);
				}
				b->color = oldColor;
				updateHeight(b);
			}
			else {
				s->color = RB_RED;
				s->height--;
				if (IsRed(p)) {
					p->color = RB_BLACK;
				}
				else {
					p->height--;
					solveDoubleBlack(p);
				}
			}
		}
		else {
			s->color = RB_BLACK;
			p->color = RB_RED;
			BinNode<int>* t = IsLChild(*s) ? s->lc : s->rc;
			hot = p;
			FromParentTo(*p) = rotateAt(t);
			solveDoubleBlack(r);
		}
	}
public:
	StatusStructure() :size(0), root(NULL), hot(NULL) {}
	BinNode<int>* insert(std::vector<Point>& P, std::vector<Segment>& S, int e, double y0) {
		BinNode<int>*& x = search(P, S, e, y0);
		if (x) {
			return x;
		}
		x = new BinNode<int>(e, hot, NULL, NULL, -1);
		size++;
		BinNode<int>* xOld = x;
		solveDoubleRed(x);
		return xOld;
	}
	bool remove(std::vector<Point>& P, std::vector<Segment>& S, int e, double y0) {
		BinNode<int>*& x = search(P, S, e, y0);
		if (!x) {
			return false;
		}
		BinNode<int>* r = removeAt(x, hot);
		if (!(--size)) {
			return true;
		}
		if (!hot) {
			root->color = RB_BLACK;
			updateHeight(root);
			return true;
		}
		if (BlackHeightUpdated(*hot)) {
			return true;
		}
		if (IsRed(r)) {
			r->color = RB_BLACK;
			r->height++;
			return true;
		}
		solveDoubleBlack(r);
		return true;
	}
	BinNode<int>* leftmost(std::vector<Point>& P, std::vector<Segment>& S, Point& p) {
		BinNode<int>* left = NULL;
		BinNode<int>* v = root;
		while (v) {
			double x = intersectionPoint(P, S, v->data, p.y);
			if (p.x - EPS < x) {
				if (x < p.x + EPS) {
					left = v;
				}
				v = v->lc;
			}
			else {
				v = v->rc;
			}
		}
		return left;
	}
	BinNode<int>* rightmost(std::vector<Point>& P, std::vector<Segment>& S, Point& p) {
		BinNode<int>* right = NULL;
		BinNode<int>* v = root;
		while (v) {
			double x = intersectionPoint(P, S, v->data, p.y);
			if (x < p.x + EPS) {
				if (p.x - EPS < x) {
					right = v;
				}
				v = v->rc;
			}
			else {
				v = v->lc;
			}
		}
		return right;
	}
	BinNode<int>* left(std::vector<Point>& P, std::vector<Segment>& S, Point& p) {
		BinNode<int>* l = NULL;
		BinNode<int>* v = root;
		while (v) {
			double x = intersectionPoint(P, S, v->data, p.y);
			if (p.x < x) {
				v = v->lc;
			}
			else {
				l = v;
				v = v->rc;
			}
		}
		return l;
	}
	BinNode<int>* right(std::vector<Point>& P, std::vector<Segment>& S, Point& p) {
		BinNode<int>* r = NULL;
		BinNode<int>* v = root;
		while (v) {
			double x = intersectionPoint(P, S, v->data, p.y);
			if (p.x < x) {
				r = v;
				v = v->lc;
			}
			else {
				v = v->rc;
			}
		}
		return r;
	}
};
