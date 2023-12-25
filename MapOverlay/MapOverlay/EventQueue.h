#pragma once
#include "Point.h"
#include "removeAt.h"

class EventQueue {
	int size;
	BinNode<Point>* root;
	BinNode<Point>* hot;
	int updateHeight(BinNode<Point>* x) {
		return x->height = IsBlack(x) + std::max(stature(x->lc), stature(x->rc));
	}
	BinNode<Point>*& search(Point& e) {
		if (!root || (abs(e.x - root->data.x) < EPS && abs(e.y - root->data.y) < EPS)) {
			hot = NULL;
			return root;
		}
		for (hot = root;;) {
			BinNode<Point>*& v = (e.y > hot->data.y) ? hot->lc : hot->rc;
			if (!v || (abs(e.x - v->data.x) < EPS && abs(e.y - v->data.y) < EPS)) {
				return v;
			}
			hot = v;
		}
	}
	BinNode<Point>* connect34(BinNode<Point>* a, BinNode<Point>* b, BinNode<Point>* c, BinNode<Point>* T0, BinNode<Point>* T1, BinNode<Point>* T2, BinNode<Point>* T3) {
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
	BinNode<Point>* rotateAt(BinNode<Point>* v) {
		BinNode<Point>* p = v->parent;
		BinNode<Point>* g = p->parent;
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
	void solveDoubleRed(BinNode<Point>* x) {
		if (IsRoot(*x)) {
			root->color = RB_BLACK;
			root->height++;
			return;
		}
		BinNode<Point>* p = x->parent;
		if (IsBlack(p)) {
			return;
		}
		BinNode<Point>* g = p->parent;
		BinNode<Point>* u = uncle(x);
		if (IsBlack(u)) {
			if (IsLChild(*x) == IsLChild(*p)) {
				p->color = RB_BLACK;
			}
			else {
				x->color = RB_BLACK;
			}
			g->color = RB_RED;
			BinNode<Point>* gg = g->parent;
			BinNode<Point>* r = FromParentTo(*g) = rotateAt(x);
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
	void solveDoubleBlack(BinNode<Point>* r) {
		BinNode<Point>* p = r ? r->parent : hot;
		if (!p) {
			return;
		}
		BinNode<Point>* s = (r == p->lc) ? p->rc : p->lc;
		if (IsBlack(s)) {
			BinNode<Point>* t = NULL;
			if (IsRed(s->rc)) {
				t = s->rc;
			}
			if (IsRed(s->lc)) {
				t = s->lc;
			}
			if (t) {
				RBColor oldColor = p->color;
				BinNode<Point>* b = FromParentTo(*p) = rotateAt(t);
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
			BinNode<Point>* t = IsLChild(*s) ? s->lc : s->rc;
			hot = p;
			FromParentTo(*p) = rotateAt(t);
			solveDoubleBlack(r);
		}
	}
public:
	BinNode<Point>* top;
	EventQueue() :size(0), root(NULL), hot(NULL), top(NULL) {}
	bool empty() {
		return !root;
	}
	bool push(Point& e) {
		BinNode<Point>*& x = search(e);
		if (x) {
			return false;
		}
		x = new BinNode<Point>(e, hot, NULL, NULL, -1);
		size++;
		BinNode<Point>* xOld = x;
		solveDoubleRed(x);
		if (!top || xOld->data.y > top->data.y) {
			top = xOld;
		}
		return true;
	}
	bool pop() {
		BinNode<Point>* succ = top->succ();
		BinNode<Point>*& x = search(top->data);
		top = succ;
		if (!x) {
			return false;
		}
		BinNode<Point>* r = removeAt(x, hot);
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
};
