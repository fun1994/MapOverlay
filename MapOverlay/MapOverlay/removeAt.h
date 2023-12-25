#pragma once
#include "BinNode.h"

template <typename T>
BinNode<T>* removeAt(BinNode<T>*& x, BinNode<T>*& hot) {
	BinNode<T>* w = x;
	BinNode<T>* succ = NULL;
	if (!HasLChild(*x)) {
		succ = x = x->rc;
	}
	else if (!HasRChild(*x)) {
		succ = x = x->lc;
	}
	else {
		w = w->succ();
		std::swap(x->data, w->data);
		BinNode<T>* u = w->parent;
		((u == x) ? u->rc : u->lc) = succ = w->rc;
	}
	hot = w->parent;
	if (succ) {
		succ->parent = hot;
	}
	return succ;
}
