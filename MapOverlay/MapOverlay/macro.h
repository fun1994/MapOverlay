#pragma once
typedef enum { RB_RED, RB_BLACK } RBColor;
#define stature(p) ((p) ? (p)->height : -1)
#define IsRoot(x) (!((x).parent))
#define IsLChild(x) (!IsRoot(x) && (&(x) == (x).parent->lc))
#define IsRChild(x) (!IsRoot(x) && (&(x) == (x).parent->rc))
#define HasLChild(x) ((x).lc)
#define HasRChild(x) ((x).rc)
#define sibling(p) (IsLChild(*(p)) ? (p)->parent->rc : (p)->parent->lc)
#define uncle(x) (sibling((x)->parent))
#define FromParentTo(x) (IsRoot(x) ? root : (IsLChild(x) ? (x).parent->lc : (x).parent->rc))
#define IsBlack(p) (!(p) || (RB_BLACK == (p)->color))
#define IsRed(p) (!IsBlack(p))
#define BlackHeightUpdated(x) ((stature((x).lc) == stature((x).rc)) && ((x).height == (IsRed(&x) ? stature((x).lc) : stature((x).lc) + 1)))
#define EPS 1e-9
