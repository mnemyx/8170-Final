//
// ExtentList Object
// D. House, June 13, 2008
// for CCLI Project
//

#ifndef _EXTENTLIST_H_
#define _EXTENTLIST_H_

#include "RBody.h"
#include "OverlapList.h"

enum {XEXT, YEXT, ZEXT};

struct extent{
  RBody *rb;
  double a;
  bool start;

  void print();
};

struct extentstacknode{
  extent entry;
  extentstacknode *next;
};

struct extentstack{
  extentstacknode *top;

  extentstack();
  ~extentstack();

  void clear();
  void push(extent ext);
  void pop();
  void remove(RBody *r);
  void outputoverlaps(RBody *r, OverlapList *overlaps);
};

class ExtentList{
private:
  extent *extlist;
  int xtype;
  int MaxBodies;
  int NEntries;
  extentstack active_extents;
  OverlapList overlaps;

  void record_exchange(extent a, extent b);

public:
  ExtentList(int xext = 1, int MaxBods = 0);
  ~ExtentList();

  void Clear();

  void setMaxBodies(int MaxBods);
  void setListType(int xext);

  void insertBody(RBody *r);

  void Sweep();
  void Sort(int initial = 0);

  void UpdateExtents();

  OverlapList &Overlaps();

  void print();
};

#endif // _EXTENTLIST_H_
