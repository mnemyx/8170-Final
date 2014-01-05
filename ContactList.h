//
// Contact Object
// D. House, June 13, 2008
// for CCLI Project
//

#ifndef _CONTACTLIST_H_
#define _CONTACTLIST_H_

#include "RBody.h"
#include "OverlapList.h"
#include "Plane.h"

struct Contact{
  RBody *a, *b;
  Vector3d p;
  Vector3d n;

  Contact();
  ~Contact();

  double vrel();
  double impulse();

  void set(RBody *abody, RBody *body,
	   const Plane &plane);
};

struct ContactList{
  Contact *contacts;
  EntryTable entries;

  int ncontacts;
  int current;

  ContactList(int Maxbodies = 0);
  ~ContactList();

  void setSize(int Maxbodies);

  void Clear();

  void Insert(const Overlap &newoverlap);
  void Remove(RBody *rb1, RBody *rb2);

  void ExtractContacts(OverlapList &overlaps);

  Contact *First();
  Contact *Next();

  void print();
  void draw();
};

#endif // _CONTACTLIST_H_
