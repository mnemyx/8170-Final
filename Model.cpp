/*
  Model.cpp

  Source File for Geometric Model Class
  Provides for construction of cuboid, cylinder and cone shapes tiled by triangles

  BIHE Computer Graphics    Donald H. House     6/22/063
  Modified - Gina Guerrero - Fall 2013
*/

#include "Model.h"
#include <cstdlib>

#ifdef __APPLE__
#  include <GLUT/glut.h>
#else
#  include <GL/glut.h>
#endif

using namespace std;

//
// Bookkeeping, remove all vertices and triangles
//
void Model::Clean(){
  nvertices = ntriangles = 0;
  current_plane = current_vtx = 0;
}

//
// Constructor, make sure model is empty
//
Model::Model(){
  Clean();
}

/**
void Model::CopyToOVert() {
    for(int i = 0; i < nvertices; i++)
        overtices[i].set(vertices[i]);
}

void Model::CopyToONorm() {
    for(int i = 0; i < ntriangles; i++)
        onormals[i].set(normals[i]);
}**/

void Model::ComputeAABB() {
    left = right = vertices[0].x;
    bottom = top = vertices[0].y;
    zback = zfront = vertices[0].z;

    for(int i = 1; i < nvertices; i++){
        if(vertices[i].x < left)
            left = vertices[i].x;
        else if(vertices[i].x > right)
            right = vertices[i].x;

        if(vertices[i].y < bottom)
            bottom = vertices[i].y;
        else if(vertices[i].y > top)
            top = vertices[i].y;

        if(vertices[i].z < zback)
            zback = vertices[i].z;
        else if(vertices[i].z > zfront)
            zfront = vertices[i].z;
    }
}

//
// Insert a vertex into the vertex table
//
int Model::AddVertex(const Vector3d &v){
  if(nvertices == MAXVERTICES){
    cerr << "Number of vertices exceeds maximum of " << MAXVERTICES << endl;
    exit(1);
  }

  vertices[nvertices] = v;
  //overtices[nvertices] = v;

  return nvertices++;
}

//
// Insert a triangle and its normal into the triangle tables
//
int Model::AddTriangle(int v0, int v1, int v2){
  if(ntriangles == MAXTRIANGLES){
    cerr << "Number of triangles exceeds maximum of " << MAXTRIANGLES << endl;
    exit(1);
  }

  triangles[ntriangles][0] = v0;
  triangles[ntriangles][1] = v1;
  triangles[ntriangles][2] = v2;

  Vector3d V0(vertices[v0].x, vertices[v0].y, vertices[v0].z);
  Vector3d V1(vertices[v1].x, vertices[v1].y, vertices[v1].z);
  Vector3d V2(vertices[v2].x, vertices[v2].y, vertices[v2].z);
   //V1.print(); V0.print(); V2.print(); V0.print();
  Vector3d V01 = V1 - V0;
  Vector3d V02 = V2 - V0;
  //V01.print();
  //cout << " V01 --------- V02 ";
  //V02.print();
  //cout << endl;

  Vector nullv(0,0,0);
  if ((V01 % V02).norm() == 0) {
	  normals[ntriangles].set(nullv);
	  //onormals[ntriangles].set(nullv);
  } else {
	normals[ntriangles] = (V01 % V02).normalize();
	//onormals[ntriangles] = (V01 % V02).normalize();
  }

  //cout << "normal for triangle: " << ntriangles << " --- ";
  //normals[ntriangles].print();
  //cout << endl;

  return ntriangles++;
}


//
// Make a cuboid model
//
void Model::BuildCuboid(float width, float height, float depth, double x, double y, double z){
  int v[8];
  Vector3d vector;
  int i;
  int isign, jsign, ksign;
  int vlist[36] = {0, 2, 3,     0, 3, 1,    // back
                   4, 5, 6,     5, 7, 6,    // front
		   0, 4, 2,     2, 4, 6,    // left
		   1, 7, 5,     1, 3, 7,    // right
		   0, 1, 4,     1, 5, 4,    // bottom
		   2, 7, 3,     2, 6, 7};   // top

  // delete any old data that may have been built previously
  Clean();
  nedges = 12;
  nplanes = 6;

  // construct the 8 vertices for the cubeoid.
  i = 0;
  for(ksign = -1; ksign <= 1; ksign += 2)
    for(jsign = -1; jsign <= 1; jsign += 2)
      for(isign = -1; isign <= 1; isign += 2){
	vector.set(isign * width / 2, jsign * height / 2, ksign * depth / 2);
	v[i++] = AddVertex(vector);
      }

  // construct the 12 triangles that make the 6 faces
  for(i = 0; i < 36; i += 3)
    AddTriangle(v[vlist[i]], v[vlist[i + 1]], v[vlist[i + 2]]);

  // add edges
  edges[0][0] = 0; edges[0][1] = 1;
  edges[1][0] = 1; edges[1][1] = 3;
  edges[2][0] = 3; edges[2][1] = 2;
  edges[3][0] = 2; edges[3][1] = 0;

  edges[4][0] = 4; edges[4][1] = 5;
  edges[5][0] = 5; edges[5][1] = 7;
  edges[6][0] = 7; edges[6][1] = 6;
  edges[7][0] = 6; edges[7][1] = 4;

  edges[8][0] = 7; edges[8][1] = 3;
  edges[9][0] = 4; edges[9][1] = 0;
  edges[10][0] = 6; edges[10][1] = 2;
  edges[11][0] = 5; edges[11][1] = 1;

}

//
// Make a cylinder model
//
void Model::BuildCylinder(float radius, float height){
  const int NUMFACETS = 16;

  int v[NUMFACETS * 2 + 2];
  Vector3d vector;
  int ksign;
  int i, j;
  float theta;

  Clean();
  Center.set(0.0,0.0,0.0);
  // construct the vertices for the 2 bases of the cylinder
  i = 0;
  for(ksign = -1; ksign <= 1; ksign += 2)
    for(theta = 0; theta < 360; theta += 360.0 / NUMFACETS){
      vector.set(radius * cos(DegToRad(theta)),
		 radius * sin(DegToRad(theta)),
		 ksign * height / 2);
      v[i++] = AddVertex(vector);
    }
  // construct the two vertices at the centers of the bases
  vector.set(0, 0, -height / 2);
  v[i++] = AddVertex(vector);
  vector.set(0, 0, height / 2);
  v[i++] = AddVertex(vector);

  // construct the triangles that make the 2 bases
  for(i = 0; i < NUMFACETS; i++){
    j = (i + 1) % NUMFACETS;
    AddTriangle(v[j], v[i], v[2 * NUMFACETS]);
    AddTriangle(v[i + NUMFACETS], v[j + NUMFACETS], v[2 * NUMFACETS + 1]);
  }

  // construct the triangles that make the sides
  for(i = 0; i < NUMFACETS; i++){
    j = (i + 1) % NUMFACETS;
    AddTriangle(v[i], v[j], v[j + NUMFACETS]);
    AddTriangle(v[i], v[j + NUMFACETS], v[i + NUMFACETS]);
  }
}

//
// Make a cone model
//
void Model::BuildCone(float radius, float height){
  const int NUMFACETS = 16;

  int v[NUMFACETS + 2];
  Vector3d vector;
  int i, j;
  float theta;

  Clean();
  Center.set(0.0,0.0,0.0);
  // construct the vertices for the base of the cone
  i = 0;
  for(theta = 0; theta < 360; theta += 360.0 / NUMFACETS){
    vector.set(radius * cos(DegToRad(theta)),
	       radius * sin(DegToRad(theta)),
	       -height / 2);
    v[i++] = AddVertex(vector);
  }

  // construct the vertex at the center of the base and at the apex of the cone
  vector.set(0, 0, -height / 2);
  v[i++] = AddVertex(vector);
  vector.set(0, 0, height / 2);
  v[i++] = AddVertex(vector);

  // construct the triangles that make the base
  for(i = 0; i < NUMFACETS; i++){
    j = (i + 1) % NUMFACETS;
    AddTriangle(v[j], v[i], v[NUMFACETS]);
  }

  // construct the triangles that make the sides
  for(i = 0; i < NUMFACETS; i++){
    j = (i + 1) % NUMFACETS;
    AddTriangle(v[i], v[j], v[NUMFACETS + 1]);
  }
}

//
// alternate way to build a sphere
// http://www.swiftless.com/tutorials/opengl/sphere.html
//
void Model::BuildSphere(double r, double x, double y, double z) {
    double theta, phi;
    int space = 15;
    double pi = 3.1415926535897;
    Vector3d vector;
    int cnt = ((90 / space) * (360 / space) * 4) * 2;
    int v[cnt];
    int i = 0;

    Clean();

    Center.set(x,y,z);

    for( theta = 0; theta <= 90 - space; theta += space){
        for( phi = 0; phi <= 360 - space; phi += space){
            vector.set(r * sin((phi) / 180 * pi) * sin((theta) / 180 * pi) - x,
                       r * cos((phi) / 180 * pi) * sin((theta) / 180 * pi) + y,
                       r * cos((theta) / 180 * pi) - z);
            v[i++] = AddVertex(vector);

            vector.set(r * sin((phi) / 180 * pi) * sin((theta + space) / 180 * pi) - x,
                       r * cos((phi) / 180 * pi) * sin((theta + space) / 180 * pi) + y,
                       r * cos((theta + space) / 180 * pi) - z);
            v[i++] = AddVertex(vector);

            vector.set(r * sin((phi + space) / 180 * pi) * sin((theta) / 180 * pi) - x,
                       r * cos((phi + space) / 180 * pi) * sin((theta) / 180 * pi) + y,
                       r * cos((theta) / 180 * pi) - z);
            v[i++] = AddVertex(vector);

            vector.set(r * sin((phi + space) / 180 * pi) * sin((theta + space) / 180 * pi) - x,
                       r * cos((phi + space) / 180 * pi) * sin((theta + space) / 180 * pi) + y,
                       r * cos((theta + space) / 180 * pi) - z);
            v[i++] = AddVertex(vector);
        }
    }

    for( theta = 0; theta <= 90 - space; theta += space){
        for( phi = 0; phi <= 360 - space; phi += space){
            vector.set(r * sin((phi) / 180 * pi) * sin((theta) / 180 * pi) - x,
                       r * cos((phi) / 180 * pi) * sin((theta) / 180 * pi) + y,
                       -r * cos((theta) / 180 * pi) - z);
            v[i++] = AddVertex(vector);

            vector.set(r * sin((phi) / 180 * pi) * sin((theta + space) / 180 * pi) - x,
                       r * cos((phi) / 180 * pi) * sin((theta + space) / 180 * pi) + y,
                       -r * cos((theta + space) / 180 * pi) - z);
            v[i++] = AddVertex(vector);

            vector.set(r * sin((phi + space) / 180 * pi) * sin((theta) / 180 * pi) - x,
                       r * cos((phi + space) / 180 * pi) * sin((theta) / 180 * pi) + y,
                       -r * cos((theta) / 180 * pi) - z);
            v[i++] = AddVertex(vector);

            vector.set(r * sin((phi + space) / 180 * pi) * sin((theta + space) / 180 * pi) - x,
                       r * cos((phi + space) / 180 * pi) * sin((theta + space) / 180 * pi) + y,
                       -r * cos((theta + space) / 180 * pi) - z);
            v[i++] = AddVertex(vector);
        }
    }

    for ( i = 0; i < cnt - 2; i++ ) {
		if( i <= (cnt / 2) - 2) {
			if(i%2 == 0)
				AddTriangle(v[i], v[i+2], v[i+1]);
			else
				AddTriangle(v[i], v[i+1], v[i+2]);
		} else {
			if(i%2 == 1)
				AddTriangle(v[i], v[i+2], v[i+1]);
			else
				AddTriangle(v[i], v[i+1], v[i+2]);
		}
	}
}

//
// Make a plane
// Added 9/2013 - Proj 1 - GBG
//
void Model::BuildPlane(float l, float h, int orientation, double x, double y, double z) {
  int v[4];
  Vector3d vector;
  int i;
  int isign, jsign;
  int vlist[6] = {0, 2, 1,     1, 2, 3};   // 2 triangles

  // delete any old data that may have been built previously
  Clean();

  Center.set(x,y,z);
  // construct the 8 vertices for the cubeoid.
  i = 0;
	for(jsign = -1; jsign <= 1; jsign += 2)
	  for(isign = -1; isign <= 1; isign += 2){
		switch(orientation) {
			case(FRONTBACK):
			    if( z < 0 )
					vector.set(jsign * l / 2, isign * h / 2, 0);
				else
					vector.set(isign * l / 2, jsign * h / 2, 0); break;
			case(SIDES):
				if( x < 0 )
					vector.set(0, jsign * l / 2, isign * h / 2);
				else
					vector.set(0, isign * l / 2, jsign * h / 2); break;
			case(TOPBOTTOM):
				if ( y > 0 )
					vector.set(jsign * l / 2, 0, isign * h / 2);
				else
					vector.set(isign * l / 2, 0, jsign * h / 2); break;
		}

		vector = vector + Center;

		// normals are wrong...how to fix it?
		v[i++] = AddVertex(vector);
	  }

  for(i = 0; i < 6; i += 3)
    AddTriangle(v[vlist[i]], v[vlist[i + 1]], v[vlist[i + 2]]);
}


//
// alternate build a plane
// put the coordinates...counter clockwise for positive normal
//
void Model::BuildPlane(Vector3d p0, Vector3d p1, Vector3d p2, Vector3d p3, Vector3d c) {
	int v[4];
	int i = 0;

	Clean();

	Center.set(c);
	nedges = 4;
    nplanes = 1;

	v[i++] = AddVertex(p0);
	v[i++] = AddVertex(p1);
	v[i++] = AddVertex(p2);
	v[i++] = AddVertex(p3);

	AddTriangle(v[0], v[1], v[2]);
	AddTriangle(v[0], v[2], v[3]);

	edges[0][0] = 0; edges[0][1] = 1;
	edges[1][0] = 1; edges[1][1] = 2;
	edges[2][0] = 2; edges[2][1] = 3;
	edges[3][0] = 3; edges[3][1] = 0;

}

//
// build circle...
//
void Model::BuildCircle(float radius, int orientation, double x, double y, double z) {
  const int NUMFACETS = 16;

  int v[NUMFACETS+1];
  Vector3d vector;
  int i, j;
  float theta;

  Clean();
  Center.set(x,y,z);
  // construct the vertices for the base of the cone
  i = 0;
  for(theta = 0; theta < 360; theta += 360.0 / NUMFACETS){
	switch (orientation) {
		case VERTICALX: vector.set(x, radius * cos(DegToRad(theta)) + y, radius * sin(DegToRad(theta)) + z); break;
		case VERTICALZ: vector.set(radius * cos(DegToRad(theta)) + x, radius * sin(DegToRad(theta)) + y, z); break;
		case HORIZON: vector.set(radius * cos(DegToRad(theta)) + x, y, radius * sin(DegToRad(theta)) + z); break;
	}
    v[i++] = AddVertex(vector);
  }

  // construct the vertex at the center
  vector.set(x, y, z);
  v[i++] = AddVertex(vector);

  // construct the triangles that make the base
  for(i = 0; i < NUMFACETS; i++){
    j = (i + 1) % NUMFACETS;
    AddTriangle(v[j], v[i], v[NUMFACETS]);
  }
}

void Model::GetShading(Vector4d color){
	float ambient_color[4];
    float diffuse_color[4];
    float specular_color[4];
    int shininess;

    Vector4d white(1,1,1,1);

    for(int i = 0; i < 3; i++){
      ambient_color[i] = .4 * color[i];
      diffuse_color[i] = .2 * white[i];
      specular_color[i] = .4 * color[i];
      shininess = 6;
    }

    glMaterialfv(GL_FRONT, GL_AMBIENT, ambient_color);
    glMaterialfv(GL_FRONT, GL_DIFFUSE, diffuse_color);
    glMaterialfv(GL_FRONT, GL_SPECULAR, specular_color);
    glMaterialf(GL_FRONT, GL_SHININESS, shininess);
}



//
// Draw the current model in wireframe or shaded
//
void Model::Draw(int wireframe){
  int itri, ivertex;
  int op = (wireframe? GL_LINE_LOOP: GL_POLYGON);

  for(itri = 0; itri < ntriangles; itri++){
    glBegin(op);
      if(!wireframe)
	glNormal3f(normals[itri].x, normals[itri].y, normals[itri].z);
      for(int i = 0; i < 3; i++){
	ivertex = triangles[itri][i];
	glVertex3f(vertices[ivertex].x, vertices[ivertex].y, vertices[ivertex].z);
      }
    glEnd();
  }
}

//
// Draw the current model with the assigned color
//
void Model::Draw(Vector4d color){
  int itri, ivertex;

  GetShading(color);

  for(itri = 0; itri < ntriangles; itri++){
    glBegin(GL_POLYGON);

	  glNormal3f(normals[itri].x, normals[itri].y, normals[itri].z);

      for(int i = 0; i < 3; i++){
		ivertex = triangles[itri][i];
		glVertex3f(vertices[ivertex].x, vertices[ivertex].y, vertices[ivertex].z);
      }
    glEnd();
  }
}

//
// Draw but specifically for the plane - to make them 2 solid colors.
//
void Model::Draw(const float* frontC, const float* backC){
  int itri, ivertex, i;

  glColor4f(backC[0], backC[1], backC[2], backC[3]);
  for(itri = 0; itri < ntriangles; itri++){

	glBegin(GL_TRIANGLES);

	glNormal3f(normals[itri].x, normals[itri].y, normals[itri].z);

    for(int i = 0; i < 3; i++){
	  ivertex = triangles[itri][i];
	  glVertex3f(vertices[ivertex].x, vertices[ivertex].y, vertices[ivertex].z);
    }

    glEnd();
  }

  glColor4f(frontC[0], frontC[1], frontC[2], backC[3]);
  for(itri = 0; itri < ntriangles; itri++){
	glBegin(GL_TRIANGLES);

	glNormal3f(normals[itri].x, normals[itri].y, normals[itri].z);

    for(int i = 0; i < 3; i++){
	  ivertex = triangles[itri][i];
	  glVertex3f(vertices[ivertex].x, vertices[ivertex].y, vertices[ivertex].z);
    }

    glEnd();
  }
}

// get triangle @ index & vertex @ index
Vector3d Model::GetTriangle(int indx) {
	Vector3d temp;
	temp.set(triangles[indx][0], triangles[indx][1], triangles[indx][2]);
	return temp;
}

Vector3d Model::GetVertex(int indx) { return vertices[indx]; }
int Model::GetNtriangles() { return ntriangles; }
Vector3d Model::GetNormal(int indx) { return normals[indx]; }


void Model::place_in_world(const Vector3d &x, const Matrix3x3 &R) {
    Vector3d dx, dy;
    Vector3d n;

    Center = x;

    for (int i = 0; i < nvertices; i++)
        vertices[i] = (R * vertices[i]) + Center;

    for (int i = 0; i < ntriangles; i++)
        normals[i] = (R * normals[i]).normalize();

    int j = 0;
    for(int i = 0; i < ntriangles; i+=2) {
        planes[j++].set(vertices[triangles[i][0]], normals[i]);
        //cout << j << endl;
    }
    //print();
    ComputeAABB();
    //cout << endl << "left: " << left << "; right: " << right;
    //cout << endl << "bottom: " << bottom << "; top: " << top;
    //cout << endl << "zback: " << zback << "; zfront: " << zfront;


}

void Model::print() {
    cout << "MODEL: " << endl;
    cout << "Vertices: " << endl;
    for(int i = 0; i < nvertices; i++)
        cout << "v[" << i << "]: " << vertices[i] << endl;
    cout << "Triangles: " << endl;
    for(int i = 0; i < ntriangles; i++)
        cout << "t[" << i << "]: (" << triangles[i][0] << ", " << triangles[i][1] << ", "<< triangles[i][2] << ")\n";
    cout << "Normals: " << endl;
    for(int i = 0; i < ntriangles; i++)
        cout << "n[" << i << "]: " << normals[i] << '\n';
    cout << "Planes:" << endl;
    for(int i = 0; i < ntriangles/2; i++) {
        cout << "planes[" << i << "]: ";
        planes[i].print();
    }
    cout << "Edges:" << endl;

    for(int i = 0; i < nedges; i++) {
        cout << "edges[" << i << "][0]: " << vertices[edges[i][0]] << ", ";
        cout << "edges[" << i << "][1]: " << vertices[edges[i][1]] << endl;
    }
}

Vector3d Model::FirstV(bool &done) {
    current_vtx = 0;
    done = false;

    return vertices[0];
}

Vector3d Model::NextV(bool &done) {
    if(current_vtx >= nvertices - 1) {
        done = true;
    } else {
        done = false;
        current_vtx++;
    }

    return vertices[current_vtx];
}

Plane Model::FirstP(bool &done) {
    current_plane = 0;
    done = false;

    return planes[0];
}

Plane Model::NextP(bool &done) {
    if(current_plane >= nplanes - 1) {
        done = true;
    } else {
        done =false;
        current_plane++;
    }
    //cout << "current_plane: " << current_plane << endl;
    //cout << "planes[current_plane]: "; planes[current_plane].print(); cout << endl;

    return planes[current_plane];
}

Plane Model::ThisPlane(Model *other, int which) {
    Plane thisplane;
    Vector3d p, n;

    if(which < ntriangles) {
        thisplane = planes[which];
    } else {
        n = (other->GetVertex(0) - vertices[which - 6]).normalize();
        p = vertices[which];
        thisplane.set(p, n);

        cout << "thisplane: "; thisplane.print(); cout << endl;
    }

    return thisplane;
}

