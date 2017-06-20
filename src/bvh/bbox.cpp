#include "bbox.h"
#include <GL/glew.h>
#include <iostream>
using namespace std;

void BBox::draw() const {

	// top
	glBegin(GL_LINE_STRIP);
	glVertex3d(max.x, max.y, max.z);
	glVertex3d(max.x, max.y, min.z);
	glVertex3d(min.x, max.y, min.z);
	glVertex3d(min.x, max.y, max.z);
	glVertex3d(max.x, max.y, max.z);
	glEnd();

	// bottom
	glBegin(GL_LINE_STRIP);
	glVertex3d(min.x, min.y, min.z);
	glVertex3d(min.x, min.y, max.z);
	glVertex3d(max.x, min.y, max.z);
	glVertex3d(max.x, min.y, min.z);
	glVertex3d(min.x, min.y, min.z);
	glEnd();

	// side
	glBegin(GL_LINES);
	glVertex3d(max.x, max.y, max.z);
	glVertex3d(max.x, min.y, max.z);
	glVertex3d(max.x, max.y, min.z);
	glVertex3d(max.x, min.y, min.z);
	glVertex3d(min.x, max.y, min.z);
	glVertex3d(min.x, min.y, min.z);
	glVertex3d(min.x, max.y, max.z);
	glVertex3d(min.x, min.y, max.z);
	glEnd();

}

void BBox::print() const
{
	cout << "min" << min.x << "  " << min.y << " " << min.z << ";  ";
	cout << "max" << max.x << "  " << max.y << " " << max.z << endl;
}