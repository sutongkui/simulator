#include <time.h>
#include <string>
#include <iostream>

#include <GL/freeglut.h>

#include "Utilities.h"

using namespace std;
 
float startTime = 0;
int totalFrames = 0;
float currentTime = 0;
char info[MAX_PATH] = { 0 };

void getFPS()
{
	float fps = 0;
	float newTime = (float)glutGet(GLUT_ELAPSED_TIME);
	float frameTime = newTime - currentTime;
	currentTime = newTime;
	++totalFrames;
	if ((newTime - startTime)>1000)
	{
		float elapsedTime = (newTime - startTime);
		fps = (totalFrames / elapsedTime) * 1000;
		startTime = newTime;
		totalFrames = 0;

		sprintf_s(info, "GLUT Cloth Demo FPS: %4.3f", fps);
		//cout << fps << endl;
	}
	glutSetWindowTitle(info);
}

