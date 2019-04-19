
#include "scene.h"
#include "wglew.h"
#include <iostream>
#include <FreeImage.h>
#include <fstream> 

#include "Utilities.h"
#include "Ray.h"

using namespace std;


// OPENGL场景的各种参数declaration
Scene* Scene::pscene = nullptr;
int Scene::oldX = 0, Scene::oldY = 0;
float Scene::rX = 15, Scene::rY = 0;
int Scene::state = 1;
float Scene::dist = -23;
float Scene::dy = 0;
GLint Scene::viewport[4];
GLfloat Scene::modelview[16];
GLfloat Scene::projection[16];
glm::vec3 Scene::Up = glm::vec3(0, 1, 0),
		  Scene::Right = glm::vec3(0, 0, 0), 
		  Scene::viewDir= glm::vec3(0, 0, 0);
int Scene::selected_index = -1;
static int current_width;
static int current_height ;
bool Scene::start_sim = false;

static int num_screenshot = 0;
GLenum GL_MODE = GL_LINE_LOOP;
bool SAVE_OBJ = false;


void Scene::draw_select_vertex()
{
	if (selected_index != -1)
	{
		glBindBuffer(GL_ARRAY_BUFFER, pscene->cloth->vbo.array_buffer);
		glm::vec4* ptr = (glm::vec4*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_ONLY);
		glUnmapBuffer(GL_ARRAY_BUFFER); // unmap it after use
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		if (ptr)
		{
			auto ver = ptr[selected_index];
		
			glPointSize(5);
			glBegin(GL_POINTS);
			glColor3f(1.0f, 0.0f, 0.0f);
			glVertex3f(ver.x, ver.y, ver.z);
			glPointSize(1);
			glEnd();
		}
	}
}

Scene* Scene::getInstance(int argc, char** argv)
{
	if (pscene == nullptr)
		pscene = new Scene(argc,argv);

	return pscene;
}

Scene::Scene(int argc, char** argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(width,height);
	glutCreateWindow("visg_sim");
	
	GLenum err = glewInit();
	if (err != GLEW_OK) {
		fprintf(stderr, "%s\n", glewGetErrorString(err));
		return;
	}
	wglSwapIntervalEXT(0);  // disable Vertical synchronization
}

Scene::~Scene()
{
	delete simulation;
}

void Scene::add_cloth(Mesh& object)
{
	cloth = &object;
	add(object);
}

void Scene::add_body(Mesh& object)
{
	body = &object;
	add(object);
}

void Scene::add(Mesh& object)
{
	//add VAOs and Buffers
	VAO_Buffer tem_vao;

	glGenVertexArrays(1, &tem_vao.vao);
	glGenBuffers(1, &tem_vao.array_buffer);
	glGenBuffers(1, &tem_vao.index_buffer);
	tem_vao.texture = object.g_textureID;
	tem_vao.index_size = object.vertex_indices.size();
	check_GL_error();

	glBindVertexArray(tem_vao.vao);
	glBindBuffer(GL_ARRAY_BUFFER, tem_vao.array_buffer);

	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*object.vertices.size() + sizeof(glm::vec2)*object.tex.size() + sizeof(glm::vec3)*object.normals.size(), NULL, GL_STATIC_DRAW);
	glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(glm::vec4)*object.vertices.size(), &object.vertices[0]);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*object.vertices.size(), sizeof(glm::vec2)*object.tex.size(), &object.tex[0]);
	glBufferSubData(GL_ARRAY_BUFFER, sizeof(glm::vec4)*object.vertices.size() + sizeof(glm::vec2)*object.tex.size(), sizeof(glm::vec3)*object.normals.size(), &object.normals[0]);
	check_GL_error();

	glVertexAttribPointer(position, 4, GL_FLOAT, GL_FALSE, sizeof(glm::vec4), 0);
	glVertexAttribPointer(texture, 2, GL_FLOAT, GL_FALSE, sizeof(glm::vec2), (const GLvoid*)(sizeof(glm::vec4)*object.vertices.size()));
	glVertexAttribPointer(normal, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (const GLvoid*)(sizeof(glm::vec4)*object.vertices.size() + sizeof(glm::vec2)*object.tex.size()));

	glEnableVertexAttribArray(position);
	glEnableVertexAttribArray(texture);
	glEnableVertexAttribArray(normal);
	glBindBuffer(GL_ARRAY_BUFFER, 0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, tem_vao.index_buffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(GLuint)*object.vertex_indices.size(), &object.vertex_indices[0], GL_STATIC_DRAW);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	glBindVertexArray(0);

	obj_vaos.push_back(tem_vao); // add new vao to the scene
	object.vbo = tem_vao;
}

void Scene::render()
{
	loadShader(); //InitGL(); //load shader

	glutSetOption(GLUT_ACTION_ON_WINDOW_CLOSE, GLUT_ACTION_CONTINUE_EXECUTION);
	glutDisplayFunc(onRender);
	glutReshapeFunc(OnReshape);
	glutIdleFunc(OnIdle);

	glutMouseFunc(OnMouseDown);
	glutMotionFunc(OnMouseMove);
	glutKeyboardFunc(OnKey);
	glutCloseFunc(OnShutdown);

	glutMainLoop();
}

void Scene::RenderBuffer(VAO_Buffer vao_buffer)
{
	GLfloat eyeDir[3] = { viewDir.x,viewDir.y,viewDir.z };

	renderShader.Use();
	glUniformMatrix4fv(renderShader("modelview"), 1, GL_FALSE, modelview);   // the platform does not support "glUniformMatrix4dv"
	glUniformMatrix4fv(renderShader("projection"), 1, GL_FALSE, projection);
	glUniform3fv(renderShader("viewPos"), 1, eyeDir);

	glBindVertexArray(vao_buffer.vao);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, vao_buffer.index_buffer);
		glBindTexture(GL_TEXTURE_2D, vao_buffer.texture);
		glDrawElements(GL_TRIANGLES, (GLsizei)vao_buffer.index_size, GL_UNSIGNED_INT, 0);
		glBindTexture(GL_TEXTURE_2D, 0);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
	glBindVertexArray(0);	
	renderShader.UnUse();
}

void Scene::init_simulation()
{
	if (cloth && body)
	{
		simulation = new Simulator(*cloth, *body);
	}
	else
	{
		cout << "check your cloth and body, make sure both ***not*** null!" << endl;
	}
}

void Scene::check_GL_error()
{
	assert(glGetError() == GL_NO_ERROR);
}

void Scene::loadShader()
{
	//set light
	GLfloat lightPos[3] = { 0,0.0f,10.0f };
	GLfloat lightColor[3] = { 0.8,0.8,0.8 };
	GLfloat objectColor[3] = { 0.8,0.8,0.8 };

	renderShader.LoadFromFile(GL_VERTEX_SHADER, "../src/shaders/sh.vert");   
	renderShader.LoadFromFile(GL_FRAGMENT_SHADER, "../src/shaders/sh.frag");
	renderShader.CreateAndLinkProgram();

	renderShader.Use();
		renderShader.AddUniform("color");
		renderShader.AddUniform("modelview");
		renderShader.AddUniform("projection");
		renderShader.AddUniform("lightPos");                 glUniform3fv(renderShader("lightPos"), 1, lightPos);
		renderShader.AddUniform("viewPos");
		renderShader.AddUniform("lightColor");               glUniform3fv(renderShader("lightColor"), 1, lightColor);
		renderShader.AddUniform("objectColor");              glUniform3fv(renderShader("objectColor"), 1, objectColor);
	renderShader.UnUse();

	check_GL_error();
	glEnable(GL_DEPTH_TEST);  
}

void Scene::save_obj(string file, vector<glm::vec3> vertices)
{
	ofstream outfile(file);
	for (auto ver : vertices)
	{
		outfile << "v " << ver.x << " " << ver.y << " " << ver.z << endl;   //数据写入文件
	}
	outfile.close();
}

void Scene::screenshot()
{

	// Make the BYTE array, factor of 3 because it's RBG.
	BYTE* pixels = new BYTE[3 * current_width * current_height];

	glReadPixels(0, 0, current_width, current_height, GL_BGR, GL_UNSIGNED_BYTE, pixels);

	// Convert to FreeImage format & save to file
	FIBITMAP* image = FreeImage_ConvertFromRawBits(pixels, current_width, current_height, 3 * current_width, 24, 0x0000FF, 0xFF0000, 0x00FF00, false);
	string str = "../screenshot/screenshot";
	str += to_string(num_screenshot++);
	str += ".bmp";

	FreeImage_Save(FIF_BMP, image, str.c_str(), 0);

	// Free resources
	FreeImage_Unload(image);
	delete[] pixels;
	cout << str << " saved successfully!" << endl;
}
// OPENGL场景的各种函数
void Scene::DrawGrid()
{
	const int GRID_SIZE = 10;
	glBegin(GL_LINES);
	glColor3f(0.5f, 0.5f, 0.5f);
	for (int i = -GRID_SIZE; i <= GRID_SIZE; i++)
	{
		glVertex3f((float)i, -2, (float)-GRID_SIZE);
		glVertex3f((float)i, -2, (float)GRID_SIZE);

		glVertex3f((float)-GRID_SIZE, -2, (float)i);
		glVertex3f((float)GRID_SIZE, -2, (float)i);
	}

	glEnd();

}
void Scene::RenderGPU_CUDA()
{
	if (pscene->simulation && start_sim)
	{
		pscene->simulation->simulate(pscene->cloth);
	}
	
	for (auto vao : pscene->obj_vaos)
		pscene->RenderBuffer(vao);
}
void Scene::onRender()
{
	getFPS();
	glClearColor(1.0, 1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslatef(0, dy, 0);
	glTranslatef(0, 0, dist);
	glRotatef(rX, 1, 0, 0);
	glRotatef(rY, 0, 1, 0);

	glGetFloatv(GL_MODELVIEW_MATRIX, modelview);
	glGetFloatv(GL_PROJECTION_MATRIX, projection);
	viewDir.x = (float)-modelview[2];
	viewDir.y = (float)-modelview[6];
	viewDir.z = (float)-modelview[10];
	Right = glm::cross(viewDir, Up);

	DrawGrid();
	draw_select_vertex();
	RenderGPU_CUDA();

	glutSwapBuffers();
}
void Scene::OnReshape(int nw, int nh)
{
	current_width = nw;
	current_height = nh;
	glViewport(0, 0, nw, nh);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(30, (GLfloat)nw / (GLfloat)nh, 0.1f, 100.0f);
}
void Scene::OnIdle()
{
	glutPostRedisplay();
}
void Scene::OnMouseMove(int x, int y)
{
	if (selected_index == -1) {
		if (state == 0)
			dist *= (1 + (y - oldY) / 60.0f);
		else
		{
			rY += (x - oldX) / 5.0f;
			rX += (y - oldY) / 5.0f;
		}
	}
	else {
		float delta = 1500 / abs(dist);
		float valX = (x - oldX) / delta;
		float valY = (oldY - y) / delta;
		if (abs(valX) > abs(valY))
			glutSetCursor(GLUT_CURSOR_LEFT_RIGHT);
		else
			glutSetCursor(GLUT_CURSOR_UP_DOWN);

		glBindBuffer(GL_ARRAY_BUFFER, pscene->cloth->vbo.array_buffer);
		glm::vec4* ptr = (glm::vec4*)glMapBuffer(GL_ARRAY_BUFFER, GL_READ_WRITE);
		glm::vec4 oldVal = ptr[selected_index];
		glUnmapBuffer(GL_ARRAY_BUFFER); // unmap it after use
		glBindBuffer(GL_ARRAY_BUFFER, 0);

		glm::vec4 newVal;
		newVal.w = 1;
		// if the pointer is valid(mapped), update VBO
		if (ptr) {
			// modify buffer data				
			oldVal.x += Right[0] * valX;

			float newValue = oldVal.y + Up[1] * valY;
			if (newValue > 0)
				oldVal.y = newValue;
			oldVal.z += Right[2] * valX + Up[2] * valY;
			newVal = oldVal;
		}

		auto new_value = glm::vec3(newVal);
		pscene->simulation->update_vertex(new_value, selected_index);

	}
	oldX = x;
	oldY = y;

	glutPostRedisplay();
}
void Scene::OnMouseDown(int button, int s, int x, int y)
{
	if (s == GLUT_DOWN)
	{
		oldX = x;
		oldY = y;
		if (button == GLUT_RIGHT_BUTTON)
		{
			int window_y = (height - y);
			int window_x = x;
			float winZ = 0;
			glReadPixels(x, height - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ);
			if (winZ == 1)
				winZ = 0;

			GLdouble MV[16], P[16];
			glGetDoublev(GL_MODELVIEW_MATRIX, MV);
			glGetDoublev(GL_PROJECTION_MATRIX, P);
			glGetIntegerv(GL_VIEWPORT, viewport);

			double objX = 0, objY = 0, objZ = 0;
			gluUnProject(window_x, window_y, winZ, MV, P, viewport, &objX, &objY, &objZ);

			// If you wanna what \gluUnProject do, you can use the following commmented to replcae it 

			// Screen coord - > NDC

			//int x0 = viewport[0];
			//int y0 = viewport[1];
			//int w = viewport[2];
			//int h = viewport[3];

			//auto x_ndc = 2.0 * (window_x - x0) / w - 1;
			//auto y_ndc = 2.0 * (window_y - y0) / h - 1;

			//auto select_dir = glm::vec3(x_ndc, y_ndc, 1.0);


			// NDC -> Camera coord
			// Solve P*(x;y;z;1) = (xn;yn;zn;-z)   (xn/-z, yn/-z, zn/-z)  = select_dir

			//auto p_mat = glm::mat4(P[0], P[1], P[2], P[3], P[4], P[5], P[6], P[7], P[8], P[9], P[10], P[11], P[12], P[13], P[14], P[15]);
			//auto N = glm::mat4(0, 0, 0, 0, 0, 0, 0, 0, select_dir.x, select_dir.y, select_dir.z, 1, 0, 0, 0, 0);
			//auto tem = p_mat + N;

			//Eigen::Matrix<float, 4, 3> A;
			//Eigen::Matrix<float, 4, 1> B;
			//A << tem[0].x, tem[1].x, tem[2].x,
			//	tem[0].y, tem[1].y, tem[2].y,
			//	tem[0].z, tem[1].z, tem[2].z,
			//	tem[0].w, tem[1].w, tem[2].w;
			//B << -tem[3].x, -tem[3].y, -tem[3].z, -tem[3].w;
			//cout << A << endl;
			//cout << B << endl;
			//Eigen::Vector3f  ans = A.colPivHouseholderQr().solve(B);

			//glm::vec3 end = glm::vec3(ans[0], ans[1], ans[2]);
			//auto ray_end = glm::vec3(glm::inverse(mv_mat) * glm::vec4(start, 1));


			auto mv_mat = glm::mat4(MV[0], MV[1], MV[2], MV[3], MV[4], MV[5], MV[6], MV[7], MV[8], MV[9], MV[10], MV[11], MV[12], MV[13], MV[14], MV[15]);
			glm::vec3 ray_end(objX, objY, objZ);
			auto ray_start = glm::vec3(glm::inverse(mv_mat) * glm::vec4(0, 0, 0, 1));  // camera_world

			Ray ray_world(ray_start, glm::normalize(ray_end - ray_start));

			auto tri_idx = find_first_intersect_triangle(ray_world, pscene->cloth);

			// return arbitary vertex idx of the triangle 
			if (tri_idx != -1)
			{
				selected_index = pscene->cloth->faces[tri_idx].vertex_index[0];
			}
		}
	}

	if (button == GLUT_MIDDLE_BUTTON)
		state = 0;
	else
		state = 1;

	if (s == GLUT_UP) {
		selected_index = -1;
		glutSetCursor(GLUT_CURSOR_INHERIT);
	}
}
void Scene::OnKey(unsigned char key, int, int)
{
	switch (key)
	{
	case 'w':
	case 'W':dy -= 0.1; break;
	case 'S':
	case 's':dy += 0.1; break;
	case 'x':
	case 'X':screenshot(); break;
	case 'M':
	case 'm':	
		if (GL_MODE == GL_LINE_LOOP)
			GL_MODE = GL_TRIANGLES;
		else if(GL_MODE == GL_TRIANGLES)
			GL_MODE = GL_POINTS;
		else
			GL_MODE = GL_LINE_LOOP;
		break;
	case 32:            //space
		start_sim = !start_sim; break;
	case 'F':
	case 'f':
		SAVE_OBJ = true;
		break;
	default:
		break;
	}

	glutPostRedisplay();
}
void Scene::OnShutdown()
{
	glutLeaveMainLoop();
}


