#include <Windows.h>
#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>
#include <time.h>
#include "robot.h"
#include "smart_robot.cpp"
#include "basic_robot.cpp"
using namespace std;


#define buffer_size 1000000
#define channels 2 
//#define delay 10 //delay between time steps, use if program is too fast
#define windowWidth 800 //display window
#define windowHeight 800 //display window
#define comm_noise_std 5 //standard dev. of sensor noise
#define PI 3.14159265358979324
#define twicePi  2 * PI
#define radius 16 //radius of a robot
#define p_control_execute .99 // probability of a controller executing its time step
#define arena_width 2400
#define arena_height 2400
#define SKIPFRAMES 0
#define shuffles 20
#define circledef 30
// Global vars.
double time_sim;  //simulation time
float zoom, view_x, view_y; //var. for zoom and scroll

#define num_robots 1000 //number of robots running
#define num_smart_robots 5 //number of robots running

robot** robots = new robot*[num_robots];//creates an array of robots
int safe_distance[num_robots][num_robots];
int order[shuffles * num_robots];

int delay = 0;

FILE *results;

char log_buffer[255];
char log_file_buffer[buffer_size];


bool log_debug_info = true;
char log_file_name[255];
bool showscene = true;

int secs;
char rt[100];

double ch[radius];


void log_info(char *s)
{
	static char *m = log_file_buffer;
	//cout << s;
	int l = strlen(s)+1;
	strcpy_s(m, l, s);
	m += l-1;
	if (m - log_file_buffer >= buffer_size-255)
	{
		fopen_s(&results,"myfile2.txt", "a");
		fprintf(results, "%s", log_file_buffer);
		fclose(results);
		m = log_file_buffer;
	}
}
//check to see if motion causes robots to collide
int find_collisions(int id, double x, double y)
{
	double two_r = 2 * radius;
	int i;
	if (x <= radius || x >= arena_width - radius || y <= radius || y >= arena_height - radius) return 1;
	double x_ulim = x + two_r;
	double x_llim = x - two_r;
	double y_ulim = y + two_r;
	double y_llim = y - two_r;
	for (i = 0;i < num_robots;i++)
	{
		if (i != id)
		{
			if (safe_distance[id][i])
			{
				safe_distance[id][i]--;
			}
			else
			{
				int dist_x = x - robots[i]->pos[0];
				int dist_y = y - robots[i]->pos[1];
				if (x_ulim > robots[i]->pos[0] && x_llim<robots[i]->pos[0] &&
					y_ulim>robots[i]->pos[1] && y_llim < robots[i]->pos[1]) //if not in the sqare limits, i dont even check the circular ones
				{

					double distance = sqrt(dist_x*dist_x + dist_y * dist_y);
					if (distance < two_r)
					{
						return 1;
					}
				}
				else
				{
					int bd = 0;
					if (abs(dist_x)>abs(dist_y))
					{
						bd = abs(dist_x);
					}
					else
					{
						bd = abs(dist_y);
					}
					if (bd > two_r+20)
					{
						double speed = robots[id]->speed + robots[i]->speed;
						if (speed > 0)
						{
							safe_distance[id][i] = (bd - (two_r + 20)) / speed;
						}
						else
						{
							safe_distance[id][i] = 1000000;
						}
						safe_distance[i][id] = safe_distance[id][i];
					}
				}
			}
		}
	}
	return 0;
}

void run_simulation_step()
{
	static int lastrun = 0;
	lastrun++;

	secs = lastrun / radius;

	int mins = secs / 60;
	secs = secs % 60;
	int hours = mins / 60;
	mins = mins % 60;
	sprintf_s(rt, 100, "%02d:%02d:%02d", hours, mins, secs);

	int i, j;

	double rotation_step = .05;//motion step size
							   //run a step of most or all robot controllers
	for (i = 0;i < num_robots;i++)
	{
		//run controller this time step with p_control_execute probability
		if ((rand())<(int)(p_control_execute*RAND_MAX))
		{
			robots[i]->robot_controller();
		}

	}

	int seed;
	seed = (rand() % shuffles) * num_robots;
	//let robots communicate
	for (i = 0;i < num_robots;i++)
	{
		int index = order[seed + i];
		robot *rs = robots[index];
		//if robot wants to communicate, send message to all robots within distance comm_range
		int c = 1;
		while (rs->tx_request)
		{
			if (rs->tx_request & c)
			{
				int ch = c;
				rs->tx_request -= c;//clear transmission flag
				for (j = 0;j < num_robots;j++)
				{
					robot *rd = robots[j];
					if (j != index)
					{
						double range = rs->comm_out_criteria(ch, rd->pos[0], rd->pos[1], safe_distance[index][j]);
						if (range)
						{
							void *msg = rs->get_message(ch);
							if (rd->comm_in_criteria(ch, rs->pos[0], rs->pos[1], range, msg))
							{
								rd->incoming_message_flag += ch;
							}
						}
					}
				}
			}
			c = c * 2;
		}
	}

	seed = (rand() % shuffles) * num_robots;
	//move robots
	for (i = 0;i < num_robots;i++)
	{
		int index = order[seed + i];
		robot *r = robots[index];

		double t = r->pos[2];
		double s = 0;
		switch (r->motor_command)
		{
		case 1:
		{
			t += r->motor_error;
			s = r->speed;
			break;
		}
		case 2:
		{
			t += rotation_step;
			s = r->speed;
			if (r->pos[2] > twicePi)
			{
				r->pos[2] -= twicePi;
			}
			break;
		}
		case 3:
		{
			t -= rotation_step;
			s = r->speed;
			if (r->pos[2] < 0)
			{
				r->pos[2] += twicePi;
			}
			break;
		}
		}
		double temp_x = s*cos(t) + r->pos[0];
		double temp_y = s*sin(t) + r->pos[1];
		if (find_collisions(index, temp_x, temp_y) == 0)
		{
			r->pos[0] = temp_x;
			r->pos[1] = temp_y;
		}
		r->pos[2] = t;
	}
	static int lastmin = 0;
	if (log_debug_info)
	{
		if (lastmin!=mins)
		{
			lastmin = mins;
			char buffer[255];
			for (int i = 0;i < num_robots;i++)
				log_info(robots[i]->get_debug_info(buffer, rt));
		}
	}
}

// Drawing routine.
void drawScene(void)
{
	run_simulation_step();
	//draws the arena
	glColor4f(0, 0, 0, 0);
	glRectd(0, 0, arena_width, arena_height);

	glutSetWindowTitle(rt);
	glEnable(GL_LINE_SMOOTH);
	glLineWidth(1.0);
	glBegin(GL_LINES);
	for (int i = 0; i <= radius; i++)
	{
		for (int j = 0;j < num_robots;j++)
		{
			glColor4f(robots[j]->color[0], robots[j]->color[1], robots[j]->color[2], 1.0);
			glVertex2f(robots[j]->pos[0]-i, robots[j]->pos[1]-ch[i]);
			glVertex2f(robots[j]->pos[0] -i, robots[j]->pos[1] + ch[i]);
			glVertex2f(robots[j]->pos[0] + i, robots[j]->pos[1] - ch[i]);
			glVertex2f(robots[j]->pos[0] + i, robots[j]->pos[1] + ch[i]);
		}
	}
	for (int j = 0;j < num_robots;j++)
	{
		glBegin(GL_LINES);
		glColor4f(0, 0, 0, 1.0);
		glVertex2f(robots[j]->pos[0], robots[j]->pos[1]);
		glVertex2f(robots[j]->pos[0] + cos(robots[j]->pos[2])*radius, robots[j]->pos[1] + sin(robots[j]->pos[2])*radius);
		if (robots[j]->dest[0] != -1)
		{
			glBegin(GL_LINES);
			glColor4f(1, 1, 1, 1.0);
			glVertex2f(robots[j]->pos[0], robots[j]->pos[1]);
			glVertex2f(robots[j]->dest[0], robots[j]->dest[1]);
		}
	}
	glEnd();
	glFlush();
	glutSwapBuffers();
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	cout << time_sim << endl;
	time_sim++;
}

// Initialization routine.
void setup(void)
{
	for (int i = 0;i < num_robots;i++)
		for (int j = 0;j < shuffles;j++)
			order[i + num_robots*j] = i;

	for (int i = 0;i < num_robots - 1;i++)
		for (int j = 0;j < shuffles;j++)
		{
			int index = j*num_robots + i;
			int r = index + rand() % (num_robots - i);
			int p = order[index];
			order[index] = order[r];
			order[r] = p;
		}
	for (int i = 0;i < num_robots;i++)
		for (int j = 0;j < num_robots;j++)
			safe_distance[i][j] = 0;
}

// OpenGL window reshape routine.
void resize(int w, int h)
{
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(0.0, 100.0, 0.0, 100.0, -1.0, 1.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

// Keyboard input processing routine.
void keyInput(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27:
		exit(0);
		break;
	case 'w'://up 
		view_y += 100;
		break;
	case 'a'://up 
		view_x -= 100;
		break;
	case 's'://up 
		view_y -= 100;
		break;
	case 'd'://up 
		view_x += 100;
		break;
	case '-':
		zoom = zoom*1.1;
		break;
	case '+':
		zoom = zoom*0.9;
		break;
	case '1':
		if (delay>0)
			delay--;
		break;
	case '2':
		delay++;
		break;
	default:
		break;
	}
}

void OnIdle(void) {

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	glOrtho(-zoom + view_x, view_x, -zoom + view_y, view_y, 0.0f, 1.0f);
	Sleep(delay);
	glutPostRedisplay();
}

//setup function for orbit behavior, make sure num_robots is 2
void setup_positions()
{
	int k = 0;
	int columns = (int)sqrt((num_robots * arena_width / arena_height));
	int rows = (int)(num_robots / columns);
	if (num_robots % columns) rows++;
	int horizontal_separation = arena_width / (columns + 1);
	int vertical_separation = (int)arena_height / (rows + 1);
	bool smart[num_robots];
	for (int i = 0;i < num_robots;i++)
		smart[i] = false;
	for (int i = 0;i < num_smart_robots;i++)
	{
		int x = 0;
		do
		{
			x = rand() * num_robots / RAND_MAX;
		} while (smart[x]);
		smart[x] = true;
	}
	for (int i = 0;i < num_robots;i++)
	{
		int c = i % columns + 1;
		int r = i / columns + 1;
		int hr = rand() % (horizontal_separation / 2) + horizontal_separation / 4;
		int x = c * horizontal_separation + hr;
		int vr = rand() % (vertical_separation / 2) + vertical_separation / 4;
		int y = r * vertical_separation + vr;
		if (smart[i])
		{
			robots[k] = new smart_robot();
		}
		else
		{
			robots[k] = new basic_robot();
		}
		double t = rand() * 2 * PI / RAND_MAX;
		robots[k]->robot_init(x, y, t);
		k++;
	}
}

// Main routine.
int main(int argc, char **argv)
{
	//seed random variable for different random behavior every time
	unsigned int t = time(NULL);
	
	sprintf_s(log_buffer, "random seed: %d\n", t);
	
	log_info(log_buffer);
	srand(t);
	
	//set the simulation time to 0
	time_sim = 0;

	//inital zoom and scroll positions
	zoom = arena_height;
	view_x = arena_width;
	view_y = arena_height;

	//place robots
	//setup_positions_gradient();
	setup_positions();

	setup();

	//do some open gl stuff
	if (showscene)
	{
		for (int i = 0;i < radius;i++)
		{
			ch[i] = sqrt(radius*radius-i*i);
		}
		glutInit(&argc, argv);
		glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);
		glutInitWindowSize(windowWidth, windowHeight);
		glutInitWindowPosition(0, 0);

		glMatrixMode(GL_PROJECTION);
		glLoadIdentity();
		glOrtho(0.0f, 1000, 1000, 0.0f, 0.0f, 1.0f);
		glClearColor(1.0, 1.0, 1.0, 0.0);

		glutCreateWindow("Kilobot simulator");
		glutDisplayFunc(drawScene);
		glutReshapeFunc(resize);
		glutIdleFunc(OnIdle);
		glutKeyboardFunc(keyInput);
		glutMainLoop();
	}
	return 0;
}
