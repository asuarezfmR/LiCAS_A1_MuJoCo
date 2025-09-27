/*
 * LiCAS A1 MuJoCo simulation and control
 *
 * Author: Alejandro Suarez, asuarezfm@us.es
 *
 * Date: 23 March 2025
 */
 
// Standard library
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <math.h>

// MuJoCo library and GLFW for 3D visualization
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

// Constant definition
#define NUM_ARM_JOINTS 4
#define PI				3.1415

// Namespaces
using namespace std;


// Global variables: MuJoCo model, data, and camera
mjModel* m = NULL;	// MuJoCo model
mjData* d = NULL;	// MuJoCo data
mjvCamera cam;		// Abstract camera
mjvOption opt;		// Visualization options
mjvScene scn;		// Abstract scene
mjrContext con;		// Custom GPU context

// Global variables: GLFW window
GLFWwindow* window;

// Global variables: mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;


// Function declaration
void initVisualization();

// Callback declaration
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouse_button(GLFWwindow* window, int button, int act, int mods);
void mouse_move(GLFWwindow* window, double xpos, double ypos);
void scroll(GLFWwindow* window, double xoffset, double yoffset);



// Main function
int main(int argc, char ** argv)
{

	if(argc != 2)
	{
		cerr << "ERROR: specify model xml file" << endl;
		return 1;
	}

	// Load and compile MuJoCo model from the XML file
	char errorMsg[1000] = "Could not load binary model";
	if(std::strlen(argv[1]) > 4 && !std::strcmp(argv[1]+std::strlen(argv[1])-4, ".mjb"))
		m = mj_loadModel(argv[1], 0);
	else
		m = mj_loadXML(argv[1], 0, errorMsg, 1000);
  
	if(!m)
	mju_error("Load model error: %s", errorMsg);


	// Make MuJoCo data
	d = mj_makeData(m);


	// Init visualization
	initVisualization();

	// Get the indices of the actoators
	int q1_motor_id[NUM_ARM_JOINTS];	// Left arm MuJoCo actuator indices
	int q2_motor_id[NUM_ARM_JOINTS];	// Right arm MuJoCo actuator indices
	char jointMotorName[32];
	for(int k = 0; k < NUM_ARM_JOINTS; k++)
	{
		sprintf(jointMotorName, "q1%d_motor", k+1);
		q1_motor_id[k] = mj_name2id(m, mjOBJ_ACTUATOR, jointMotorName);
		sprintf(jointMotorName, "q2%d_motor", k+1);
		q2_motor_id[k] = mj_name2id(m, mjOBJ_ACTUATOR, jointMotorName);
		if (q1_motor_id[k] == -1 || q2_motor_id[k] == -1)
		{
			cerr << "ERROR: could not find actuator motors. Check actuator names in XML model file." << endl;
			mj_deleteData(d);
			mj_deleteModel(m);
			return 1;
		}
	}

	// Get the joint index
	int q1_id[NUM_ARM_JOINTS];	// Left arm MuJoCo joint indices
	int q2_id[NUM_ARM_JOINTS];	// Right arm MuJoCo joint indices
	char jointName[32];
	for(int k = 0; k < NUM_ARM_JOINTS; k++)
	{
		sprintf(jointName, "q1%d", k+1);
		q1_id[k] = mj_name2id(m, mjOBJ_JOINT, jointName);
		sprintf(jointName, "q2%d", k+1);
		q2_id[k] = mj_name2id(m, mjOBJ_JOINT, jointName);
		if (q1_id[k] == -1 || q2_id[k] == -1)
		{
			cerr << "ERROR: could not find joints. Check joint names in XML model file." << endl;
			mj_deleteData(d);
			mj_deleteModel(m);
			return 1;
		}
	}
	
	// Create log file
	ofstream logFile;
	logFile.open("SimulationLogFile.txt");
  

	// Run main loop, target real-time simulation and 60 fps rendering
	while(!glfwWindowShouldClose(window))
	{
		// Advance interactive simulation for 1/60 sec
		// Assuming MuJoCo can simulate faster than real-time, which it usually can,
		// this loop will finish on time for the next frame to be rendered at 60 fps.
		// Otherwise add a cpu timer and exit this loop when it is time to render.
    
    	// Get time stamp at the begining of the iteration
		mjtNum simstart = d->time;
    
		// Read the current joint angles
		double q1_ref[NUM_ARM_JOINTS];
		double q1[NUM_ARM_JOINTS];
		double dq1[NUM_ARM_JOINTS];
		double q1_e[NUM_ARM_JOINTS];
		double tau1[NUM_ARM_JOINTS];
		
		double q2_ref[NUM_ARM_JOINTS];
		double q2[NUM_ARM_JOINTS];
		double dq2[NUM_ARM_JOINTS];
		double q2_e[NUM_ARM_JOINTS];
		double tau2[NUM_ARM_JOINTS];
		
		for(int k = 0; d->time > 2.5 && k < NUM_ARM_JOINTS; k++)
		{
			// Get the joint position
			q1[k] = d->qpos[m->jnt_qposadr[q1_id[k]]];
			q2[k] = d->qpos[m->jnt_qposadr[q2_id[k]]];
			
			// Get the joint speed
			dq1[k] = d->qvel[m->jnt_dofadr[q1_id[k]]];
			dq2[k] = d->qvel[m->jnt_dofadr[q2_id[k]]];
			
			// Generate the joint position reference
			if(k == 0)
			{
				q1_ref[k] = -60*PI/180.0;
				q2_ref[k] = -60*PI/180.0;
			}
			else if(k == 1)
			{
				q1_ref[k] = 30*PI/180.0;
				q2_ref[k] = -30*PI/180.0;
			}
			else if(k == 2)
			{
				q1_ref[k] = -60*PI/180.0;
				q2_ref[k] = 45*PI/180.0;
			}
			else if(k == 3)
			{
				q1_ref[k] = -90*PI/180.0;
				q2_ref[k] = -60*PI/180.0;
			}
			
			// Compute the joint error
			q1_e[k] = q1_ref[k] - q1[k];
			q2_e[k] = q2_ref[k] - q2[k];
			
			// Compute the torque
			// NOTE: This is the motor torque applied before gearbox.
			// Check gearbox ratio in XML model (100 by default)
			double kp = 0.01;
			double kd = 0.0;
			tau1[k] = kp * q1_e[k];
			tau2[k] = kp * q2_e[k];
			
			// Apply the torque to MuJoCo actuator
			d->ctrl[q1_motor_id[k]] = tau1[k];
			d->ctrl[q2_motor_id[k]] = tau2[k];
		}


		// Update model state until the visualization period at 60 FPS is complete 
		while (d->time - simstart < 1.0/60.0)
			mj_step(m, d);


		// Print data
		/*
		cout << "Joint angle:\t{" << 57.296*q11 << ", " << 57.296*q21 << "} [deg]" << endl;
		cout << "Joint speed:\t{" << 57.296*dq21 << ", " << 57.296*dq21 << "} [deg/s]" << endl;
		cout << "---" << endl;
		*/
    
		// get framebuffer viewport
		mjrRect viewport = {0, 0, 0, 0};
		glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

		// update scene and render
		mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
		mjr_render(viewport, &scn, &con);

    	// swap OpenGL buffers (blocking call due to v-sync)
    	glfwSwapBuffers(window);

    	// process pending GUI events, call GLFW callbacks
    	glfwPollEvents();
	}
	
	
	// Close log file
	logFile.close();

	// Free visualization resources
	mjv_freeScene(&scn);
	mjr_freeContext(&con);
  
	// Free MuJoCo resources
	mj_deleteData(d);
	mj_deleteModel(m);

	// Terminate GLFW (crashes with Linux NVidia drivers)
#if defined(__APPLE__) || defined(_WIN32)
	glfwTerminate();
#endif


  return 0;
}



// Init GLFW for 3D visualization
void initVisualization()
{	
	if(!glfwInit())
		mju_error("Could not initialize GLFW");

	// Create window, make OpenGL context current, request v-sync
	window = glfwCreateWindow(1200, 900, "Viewer", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

	// Initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultOption(&opt);
	mjv_defaultScene(&scn);
	mjr_defaultContext(&con);

	// create scene and context
	mjv_makeScene(m, &scn, 2000);
	mjr_makeContext(m, &con, mjFONTSCALE_150);

	// install GLFW mouse and keyboard callbacks
	glfwSetKeyCallback(window, keyboard);
	glfwSetCursorPosCallback(window, mouse_move);
	glfwSetMouseButtonCallback(window, mouse_button);
	glfwSetScrollCallback(window, scroll);
}


// Keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	// backspace: reset simulation
	if(act == GLFW_PRESS && key == GLFW_KEY_BACKSPACE)
	{
		mj_resetData(m, d);
		mj_forward(m, d);
	}
}


// Mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	// update button state
	button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
	button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
	button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

	// update mouse position
	glfwGetCursorPos(window, &lastx, &lasty);
}


// Mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	// No buttons down: nothing to do
	if (!button_left && !button_middle && !button_right)
	{
		return;
	}

	// Compute mouse displacement, save
	double dx = xpos - lastx;
	double dy = ypos - lasty;
	lastx = xpos;
	lasty = ypos;

	// Get current window size
	int width, height;
	glfwGetWindowSize(window, &width, &height);

	// Get shift key state
	bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                    glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

	// Determine action based on mouse button
	mjtMouse action;
	if (button_right) {
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	} else if (button_left) {
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	} else {
		action = mjMOUSE_ZOOM;
	}

	// Move camera
	mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}


// Scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	// emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}




