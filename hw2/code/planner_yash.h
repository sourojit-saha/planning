#include <math.h>
#include <random>
#include <vector>
#include <array>
#include <algorithm>

#include <tuple>
#include <string>
#include <stdexcept>
#include <regex> // For regex and split logic
#include <iostream> // cout, endl
#include <fstream> // For reading/writing files
#include <assert.h> 

// added
#include <queue>
#include <ctime>
using namespace std;
//end

/* Input Arguments */
#define	MAP_IN      prhs[0]
#define	ARMSTART_IN	prhs[1]
#define	ARMGOAL_IN     prhs[2]
#define	PLANNER_ID_IN     prhs[3]

/* Planner Ids */
#define RRT         0
#define RRTCONNECT  1
#define RRTSTAR     2
#define PRM         3

/* Output Arguments */
#define	PLAN_OUT	plhs[0]
#define	PLANLENGTH_OUT	plhs[1]

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

#if !defined(MAX)
#define	MAX(A, B)	((A) > (B) ? (A) : (B))
#endif

#if !defined(MIN)
#define	MIN(A, B)	((A) < (B) ? (A) : (B))
#endif

#define PI 3.141592654

//the length of each link in the arm
#define LINKLENGTH_CELLS 10


// Some potentially helpful imports
using std::vector;
using std::array;
using std::string;
using std::runtime_error;
using std::tuple;
using std::make_tuple;
using std::tie;
using std::cout;
using std::endl;



struct Node {
	int id;
    double* angles;
    Node* parent;
    double cost;
};


class RRTplan{
    private:
        double* map;
        int x_size;
        int y_size;
        double* armstart_anglesV_rad;
        double* armgoal_anglesV_rad;
        int numofDOFs;
        double*** plan;
        int* planlength;
        vector<Node*>* tree;
        vector<Node*>* goal_tree;
        Node* closest;
        double* angles;

    public:
        RRTplan(double *map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs);
        void RRTplanner(double*** plan, int* planlength);
        void RRTStarplanner(double*** plan, int* planlength);
        void RRTConnectplanner(double*** plan, int* planlength);
        void generate_sample(double** sample_angles);
        double findClosest(double* angles, vector<Node*>* tree, Node** closest);
        bool IsValidTransition(Node* closest, double* angles, double dist, double step);
        bool isGoal(double* temp_angles, double* goal_angles);
        double inRadius(double* angles, vector<Node*>* tree, Node** closest, vector<Node*>* inrad, vector<double>* inraddist, double radius);
};

struct PRMNode {
    int id;
    double* angles;
    vector<PRMNode*>* closenodes;
    int cts;
    int ctg;
    PRMNode* closestart;
};

class PRMplan{
    private:
        double* map;
        int x_size;
        int y_size;
        double* armstart_anglesV_rad;
        double* armgoal_anglesV_rad;
        int numofDOFs;
        double*** plan;
        int* planlength;
        vector<PRMNode*>* rmap;
        double* angles;
        PRMNode* start;
        PRMNode* goal;

    public:
        PRMplan(double *map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs);
        void PRMplanner(double*** plan, int* planlength);
        void generate_samplePRM(double** angles);
        bool IsValidTransitionPRM(PRMNode* closest, double* angles, double dist, double step);
        void findclosenodesPRM(double* angles, vector<PRMNode*>* rmap, vector<PRMNode*>* nearNodes, vector<double>* nearNodeDistances, double radius);
        bool stgconnected(PRMNode* node);
};

