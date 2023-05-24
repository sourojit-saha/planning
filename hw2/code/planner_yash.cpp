/*=================================================================
 *
 * planner.c
 *
 *=================================================================*/
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
#include "planner_yash.h"
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

/// @brief 
/// @param filepath 
/// @return map, x_size, y_size
tuple<double*, int, int> loadMap(string filepath) {
	std::FILE *f = fopen(filepath.c_str(), "r");
	if (f) {
	}
	else {
		printf("Opening file failed! \n");
		throw runtime_error("Opening map file failed!");
	}
	int height, width;
	if (fscanf(f, "height %d\nwidth %d\n", &height, &width) != 2) {
		throw runtime_error("Invalid loadMap parsing map metadata");
	}
	
	////// Go through file and add to m_occupancy
	double* map = new double[height*width];

	double cx, cy, cz;
	for (int y = 0; y < height; y++) {
		for (int x = 0; x < width; x++) {
			char c;
			do {
				if (fscanf(f, "%c", &c) != 1) {
					throw runtime_error("Invalid parsing individual map data");
				}
			} while (isspace(c));
			if (!(c == '0')) { 
				map[y+x*width] = 1; // Note transposed from visual
			} else {
				map[y+x*width] = 0;
			}
		}
	}
	fclose(f);
	return make_tuple(map, width, height);
}

// Splits string based on deliminator
vector<string> split(const string& str, const string& delim) {   
		// https://stackoverflow.com/questions/14265581/parse-split-a-string-in-c-using-string-delimiter-standard-c/64886763#64886763
		const std::regex ws_re(delim);
		return { std::sregex_token_iterator(str.begin(), str.end(), ws_re, -1), std::sregex_token_iterator() };
}


double* doubleArrayFromString(string str) {
	vector<string> vals = split(str, ",");
	double* ans = new double[vals.size()];
	for (int i = 0; i < vals.size(); ++i) {
		ans[i] = std::stod(vals[i]);
	}
	return ans;
}

bool equalDoubleArrays(double* v1, double *v2, int size) {
    for (int i = 0; i < size; ++i) {
        if (abs(v1[i]-v2[i]) > 1e-3) {
            cout << endl;
            return false;
        }
    }
    return true;
}

typedef struct {
	int X1, Y1;
	int X2, Y2;
	int Increment;
	int UsingYIndex;
	int DeltaX, DeltaY;
	int DTerm;
	int IncrE, IncrNE;
	int XIndex, YIndex;
	int Flipped;
} bresenham_param_t;


void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size) {
	double cellsize = 1.0;
	//take the nearest cell
	*pX = (int)(x/(double)(cellsize));
	if( x < 0) *pX = 0;
	if( *pX >= x_size) *pX = x_size-1;

	*pY = (int)(y/(double)(cellsize));
	if( y < 0) *pY = 0;
	if( *pY >= y_size) *pY = y_size-1;
}


void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params) {
	params->UsingYIndex = 0;

	if (fabs((double)(p2y-p1y)/(double)(p2x-p1x)) > 1)
		(params->UsingYIndex)++;

	if (params->UsingYIndex)
		{
			params->Y1=p1x;
			params->X1=p1y;
			params->Y2=p2x;
			params->X2=p2y;
		}
	else
		{
			params->X1=p1x;
			params->Y1=p1y;
			params->X2=p2x;
			params->Y2=p2y;
		}

	 if ((p2x - p1x) * (p2y - p1y) < 0)
		{
			params->Flipped = 1;
			params->Y1 = -params->Y1;
			params->Y2 = -params->Y2;
		}
	else
		params->Flipped = 0;

	if (params->X2 > params->X1)
		params->Increment = 1;
	else
		params->Increment = -1;

	params->DeltaX=params->X2-params->X1;
	params->DeltaY=params->Y2-params->Y1;

	params->IncrE=2*params->DeltaY*params->Increment;
	params->IncrNE=2*(params->DeltaY-params->DeltaX)*params->Increment;
	params->DTerm=(2*params->DeltaY-params->DeltaX)*params->Increment;

	params->XIndex = params->X1;
	params->YIndex = params->Y1;
}

void get_current_point(bresenham_param_t *params, int *x, int *y) {
	if (params->UsingYIndex) {
        *y = params->XIndex;
        *x = params->YIndex;
        if (params->Flipped)
            *x = -*x;
    }
	else {
        *x = params->XIndex;
        *y = params->YIndex;
        if (params->Flipped)
            *y = -*y;
    }
}

int get_next_point(bresenham_param_t *params) {
	if (params->XIndex == params->X2) {
        return 0;
    }
	params->XIndex += params->Increment;
	if (params->DTerm < 0 || (params->Increment < 0 && params->DTerm <= 0))
		params->DTerm += params->IncrE;
	else {
        params->DTerm += params->IncrNE;
        params->YIndex += params->Increment;
	}
	return 1;
}



int IsValidLineSegment(double x0, double y0, double x1, double y1, double*	map,
			 int x_size, int y_size) {
	bresenham_param_t params;
	int nX, nY; 
	short unsigned int nX0, nY0, nX1, nY1;

	//printf("checking link <%f %f> to <%f %f>\n", x0,y0,x1,y1);
		
	//make sure the line segment is inside the environment
	if(x0 < 0 || x0 >= x_size ||
		x1 < 0 || x1 >= x_size ||
		y0 < 0 || y0 >= y_size ||
		y1 < 0 || y1 >= y_size)
		return 0;

	ContXY2Cell(x0, y0, &nX0, &nY0, x_size, y_size);
	ContXY2Cell(x1, y1, &nX1, &nY1, x_size, y_size);

	//printf("checking link <%d %d> to <%d %d>\n", nX0,nY0,nX1,nY1);

	//iterate through the points on the segment
	get_bresenham_parameters(nX0, nY0, nX1, nY1, &params);
	do {
		get_current_point(&params, &nX, &nY);
		if(map[GETMAPINDEX(nX,nY,x_size,y_size)] == 1)
			return 0;
	} while (get_next_point(&params));

	return 1;
}

int IsValidArmConfiguration(double* angles, int numofDOFs, double*	map,
			 int x_size, int y_size) {
    double x0,y0,x1,y1;
    int i;
		
	 //iterate through all the links starting with the base
	x1 = ((double)x_size)/2.0;
	y1 = 0;
	for(i = 0; i < numofDOFs; i++){
		//compute the corresponding line segment
		x0 = x1;
		y0 = y1;
		x1 = x0 + LINKLENGTH_CELLS*cos(2*PI-angles[i]);
		y1 = y0 - LINKLENGTH_CELLS*sin(2*PI-angles[i]);

		//check the validity of the corresponding line segment
		if(!IsValidLineSegment(x0,y0,x1,y1,map,x_size,y_size))
			return 0;
	}    
	return 1;
}

static void planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	//no plan by default
	*plan = NULL;
	*planlength = 0;
		
    //for now just do straight interpolation between start and goal checking for the validity of samples

    double distance = 0;
    int i,j;
    for (j = 0; j < numofDOFs; j++){
        if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
            distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    }
    int numofsamples = (int)(distance/(PI/20));
    if(numofsamples < 2){
        printf("The arm is already at the goal\n");
        return;
    }
	int countNumInvalid = 0;
    *plan = (double**) malloc(numofsamples*sizeof(double*));
    for (i = 0; i < numofsamples; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
        for(j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
        }
        if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
			++countNumInvalid;
        }
    }
	printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
    *planlength = numofsamples;
    
    return;
}

// additions

RRTplan::RRTplan(double *map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs){
	this->map = map;
	this->x_size = x_size;
	this->y_size = y_size;
	this->armstart_anglesV_rad = new double[numofDOFs];
	this->armgoal_anglesV_rad = new double[numofDOFs];
	this->numofDOFs = numofDOFs;
	
	for(int i=0; i<numofDOFs; i++){
		this->armstart_anglesV_rad[i] = armstart_anglesV_rad[i];
		this->armgoal_anglesV_rad[i] = armgoal_anglesV_rad[i];
	}

	tree = new vector<Node*>();
	Node* start = new Node;
	start->id = 1;
	start->angles = armstart_anglesV_rad;
	start->parent = 0;
	tree->push_back(start);
	closest = new Node;
	angles = new double[numofDOFs];
}

void RRTplan::generate_sample(double** sample_angles){
	for (int i=0; i<numofDOFs; i++){
		(*sample_angles)[i] = (rand() % 360) * (PI/180);
	}
}

double RRTplan::findClosest(double* angles, vector<Node*>* tree, Node** closest){
	double leastdist = pow(2*PI, 2)*numofDOFs;
	for (auto & i : *tree){
		double dist = 0;
		for (int j=0; j<numofDOFs; j++){
			dist += pow(fabs((i)->angles[j] - angles[j]), 2);
		}
		if (dist < leastdist){
			leastdist = dist;
			*closest = i;
		}
	}
	leastdist = sqrt(leastdist);
	return leastdist;
}

bool RRTplan::IsValidTransition(Node* closest, double* angles, double dist, double step){
	double* temp = new double[numofDOFs];
	int numofsteps = (int) (dist/step);
	for (int i=0; i<=numofsteps; i++){
		for (int j=numofDOFs-1; j>=0; j--){
			double diff = angles[j] - closest->angles[j];
			temp[j] = closest->angles[j] + i * step * diff;
		}
		if (!IsValidArmConfiguration(temp, numofDOFs, map, x_size, y_size)){
			return false;
		}
	}
	return true;
}

bool RRTplan::isGoal(double* temp_angles, double* goal_angles){
	double diff = 0;
	for (int i=0; i<numofDOFs; i++){
		diff += temp_angles[i] - goal_angles[i];
	}
	diff = round( diff * 1000.0 ) / 1000.0;
	if(diff == 0.0){
		return true;
	}
	return false;
}

double RRTplan::inRadius(double* angles, vector<Node*>* tree, Node** closest, vector<Node*>* inrad, vector<double>* inraddist, double radius){
	double leastdist = pow(2*PI, 2)*numofDOFs;
	for (int i = 0; i < tree->size(); i++) {
        Node* temp = (*tree)[i];
        double dist = 0;
        for (int j = 0; j < numofDOFs; j++) {
            dist += pow(fabs(angles[j] - temp->angles[j]), 2);
        }
        if(dist < leastdist) {
            leastdist = dist;
			*closest = temp;
        }
    }
	leastdist = sqrt(leastdist);
	return leastdist;
}

void RRTplan::RRTplanner(double*** plan, int* planlength){
	clock_t t_start = clock();
	*plan = NULL;
	*planlength = 0;
	double eps = PI/4;
	double step = 0.2;
	int max_samples = 500000;

	for(int k=0; k<max_samples; k++){
		double* angles = new double[numofDOFs];
		if(k%99 == 0){
			for(int i=0; i<numofDOFs; i++){
				angles[i] = armgoal_anglesV_rad[i];
			}
		}
		else {
            generate_sample(&angles);
        }
        if(!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
            continue;
		}

		double least_dist = findClosest(angles, tree, &closest);
		Node* temp = new Node;
		temp->parent = closest;
		temp->id = closest->id + 1;
		if (least_dist <= eps){
			temp->angles = angles;
		}
		else{
			for(int j=0; j<numofDOFs; j++){
				angles[j] = closest->angles[j] + eps * ((angles[j] - closest->angles[j])/least_dist);
			}
			least_dist = eps;
			temp->angles = angles;
		}
		if(!IsValidTransition(closest, angles, least_dist, step)){
			continue;
		}
				
		tree->push_back(temp);

		if(isGoal(angles, armgoal_anglesV_rad)){
			*plan = (double**) malloc(temp->id * sizeof(double*));
			*planlength = temp->id;

			for (int i = *planlength - 1; i >= 0; i--) {
				(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
				for(int j = 0; j < numofDOFs; j++){
					(*plan)[i][j] = temp->angles[j];
				}
				temp = temp->parent;
			}
			cout<<"Planner time: "<<(clock() - t_start)/(double)CLOCKS_PER_SEC<<endl;
			return;
		}
	}
}



void RRTplan::RRTConnectplanner(double*** plan, int* planlength){
	clock_t t_start = clock();
	*plan = NULL;
	*planlength = 0;
	double eps = PI/15;
	double step = 0.2;
	int max_samples = 5000000;
	goal_tree = new vector<Node*>();
	Node* goal = new Node;
	goal->id = 1;
	goal->parent = 0;
	goal->angles = armgoal_anglesV_rad;
	goal_tree->push_back(goal);
	// Node* closest;
	vector<Node*>* present_tree;
	present_tree = tree;
	int isStart = 1;

	for(int k=0; k<max_samples; k++){
		// cout<<"k= "<<k<<endl;
		double* angles = new double[numofDOFs];
		generate_sample(&angles);
		if(!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
            continue;
		}
		double least_dist = findClosest(angles, present_tree, &closest);
		Node* temp = new Node();
		temp->parent = closest;
		temp->id = closest->id + 1;
		if (least_dist <= eps){
			temp->angles = angles;
		}
		else{
			for(int j=0; j<numofDOFs; j++){
				angles[j] = closest->angles[j] + eps * ((angles[j] - closest->angles[j])/least_dist);
			}
			least_dist = eps;
			temp->angles = angles;
		}
		// cout<<"angle test: "<<temp->angles[1]<<endl;
		if(IsValidTransition(closest, angles, least_dist, step)){
            present_tree->push_back(temp);
		}
		if(!isStart){
			present_tree = tree;
			isStart = 1;
		}
		else{
			present_tree = goal_tree;
			isStart = 0;
		}
		if(IsValidTransition(closest, angles, least_dist, step)){
			cout<<"In valid"<<endl;
			least_dist = findClosest(angles, present_tree, &closest);
			while(eps < least_dist){
				Node* temp2 = new Node;
				for(int j=0; j<numofDOFs; j++){
				angles[j] = closest->angles[j] + eps * ((angles[j] - closest->angles[j])/least_dist);
				// cout<<"angle test2: "<<closest->angles[j]<<endl;
				}
				temp2->angles = angles;
				temp2->parent = closest;
				temp2->id = closest->id+1;
				// cout<<"temp id: "<<temp2->id<<endl;
				if(!IsValidTransition(closest, angles, least_dist, step)){
					break;
				}
				present_tree->push_back(temp2);
				least_dist = least_dist - eps;
				closest = temp2;

			}
			if(least_dist <= eps){
				cout<<"Goal"<<endl;
				Node* temp_start = new Node;
				Node* temp_goal = new Node;
				if(!isStart){
					temp_goal = closest;
					temp_start = temp;
				}
				else{
					temp_start = closest;
					temp_goal = temp;
				}
				*planlength = int(temp_start->id + temp_goal->id);
				*plan = (double**) malloc(*planlength * sizeof(double*));

				Node* iter = new Node;
				iter = temp_start;
				for(int i=temp_start->id-1; i>=0; i--){
					(*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
					for(int j = 0; j < numofDOFs; j++){
						(*plan)[i][j] = iter->angles[j];
					}
					iter = iter->parent;
					cout<<"iter parent: "<<iter->parent<<endl;
				}
				iter = temp_goal;
				int a = temp_start->id;
				for(int i=0; i<temp_goal->id; i++){
					(*plan)[i+a] = (double*) malloc(numofDOFs * sizeof(double));
					for(int j = 0; j < numofDOFs; j++){
						(*plan)[i+a][j] = iter->angles[j];
					}
					iter = iter->parent;
				}
				cout<<"Planner time: "<<(clock() - t_start)/(double)CLOCKS_PER_SEC<<endl;
				return;

			}
		}
		
	}

}



void RRTplan::RRTStarplanner(double*** plan, int* planlength){
	clock_t t_start = clock();
	*plan = NULL;
	*planlength = 0;
	double eps = PI/4;
	double step = 0.2;
	Node* goal;
	int max_samples = 500000;


	for(int k=0; k<max_samples; k++){
		double* angles = new double[numofDOFs];
		if(k%5 == 0){
			for(int i=0; i<numofDOFs; i++){
				angles[i] = armgoal_anglesV_rad[i];
			}
		}
		else {
            generate_sample(&angles);
        }
        if(!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
			continue;
		}
        double r = min(pow((500 * log(tree->size())/tree->size()), (1.0/numofDOFs)), eps);
		vector<Node*>* inradius = new vector<Node*>();
		vector<double>* inradius_dist = new vector<double>();

		double least_dist = inRadius(angles, tree, &closest, inradius, inradius_dist, r);
		Node* temp = new Node;
		temp->parent = closest;
		temp->id = closest->id + 1;
		if(least_dist > eps){
			for(int j=0; j<numofDOFs; j++){
				angles[j] = closest->angles[j] + eps * (angles[j] - closest->angles[j])/least_dist;
			}
			least_dist = eps;
			temp->angles = angles;
		}

		else{
			temp->angles = angles;
		}

		if(!IsValidTransition(closest, angles, least_dist, step)){
			continue;
		}
		vector<int>* inradobs = new vector<int>();
		Node* leastNode = closest;
		double leastCost = closest->cost + least_dist;
		int length = inradius_dist->size();
		for(int i=0; i<length; i++){
			inradobs->push_back(int(IsValidTransition((*inradius)[i], angles, (*inradius_dist)[i], step)));
			if(IsValidTransition((*inradius)[i], angles, (*inradius_dist)[i], step)){
				double cost = (*inradius)[i]->cost + (*inradius_dist)[i];
				if(cost < leastCost){
					leastNode = (*inradius)[i];
					leastCost = cost;
				}
			}
		}
		
		temp->cost = leastCost;
		temp->id = leastNode->id + 1;
		temp->parent = leastNode;
		temp->angles = angles;
		tree->push_back(temp);

		for (int i = 0; i < inradius->size(); i++) {
			if ((*inradius)[i] != leastNode){
				double cost = temp->cost + (*inradius_dist)[i];
				if ((*inradobs)[i] && ((*inradius)[i]->cost > cost)) {
					(*inradius)[i]->cost = cost;
					(*inradius)[i]->parent = temp;
					(*inradius)[i]->id = temp->id + 1;
				}
			}
		}
		
		if (isGoal(angles, armgoal_anglesV_rad)) { 
			goal = temp;
			break;
		}
	}
	Node* new_temp = goal;
	int planLength = 1;
	while(new_temp->parent != 0){
		planLength++;
		new_temp = new_temp->parent;
	}
	*plan = (double**) malloc(planLength * sizeof(double*));
    *planlength = planLength;

	for (int i = *planlength - 1; i >= 0; i--) {
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = goal->angles[j];
        }
        goal = goal->parent;
    }
	cout<<"Planner time: "<<(clock() - t_start)/(double)CLOCKS_PER_SEC<<endl;
	return;
}



PRMplan::PRMplan(double *map, int x_size, int y_size, double* armstart_anglesV_rad, double* armgoal_anglesV_rad, int numofDOFs){
	this->map = map;
	this->x_size = x_size;
	this->y_size = y_size;
	this->armstart_anglesV_rad = new double[numofDOFs];
	this->armgoal_anglesV_rad = new double[numofDOFs];
	this->numofDOFs = numofDOFs;
	
	for(int i=0; i<numofDOFs; i++){
		this->armstart_anglesV_rad[i] = armstart_anglesV_rad[i];
		this->armgoal_anglesV_rad[i] = armgoal_anglesV_rad[i];
	}

	rmap = new vector<PRMNode*>();

	start = new PRMNode;
	start->id = 1;
	start->angles = armstart_anglesV_rad;
	start->closenodes = new vector<PRMNode*>();
	start->cts = 1;
	start->ctg = 0;
	rmap->push_back(start);

	goal = new PRMNode;
	goal->id = -1;
	goal->angles = armgoal_anglesV_rad;
	goal->closenodes = new vector<PRMNode*>();
	goal->cts = 0;
	goal->ctg = 1;
	rmap->push_back(goal);

	angles = new double[numofDOFs];

}

void PRMplan::generate_samplePRM(double** angles){
	for (int i=0; i<numofDOFs; i++){
		(*angles)[i] = (rand() % 360) * (PI/180);
	}
}

bool PRMplan::IsValidTransitionPRM(PRMNode* closest, double* angles, double dist, double step){
	double* temp = new double[numofDOFs];
	int numofsteps = (int) (dist/step);
	for (int i=0; i<=numofsteps; i++){
		for (int j=numofDOFs-1; j>=0; j--){
			double diff = angles[j] - closest->angles[j];
			temp[j] = closest->angles[j] + i * step * diff;
		}
		if (!IsValidArmConfiguration(temp, numofDOFs, map, x_size, y_size)){
			return false;
		}
	}
	return true;
}

void PRMplan::findclosenodesPRM(double* angles, vector<PRMNode*>* rmap, vector<PRMNode*>* nearNodes, vector<double>* nearNodedist, double radius) {
    radius = radius * radius;
	for (auto & i : *rmap){
		double dist = 0;
		for (int j=0; j<numofDOFs; j++){
			dist += pow(fabs((i)->angles[j] - angles[j]), 2);
		}
		if (dist <= radius){
			nearNodes->push_back(i);
			nearNodedist->push_back(sqrt(dist));
		}
	}
}

bool PRMplan::stgconnected(PRMNode* node) {
    bool connected;
	// (node->cts && node->ctg);
	if(node->cts==1 && node->ctg==1){
		connected = true;
	}
	else{
		connected = false;
	}
    PRMNode* currNeighbor;
    int temp_cts;
    int temp_ctg;

	for (auto & i : *(node->closenodes)){
		temp_cts = i->cts;
		temp_ctg = i->ctg;
		i->cts = (i->cts || node->cts);
		i->ctg = (i->ctg || node->ctg);
		if(temp_cts != node->cts || temp_ctg != node->ctg){
			if(stgconnected(i)){
				connected = true;
			}
		}
	}
    return connected;
}

void PRMplan::PRMplanner(double*** plan, int* planlength){
	clock_t t_start = clock();

	*plan = NULL;
	*planlength = 0;
	double eps = PI/4;
	double step = 0.2;
	int max_samples = 500000;

	vector<PRMNode*>* nearNodes;
	vector<double>* nearNodedist;
	

	for(int k=0; k<max_samples; k++){
		angles = new double[numofDOFs];
		generate_samplePRM(&angles);
		if(!IsValidArmConfiguration(angles, numofDOFs, map, x_size, y_size)){
			continue;
		}

		double r = min(pow((500 * log(rmap->size())/rmap->size()), (1.0/numofDOFs)), eps);
		
		nearNodes = new vector<PRMNode*>();
		nearNodedist = new vector<double>();

		findclosenodesPRM(angles, rmap, nearNodes, nearNodedist, r);

		PRMNode* temp = new PRMNode;
        temp->id = -1;
		temp->angles = angles;
        temp->cts = 0;
        temp->ctg = 0;
        temp->closenodes = new vector<PRMNode*>();
        rmap->push_back(temp);

		PRMNode* temp2 = new PRMNode;
		double temp2dist;
		for(int i = 0; i < nearNodes->size(); i++) {
            temp2 = (*nearNodes)[i];
            temp2dist = (*nearNodedist)[i];
            if(IsValidTransitionPRM(temp2, angles, temp2dist, step)){
				temp2->closenodes->push_back(temp);
            	temp->closenodes->push_back(temp2);	
			}
        }

		if(!stgconnected(temp)){
			continue;;
		}
		break;
	}

	queue<PRMNode*> path;
    path.push(start);
            
    PRMNode* iter;
	PRMNode* closest;
    while(!path.empty()) {
        iter = path.front();
        path.pop();
        if (iter == goal) {
            break;
        }
        for(int i = 0; i < iter->closenodes->size(); i++) {
            closest = (*(iter->closenodes))[i];
            if (closest->id != -1) {
                continue;
            }
			closest->closestart = iter;
			closest->id = iter->id + 1;
			path.push(closest);
        }
    }

    *plan = (double**) malloc(iter->id * sizeof(double*));
    *planlength = iter->id;

    for (int i = *planlength - 1; i >= 0; i--) {
        (*plan)[i] = (double*) malloc(numofDOFs * sizeof(double));
        for(int j = 0; j < numofDOFs; j++){
            (*plan)[i][j] = iter->angles[j];
        }
        iter = iter->closestart;
    }
    cout<<"Planning time: "<<(clock() - t_start ) / (double) CLOCKS_PER_SEC<<endl;

}


//end



/** Your final solution will be graded by an grading script which will
 * send the default 6 arguments:
 *    map, numOfDOFs, commaSeparatedStartPos, commaSeparatedGoalPos, 
 *    whichPlanner, outputFilePath
 * An example run after compiling and getting the planner.out executable
 * >> ./planner.out map1.txt 5 1.57,0.78,1.57,0.78,1.57 0.392,2.35,3.14,2.82,4.71 0 output.txt
 * See the hw handout for full information.
 * If you modify this for testing (e.g. to try out different hyper-parameters),
 * make sure it can run with the original 6 commands.
 * Programs that do not will automatically get a 0.
 * */
int main(int argc, char** argv) {
	double* map;
	int x_size, y_size;

	tie(map, x_size, y_size) = loadMap(argv[1]);
	const int numOfDOFs = std::stoi(argv[2]);
	double* startPos = doubleArrayFromString(argv[3]);
	double* goalPos = doubleArrayFromString(argv[4]);
	int whichPlanner = std::stoi(argv[5]);
	string outputFile = argv[6];

	if(!IsValidArmConfiguration(startPos, numOfDOFs, map, x_size, y_size)||
			!IsValidArmConfiguration(goalPos, numOfDOFs, map, x_size, y_size)) {
		throw runtime_error("Invalid start or goal configuration!\n");
	}

	///////////////////////////////////////
	//// Feel free to modify anything below. Be careful modifying anything above.
	srand(0);
	double** plan = NULL;
	int planlength = 0;

	if(whichPlanner == 0){
		cout<<"RRT planner"<<endl;
		RRTplan obj = RRTplan(map, x_size, y_size, startPos, goalPos, numOfDOFs);
		obj.RRTplanner(&plan, &planlength);
	}

	else if ((whichPlanner == 1))
	{
		cout<<"RRT Connect planner"<<endl;
		RRTplan obj1 = RRTplan(map, x_size, y_size, startPos, goalPos, numOfDOFs);
		obj1.RRTConnectplanner(&plan, &planlength);
	}

	else if (whichPlanner == 2)
	{
		cout<<"RRT Star planner"<<endl;
		RRTplan obj2 = RRTplan(map, x_size, y_size, startPos, goalPos, numOfDOFs);
		obj2.RRTStarplanner(&plan, &planlength);
	}
	
	else if (whichPlanner == 3)
	{
		cout<<"PRM planner"<<endl;
		PRMplan obj3 = PRMplan(map, x_size, y_size, startPos, goalPos, numOfDOFs);
		obj3.PRMplanner(&plan, &planlength);
	}
	else{
		cout<<"No planner"<<endl;
	}
	


	//// Feel free to modify anything above.
	//// If you modify something below, please change it back afterwards as my 
	//// grading script will not work and you will recieve a 0.
	///////////////////////////////////////

    // Your solution's path should start with startPos and end with goalPos
    if (!equalDoubleArrays(plan[0], startPos, numOfDOFs) || 
    	!equalDoubleArrays(plan[planlength-1], goalPos, numOfDOFs)) {
		throw std::runtime_error("Start or goal position not matching");
	}

	/** Saves the solution to output file
	 * Do not modify the output log file output format as it is required for visualization
	 * and for grading.
	 */
	std::ofstream m_log_fstream;
	m_log_fstream.open(outputFile, std::ios::trunc); // Creates new or replaces existing file
	if (!m_log_fstream.is_open()) {
		throw std::runtime_error("Cannot open file");
	}
	m_log_fstream << argv[1] << endl; // Write out map name first
	/// Then write out all the joint angles in the plan sequentially
	for (int i = 0; i < planlength; ++i) {
		for (int k = 0; k < numOfDOFs; ++k) {
			m_log_fstream << plan[i][k] << ",";
		}
		m_log_fstream << endl;
	}
}
