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

#include <cfloat>

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

void print_dash(){
	std::cout<<"---"<<std::endl;
	return;
};

double gen_angle(){
    double angle_R = (rand() % 360) * (PI/180.0);
    return angle_R;
}

class rrt_node{
	public:
		// vector<double> config;
		double * config;
		rrt_node * parent_ptr;
		int idx;
};

double get_dist(double * a, rrt_node * b, int size){
	double dist = 0;
	for(int i=0; i<size; i++){
		dist = dist + ((a[i] - b->config[i])*(a[i] - b->config[i]));
		std::cout<<"angles "<<a[i]<<" "<<b->config[i]<<" "<<dist<<std::endl;
	};
	std::cout<<" "<<std::endl;
	return sqrt(dist);

}

bool goal_reached(double* node_config, double* goal_config, int size){
	bool goal_reached = false;
	// for (int i=0; i<size; i++){
	// 	if (node_config[i]!=goal_config[i]){
	// 		goal_reached = false;
	// 		return goal_reached;
	// 	}
	// }
	double dist = 0;
	for (int i=0; i<size; i++){
		dist = dist + (node_config[i]-goal_config[i])*(node_config[i]-goal_config[i]);
	}
	dist = sqrt(dist);
	if (dist<0.5){
		goal_reached = true;
	}

	return goal_reached;

}

double get_dist_arr(double * a, double * b, int size){
	double dist = 0;
	for(int i=0; i<size; i++){
		dist = dist + (a[i]-b[i])*(a[i]-b[i]);
		dist = dist + fabs(a[i]-b[i]);
	}
	return sqrt(dist);
	// return (dist);

}


static void RRT_planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{

	const int max_count = 5;
	int count = 0;
	vector<rrt_node*>vertices;
	double config[numofDOFs];
	bool is_valid_config;
	bool is_goal = false;
	vector<double>distances;
	// vector<double>dummy_vector;
	double dummy_arr[numofDOFs];
	double diff[numofDOFs];
	const double epsilon = 1;
	bool goal_node_flag = false;
	double config_arr[numofDOFs];

	// creating start and goal rrt_node;
	rrt_node * start_node = new rrt_node;
	rrt_node * goal_node = new rrt_node;

	for (int i=0; i<numofDOFs; i++){
		// start_node->config.push_back(armstart_anglesV_rad[i]);
		start_node->config[i] = armstart_anglesV_rad[i];
		start_node->parent_ptr = NULL;
		start_node->idx = -1;
		// goal_node->config.push_back(armgoal_anglesV_rad[i]);
		// goal_node->parent_ptr = NULL;
		// goal_node->idx = -2;
	}

	vertices.push_back(start_node);
	// vertices.push_back(goal_node);

	while(count < max_count){
		// generate sample
		// is_vaid_config = false;
		double nearest_node_config[numofDOFs];
		goal_node_flag = false;
		is_goal = false;
		// dummy_vector.clear();
		int prob = rand()%100;
		if (prob>50){
			for (int i=0; i<numofDOFs; i++){
				config[i]=armgoal_anglesV_rad[i];
			}
			goal_node_flag = true;
			std::cout<<"haha"<<std::endl;
		}
		else {
			for (int i=0; i<numofDOFs; i++){
				config[i] = gen_angle();
			}
			std::cout<<"yoy"<<std::endl;
		};
	
		// if valid config, do the following
	
		// find nearest neighbor
		double max_dist = FLT_MAX;
		double sum = 0;
		double unit_vect_arr[numofDOFs];
		rrt_node * nearest_node = NULL;
		int idx = -3;
		// std::cout<<"size: "<<vertices.size()<<std::endl;
		for(auto& it : vertices){

			double distance = get_dist(config, it, numofDOFs);
			// std::cout<<"dist: "<<distance<<std::endl;
			if (distance<max_dist){
				// std::cout<<"updating"<<std::endl;
				max_dist = distance;
				idx = it->idx;
				nearest_node = it;
			}
		}
	
		std::cout<<"2"<<std::endl;

		for(int i=0; i<numofDOFs; i++){
			std::cout<<"here "<<i<<std::endl;
			nearest_node_config[i] = nearest_node->config[i];
		}
		std::cout<<"5"<<std::endl;
		// get diff with nearest neighbor
		for (int i=0; i<numofDOFs; i++){
			diff[i] = config[i] - nearest_node->config[i];
			sum = sum + (diff[i]*diff[i]);
		}
		// std::cout<<"3"<<std::endl;
		for (int i=0; i<numofDOFs; i++){
			unit_vect_arr[i] = diff[i]/sqrt(sum);
		}
		// std::cout<<"unit vector done"<<std::endl;

		// interpolate to epsilon and check
		double interpolated_node[numofDOFs];
		for(int i=0; i<numofDOFs; i++){
			interpolated_node[i] = nearest_node->config[i] + unit_vect_arr[i]*epsilon;
		};
		std::cout<<"4"<<std::endl;
		is_valid_config = IsValidArmConfiguration(interpolated_node, numofDOFs, map, x_size, y_size);
		if(!is_valid_config){
			std::cout<<"is not valid - cont."<<std::endl;
			continue;
		}


		else if (is_valid_config){
			// std::cout<<get_dist_arr(interpolated_node, armgoal_anglesV_rad, numofDOFs)<<std::endl;
			for(int i=0; i<numofDOFs; i++){
				std::cout<<config[i]<<" "<<nearest_node_config[i]<<" "<<unit_vect_arr[i]<<" "<<interpolated_node[i]<<" "<<armgoal_anglesV_rad[i]<<std::endl;
			}
			if(get_dist_arr(interpolated_node, armgoal_anglesV_rad, numofDOFs)<epsilon){
				// std::cout<<get_dist_arr(interpolated_node, armgoal_anglesV_rad, numofDOFs)<<std::endl;
				std::cout<<"is goal"<<std::endl;
				is_goal = true;
				for(int i=0; i<numofDOFs; i++){
					interpolated_node[i] = armgoal_anglesV_rad[i];
				}
				// std::cout<<"goal node - creating new node"<<std::endl;
				rrt_node * new_node = new rrt_node;
				new_node->idx = count;
				new_node->parent_ptr = nearest_node;
				for(int i=0; i<numofDOFs; i++){
					// new_node->config.push_back(interpolated_node[numofDOFs - i]);
					new_node->config[i] = interpolated_node[numofDOFs];
				}
				vertices.push_back(new_node);
			}

			else{
				std::cout<<"is not goal"<<std::endl;
				// std::cout<<"creating new node"<<std::endl;
				rrt_node * new_node = new rrt_node;
				new_node->idx = count;
				new_node->parent_ptr = nearest_node;
				for(int i=0; i<numofDOFs; i++){
					// new_node->config.push_back(interpolated_node[numofDOFs - i]);
					new_node->config[i] = interpolated_node[numofDOFs];
				}
				vertices.push_back(new_node);				
			}
		}
				

		



		// is_goal = goal_reached(interpolated_node, armgoal_anglesV_rad, numofDOFs);

		// if(is_goal){
		// 	std::cout<<"goal reached"<<std::endl;
		// 	break;
		// }

	

		// print_dash();
		std::cout<<"--"<<count<<"--"<<std::endl;

		count++;
	}

	// backtracking

	// for(int i=0; i<numofDOFs;i++){
	// 	rrt_node * top = vertices.front();
	// 	std::cout<<top->config[i];
	// }









	// // class node{
	// // 	public:
	// // 		vector<vector<int>> config;
	// // 		vector<int> idx; 
	// // };


	// const int max_count = 10;
	// int count = 0; 
	// vector<vector<float>> node_vector;
	// vector<int>parent_vector;
	// node_vector.reserve(max_count);
	// parent_vector.reserve(max_count);	
	// vector<float>dummy_vector;
	// double arr[numofDOFs];
	// bool is_valid_config;

	// // inserting start config
	// for(int i=0; i<numofDOFs; i++){
	// 	dummy_vector.push_back(armstart_anglesV_rad[i]);
	// };
	// node_vector.push_back(dummy_vector);
	// parent_vector.push_back(count);
	// count++;

	// // for(int i=0; i<numofDOFs; i++){
	// // 	std::cout<<"node vector: "<<node_vector[0][i]<<std::endl;
	// // };

	// // vector<node> nodes(max_count);
	// //starting sampling
	// while(count<max_count){
	// 	std::cout<<"Count: "<<count<<std::endl;
	// 	// generating sample config
	// 	for(int i=0; i<numofDOFs; i++){
	// 		float angle = gen_angle();
	// 		dummy_vector.push_back(angle);
	// 		arr[i] = angle;
	// 	}

	// 	// checking configuration is valid or not
	// 	is_valid_config = IsValidArmConfiguration(arr, numofDOFs, map, x_size, y_size);
	// 	if (!is_valid_config){
	// 		std::cout<<"invalid config"<<std::endl;
	// 		continue;
	// 	}

	// 	// when config is valid do the following
		
	// 	// std::cout<<"config generated "<<sizeof(node_vector)<<std::endl;
	// 	// std::cout<<"valid config generated!!"<<std::endl;
		


	// 	count++;
	// 	print_dash();

	// }


	// //no plan by default
	// *plan = NULL;
	// *planlength = 0;
		
    // //for now just do straight interpolation between start and goal checking for the validity of samples

    // double distance = 0;
    // int i,j;
    // for (j = 0; j < numofDOFs; j++){
    //     if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
    //         distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
    // }
    // int numofsamples = (int)(distance/(PI/20));
    // if(numofsamples < 2){
    //     printf("The arm is already at the goal\n");
    //     return;
    // }
	// int countNumInvalid = 0;
    // *plan = (double**) malloc(numofsamples*sizeof(double*));
    // for (i = 0; i < numofsamples; i++){
    //     (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
    //     for(j = 0; j < numofDOFs; j++){
    //         (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
    //     }
    //     if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
	// 		++countNumInvalid;
    //     }
    // }
	// printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
    // *planlength = numofsamples;
    
    // return;
}


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
	srand(10);
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

	double** plan = NULL;
	int planlength = 0;
	RRT_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);

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
