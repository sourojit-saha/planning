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
#include <ctime>

#include <cfloat>
#include <queue>
#include <unordered_map>
#include <set>
#include <unordered_set>
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

// void print_dash(){
// 	std::cout<<"---"<<std::endl;
// 	return;
// };

double gen_angle(){
    double angle_R = (rand() % 360) * (PI/180.0);
    return angle_R;
}

int gen_prob(){
	int a = rand()%100;
	return a;
}


// rand()/static_cast<double>(RAND_MAX)


double get_dist(double * a, double *b, int size){
	double dist = 0;
	for(int i=0; i<size; i++){
		dist = dist + (a[i] - b[i])*(a[i] - b[i]);
	}
	return sqrt(dist);
}


// int IsValidTransition(rrt_node* closest, double* new_node, int numofDOFS, double dist, double step, double* map, int x_size, int y_size){
// 	double* temp = new double[numofDOFS];
// 	int numofsteps = (int) (dist/step);
// 	for (int i=0; i<=numofsteps; i++){
// 		for (int j=0; j<numofDOFS; j++){
// 			temp[j] = closest->config[j] + i * step * (new_node[j] - closest->config[j]);
// 		}
// 		if (!IsValidArmConfiguration(temp, numofDOFS, map, x_size, y_size)){
// 			return 0;
// 		}
// 	}
// 	return 1;
// }

void print_config(double* a, int size){
	for(int i=0; i<size; i++){
		std::cout<<a[i]<<", ";
	}
	std::cout<<""<<std::endl;
}

class rrt_node{
    public:
        double config[10];
		int self_idx;
        int parent_idx;
};

class rrtc_node{
	public:
		double config[10];
		int self_idx;
		int parent_idx;
		int tree;
};

class rrtstar_node{
	public:
		double config[10];
		int self_idx;
		int parent_idx;
		double cost;
};

class prm_node{
	public:
		double config[10];
		int self_idx;
		vector<int>neighbors;
		bool connected_to_start = 0; //1
		bool connected_to_goal = 0;  //2
		// double g;
		// double h;
		double f = FLT_MAX;
		int parent = -1;


};

bool compare(prm_node a, prm_node b){
	return(a.f > b.f);
};

// rrt_node get_nearest_rrt_node(double * sample, vector<rrt_node> tree, int size){
// 	double max_dist = FLT_MAX;
// 	rrt_node nearest_node;
// 	for (auto& it:tree){
// 		double distance = get_dist(sample, it.config, size);
	
// 		if(distance < max_dist){
// 			max_dist = distance;
// 			nearest_node = it;
// 		}
// 	};

// 	return nearest_node;

// }

static void config_generator(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	double sample[20];
	int j=0;
	while(j<10){
		for(int i=0; i<numofDOFs;i++){
			sample[i] = gen_angle();
		}
		if(IsValidArmConfiguration(sample, numofDOFs, map, x_size, y_size)){
			for (int i=0; i<numofDOFs;i++){
				std::cout<<sample[i]<<",";
			}
			std::cout<<"\n"<<std::endl;
			j++;
		}
		else{
			continue;
		}
	}
	
};

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
	clock_t start = clock();
	const int max_count = 400000;
	vector<rrt_node>vertices;
    vertices.reserve(max_count);
	rrt_node dummy_node;
	rrt_node nearest_node;
	int count = 0;
	bool is_goal = false;
	bool is_valid_config = false;
	double sample[10];
	double max_dist = FLT_MAX;
	double sum = 0;
	double diff[10];
	double unit_vect[10];
	double interpolated_node[10];
	const double epsilon = 0.2;

	
	for(int i=0; i<numofDOFs; i++){
		dummy_node.config[i] = armstart_anglesV_rad[i];

	}
	dummy_node.self_idx = 0;
	dummy_node.parent_idx = 0;
	vertices.push_back(dummy_node);

	// dummy_node.self_idx = count+1000;
	// dummy_node.parent_idx = count+1000;
	// vertices.push_back(dummy_node);

	// count++; //////////////////////////////////////

	while (true){
		// double sample[10];
		is_goal = false;
		is_valid_config = 0;
		if(gen_prob()>50){
			for(int i=0; i<numofDOFs;i++){
				sample[i] = armgoal_anglesV_rad[i];
			}
			// std::cout<<"goal"<<std::endl;
		}
		else{
			for(int i=0; i<numofDOFs;i++){
				sample[i] = gen_angle();
			}
		}
		// std::cout<<"sample ";
		// print_config(sample, numofDOFs);

		max_dist = FLT_MAX;
		// std::cout<<"size---> "<<vertices.size()<<std::endl;
		for (auto& it:vertices){
			double distance = get_dist(sample, it.config, numofDOFs);
		
			if(distance < max_dist){
				max_dist = distance;
				nearest_node = it;
			}
		};	

		sum = 0;
		for(int i=0; i<numofDOFs;i++){
			diff[i] = sample[i] - nearest_node.config[i];
			sum = sum + (diff[i]*diff[i]);
		}

		for(int i=0; i<numofDOFs;i++){
			unit_vect[i] = diff[i]/sqrt(sum);
		}

		for(int i=0; i<numofDOFs;i++){
			interpolated_node[i] = nearest_node.config[i] + (unit_vect[i]*epsilon);
		}		
		// print_config(interpolated_node, numofDOFs);

		is_valid_config = IsValidArmConfiguration(interpolated_node, numofDOFs, map, x_size, y_size);

		if(!is_valid_config){
			// std::cout<<"invalid"<<std::endl;
			continue;
		}
		else if(is_valid_config){
		if(get_dist(interpolated_node, armgoal_anglesV_rad, numofDOFs)<epsilon){
			// std::cout<<"is goal"<<std::endl;
			for(int i=0; i<numofDOFs;i++){
				// interpolated_node[i] = armgoal_anglesV_rad[i];
				dummy_node.config[i] = armgoal_anglesV_rad[i];
			}
			// dummy_node.self_idx = count;
			dummy_node.self_idx = vertices.size();
			dummy_node.parent_idx = nearest_node.self_idx;
			vertices.push_back(dummy_node);
			// std::cout<<"-pushed"<<std::endl;
			// for(int i=0; i<numofDOFs;i++){
			// 	std::cout<<dummy_node.config[i]<<std::endl;
			// }
			break;
		}
		else{
			for(int i=0; i<numofDOFs;i++){
				dummy_node.config[i] = interpolated_node[i];
			}
			// dummy_node.self_idx = count;
			dummy_node.self_idx = vertices.size();
			dummy_node.parent_idx = nearest_node.self_idx;
			vertices.push_back(dummy_node);
			// std::cout<<"->pushed"<<std::endl;
		}
		}
		// std::cout<<vertices.size()<<" "<<count<<std::endl;
		// std::cout<<"---"<<std::endl;
		count++;

	} // while loop ends here
	// for(auto& it:vertices){
	// 	std::cout<<" it.parent_idx "<<it.parent_idx<<std::endl;
	// }
	// backtracking
	// std::cout<<"backtracking"<<std::endl;
	*plan = NULL;
	*planlength = 0;
	int length = 0;
	int collision = 0;
	vector<rrt_node>forward_vect;	
	// rrt_node temp_node = vertices.back();	
	int goal_idx = vertices.back().self_idx;
	int k = goal_idx;
	while(true){
		if(k==0){
			forward_vect.push_back(vertices[k]);
			length++;
			break;
		}
		forward_vect.push_back(vertices[k]);
		k = forward_vect.back().parent_idx;
		length++;

	}
	// print_config(forward_vect.front().config, numofDOFs);
	// print_config(forward_vect.back().config, numofDOFs);
	// std::cout<<"size "<<forward_vect.size()<<std::endl;
	// std::cout<<"length "<<length<<std::endl;

	// for(int j = 0; j < numofDOFs; j++){
	// 	std::cout<<forward_vect[0].config[j]<<"   "<<forward_vect[length-1].config[j]<<std::endl;
	// 	// std::cout<<(*plan)[i][j]<<std::endl;
	// }
	*planlength = length;

	*plan = (double**) malloc(length*sizeof(double*));
    for (int i = 0; i < length; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		// std::cout<<temp_node<<std::endl;
        for(int j = 0; j < numofDOFs; j++){

			(*plan)[i][j] = forward_vect[length-i-1].config[j];
			// std::cout<<(*plan)[i][j]<<std::endl;
        }	
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)){
			collision++;
		}
		// std::cout<<"~~~"<<std::endl;

		// temp_node = temp_node->parent_ptr;

	}
	// std::cout<<"length "<<length<<std::endl;
	// std::cout<<"collision "<<collision<<std::endl;
	// std::cout<<"iterations "<<count<<std::endl;
	// std::cout<<"graph "<<vertices.size()<<std::endl;
	// std::cout<<"time "<<double((clock() - start)/CLOCKS_PER_SEC)<<std::endl;
	return;
	// std::cout<<"size "<<forward_vect.size()<<std::endl;
	// std::cout<<"length "<<length<<std::endl;

	}	


static void RRTConnect_planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	
	clock_t start = clock();
	const int max_count = 10000;
	const int max_iter = 10;
	vector<rrtc_node>start_tree;
	vector<rrtc_node>goal_tree;
	start_tree.reserve(max_count);
	goal_tree.reserve(max_count);
	int start_counter = 0;
	int goal_counter = 0;
	int count = 0;
	rrtc_node dummy_node;
	rrtc_node dummy_node_1;
	rrtc_node dummy_node_2;
	bool which_tree = 0;
	double sample[10];
	rrtc_node nearest_node_1;
	rrtc_node nearest_node_2;
	double max_dist = FLT_MAX;
	double sum = 0;
	double diff[10];
	double unit_vect[10];
	const double epsilon = 0.4;
	double interpolated_node[10];
	bool is_valid_config = 0;
	bool is_goal;
	// double extented_node[10];
	int idx;
	int start_idx;
	int goal_idx;

	for(int i=0; i<numofDOFs; i++){
		dummy_node_1.config[i] = armstart_anglesV_rad[i];
		dummy_node_2.config[i] = armgoal_anglesV_rad[i];
	}

	dummy_node_1.self_idx = start_tree.size();
	dummy_node_2.self_idx = goal_tree.size();

	dummy_node_1.parent_idx = 0;
	dummy_node_2.parent_idx = 0;

	dummy_node_1.tree = 1;
	dummy_node_2.tree = 2;	

	start_tree.push_back(dummy_node_1);
	goal_tree.push_back(dummy_node_2);

	// start_counter++;
	// goal_counter++;

	while(true){
		// generate sample
		for(int i=0; i<numofDOFs; i++){
			sample[i] = gen_angle();
		}

		if(which_tree == 0){
			// start_tree
			// std::cout<<"starting with start tree"<<std::endl;
			is_goal = 0;
			max_dist = FLT_MAX;
			for (auto& it:start_tree){
				double distance = get_dist(sample, it.config, numofDOFs);
			
				if(distance < max_dist){
					max_dist = distance;
					nearest_node_1 = it;
				}
			};	

			sum = 0;
			for(int i=0; i<numofDOFs;i++){
				diff[i] = sample[i] - nearest_node_1.config[i];
				sum = sum + (diff[i]*diff[i]);
			}

			for(int i=0; i<numofDOFs;i++){
				unit_vect[i] = diff[i]/sqrt(sum);
			}

			for(int i=0; i<numofDOFs;i++){
				interpolated_node[i] = nearest_node_1.config[i] + (unit_vect[i]*epsilon);
			}

			is_valid_config = IsValidArmConfiguration(interpolated_node, numofDOFs, map, x_size, y_size);
	
			if(!is_valid_config){
				continue;
			}


			max_dist = FLT_MAX;
			for (auto& it:goal_tree){
				double distance = get_dist(interpolated_node, it.config, numofDOFs);
			
				if(distance < max_dist){
					max_dist = distance;
					nearest_node_2 = it;
				}
			};

			if(get_dist(interpolated_node, nearest_node_2.config, numofDOFs)<epsilon){
				// std::cout<<"is goal - start tree - extend"<<std::endl;
				is_goal = 1;
				dummy_node.self_idx = -1;
				dummy_node.parent_idx = nearest_node_1.self_idx;
				for(int i=0; i<numofDOFs; i++){
					dummy_node.config[i] = nearest_node_2.config[i];
				}
				start_tree.push_back(dummy_node);

				// for(int i=0; i<numofDOFs; i++){
				// 	std::cout<<start_tree.back().config[i]<<" "<<nearest_node_2.config[i]<<std::endl;
				// }

				start_idx = start_tree.back().self_idx;
				goal_idx = nearest_node_2.self_idx;
				break;

			}

			else{
				dummy_node.self_idx = start_tree.size();
				dummy_node.parent_idx = nearest_node_1.self_idx;
				for(int i=0; i<numofDOFs; i++){
					dummy_node.config[i] = interpolated_node[i];
				}
				start_tree.push_back(dummy_node);
			}

			// connect function
			// std::cout<<"starting connection function in start tree"<<std::endl;

			sum = 0;
			for(int i=0; i<numofDOFs;i++){
				diff[i] = interpolated_node[i] - nearest_node_2.config[i];
				sum = sum + (diff[i]*diff[i]);
			}

			for(int i=0; i<numofDOFs;i++){
				unit_vect[i] = diff[i]/sqrt(sum);
			}

			dummy_node_1 = nearest_node_2;

			while(true){

				for(int i=0; i<numofDOFs;i++){
					dummy_node_1.config[i] = dummy_node_1.config[i] + (unit_vect[i]*epsilon);
				}

				is_valid_config = IsValidArmConfiguration(dummy_node_1.config, numofDOFs, map, x_size, y_size);

				if(is_valid_config){
					if(get_dist(dummy_node_1.config, interpolated_node, numofDOFs)<epsilon){
						// std::cout<<"is goal - start tree - connect"<<std::endl;
						dummy_node_1.parent_idx = dummy_node_1.self_idx;
						dummy_node_1.self_idx = goal_tree.size();

						for(int i=0; i<numofDOFs;i++){
							dummy_node_1.config[i] = interpolated_node[i];
						}
						goal_tree.push_back(dummy_node_1);
						start_idx = start_tree.back().self_idx;
						goal_idx = goal_tree.back().self_idx;
						is_goal = 1;

						// for(int i=0; i<numofDOFs; i++){
						// 	std::cout<<goal_tree.back().config[i]<<" "<<interpolated_node[i]<<std::endl;
						// }						
						break;
					}

					else{

						dummy_node_1.parent_idx = dummy_node_1.self_idx;
						dummy_node_1.self_idx = goal_tree.size();
						goal_tree.push_back(dummy_node_1);
					}

				}

				else{
					break;
				}

			}

			if(is_goal){
				break;
			}

			which_tree = not(which_tree);

			// start_counter++;

		}

		else if(which_tree == 1){
			// goal_tree
			// std::cout<<"starting with goal tree"<<std::endl;
			is_goal = 0;
			max_dist = FLT_MAX;
			for (auto& it:goal_tree){
				double distance = get_dist(sample, it.config, numofDOFs);
			
				if(distance < max_dist){
					max_dist = distance;
					nearest_node_1 = it;
				}
			};	

			sum = 0;
			for(int i=0; i<numofDOFs;i++){
				diff[i] = sample[i] - nearest_node_1.config[i];
				sum = sum + (diff[i]*diff[i]);
			}

			for(int i=0; i<numofDOFs;i++){
				unit_vect[i] = diff[i]/sqrt(sum);
			}

			for(int i=0; i<numofDOFs;i++){
				interpolated_node[i] = nearest_node_1.config[i] + (unit_vect[i]*epsilon);
			}

			is_valid_config = IsValidArmConfiguration(interpolated_node, numofDOFs, map, x_size, y_size);
	
			if(!is_valid_config){
				continue;
			}


			max_dist = FLT_MAX;
			for (auto& it:start_tree){
				double distance = get_dist(interpolated_node, it.config, numofDOFs);
			
				if(distance < max_dist){
					max_dist = distance;
					nearest_node_2 = it; //from start tree
				}
			};

			if(get_dist(interpolated_node, nearest_node_2.config, numofDOFs)<epsilon){
				// std::cout<<"is goal - goal tree - extend"<<std::endl;
				is_goal = 1;
				dummy_node.self_idx = -1;
				dummy_node.parent_idx = nearest_node_1.self_idx;
				for(int i=0; i<numofDOFs; i++){
					dummy_node.config[i] = nearest_node_2.config[i];
				}
				goal_tree.push_back(dummy_node);

				// for(int i=0; i<numofDOFs; i++){
				// 	std::cout<<goal_tree.back().config[i]<<" "<<nearest_node_2.config[i]<<std::endl;
				// }

				start_idx = nearest_node_2.self_idx;
				goal_idx = goal_tree.back().self_idx;
				break;

			}

			else{
				dummy_node.self_idx = goal_tree.size();
				dummy_node.parent_idx = nearest_node_1.self_idx;
				for(int i=0; i<numofDOFs; i++){
					dummy_node.config[i] = interpolated_node[i];
				}
				goal_tree.push_back(dummy_node);
			}

			// connect function

			// std::cout<<"starting connection function in goal tree"<<std::endl;

			sum = 0;
			for(int i=0; i<numofDOFs;i++){
				diff[i] = interpolated_node[i] - nearest_node_2.config[i];
				sum = sum + (diff[i]*diff[i]);
			}

			for(int i=0; i<numofDOFs;i++){
				unit_vect[i] = diff[i]/sqrt(sum);
			}

			dummy_node_1 = nearest_node_2;

			while(true){

				for(int i=0; i<numofDOFs;i++){
					dummy_node_1.config[i] = dummy_node_1.config[i] + (unit_vect[i]*epsilon);
				}

				is_valid_config = IsValidArmConfiguration(dummy_node_1.config, numofDOFs, map, x_size, y_size);

				if(is_valid_config){
					if(get_dist(dummy_node_1.config, interpolated_node, numofDOFs)<epsilon){
						// std::cout<<"is goal - goal tree - connect"<<std::endl;
						dummy_node_1.parent_idx = dummy_node_1.self_idx;
						dummy_node_1.self_idx = start_tree.size();

						for(int i=0; i<numofDOFs;i++){
							dummy_node_1.config[i] = interpolated_node[i];
						}
						start_tree.push_back(dummy_node_1);
						is_goal = 1;
						// for(int i=0; i<numofDOFs; i++){
						// 	// std::cout<<start_tree.back().config[i]<<" "<<interpolated_node[i]<<std::endl;
						// 	std::cout<<start_tree.back().config[i]<<" "<<goal_tree.back().config[i]<<std::endl;
						// }

						start_idx = start_tree.back().self_idx;
						goal_idx = goal_tree.back().self_idx;///////////////////////////////////////
						break;
					}

					else{

						dummy_node_1.parent_idx = dummy_node_1.self_idx;
						dummy_node_1.self_idx = start_tree.size();
						start_tree.push_back(dummy_node_1);
					}
				}

				else{
					break;
				}

			}

			if(is_goal){
				break;
			}

			which_tree = not(which_tree);

		}

		count++;
	}

	// std::cout<<"---"<<std::endl;
	// for(int i=0; i<numofDOFs; i++){
	// 	std::cout<<start_tree[start_idx].config[i]<<" "<<goal_tree[goal_idx].config[i]<<std::endl;
	// }

	//backtracking
	*plan = NULL;
	*planlength = 0;
	int length_start = 0;
	int length_goal = 0;
	int length = 0;
	int collision = 0;

	vector<rrtc_node>start_;
	vector<rrtc_node>goal_;

	// int idx = 0;
	idx = start_tree.back().self_idx;

	while(true){
		// std::cout<<"start_tree "<<idx<<std::endl;
		if(idx == 0){
			start_.push_back(start_tree[idx]);
			length_start++;
			break;
		}
		start_.push_back(start_tree[idx]);
		idx = start_.back().parent_idx;
		length_start++;
	}

	// idx = 0;
	idx = goal_tree.back().self_idx;

	while(true){
		// std::cout<<"goal_tree "<<idx<<std::endl;
		if(idx == 0){
			goal_.push_back(goal_tree[idx]);
			length_goal++;
			break;
		}
		goal_.push_back(goal_tree[idx]);
		idx = goal_.back().parent_idx;
		length_goal++;
	}
	length = length_start + length_goal;
	*planlength = length;
	// std::cout<<"..."<<std::endl;

	// for(int i=0; i<numofDOFs; i++){
	// 	std::cout<<start_[0].config[i]<<" "<<goal_[0].config[i]<<std::endl;
	// }

	// std::cout<<"~~~"<<std::endl;

	// for(int i=0; i<numofDOFs; i++){
	// 	std::cout<<start_[length_start-1].config[i]<<" "<<goal_[length_goal-1].config[i]<<std::endl;
	// }

	*plan = (double**) malloc(length*sizeof(double*));
    for (int i = 0; i < length_start; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		// std::cout<<temp_node<<std::endl;
        for(int j = 0; j < numofDOFs; j++){

			(*plan)[i][j] = start_[length_start-i-1].config[j];
			// std::cout<<(*plan)[i][j]<<std::endl;
        }	
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)){
			collision++;
		}

	}

    for (int i = length_start; i < length; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		// std::cout<<temp_node<<std::endl;
        for(int j = 0; j < numofDOFs; j++){

			(*plan)[i][j] = goal_[i-length_start].config[j];
			// std::cout<<(*plan)[i][j]<<std::endl;
        }	
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)){
			collision++;
		}
	}
	// std::cout<<"length "<<length<<std::endl;
	// std::cout<<"collision "<<collision<<std::endl;
	// std::cout<<"iterations "<<count<<std::endl;
	// std::cout<<"graph "<<start_tree.size() + goal_tree.size()<<std::endl;
	// std::cout<<"time "<<double((clock() - start)/CLOCKS_PER_SEC)<<std::endl;
	return;


}


static void RRTStar_planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{

	clock_t start = clock();
	const int max_count = 10000;
	vector<rrtstar_node>tree;
	tree.reserve(max_count);
	int count = 0;
	int max_iter = 5;
	rrtstar_node dummy_node;
	rrtstar_node nearest_node;
	double sample[10];
	double radius;
	double max_dist = FLT_MAX;
	double sum = 0;
	double unit_vect[10];
	double diff[10];
	double interpolated_node[10];
	const double epsilon = 0.4;
	bool is_valid_config = 0;
	bool is_goal = 0;
	vector<rrtstar_node>closest_nodes;
	closest_nodes.reserve(1000);
	const double gamma = 100;
	double max_cost = FLT_MAX;
	double distance;
	double cost;

	for(int i=0;i<numofDOFs; i++){
		dummy_node.config[i] = armstart_anglesV_rad[i];
	}

	dummy_node.self_idx = tree.size();
	dummy_node.parent_idx = 0;
	dummy_node.cost = 0;
	tree.push_back(dummy_node);

	while(true){
		closest_nodes.clear();
		is_valid_config = 0;
		is_goal = 0;
		if(gen_prob()>50){
			for(int i=0;i<numofDOFs; i++){
				sample[i] = armgoal_anglesV_rad[i];
			}
		}
		else{
			for(int i=0;i<numofDOFs; i++){
				sample[i] = gen_angle();
			}
		}

		max_dist = FLT_MAX;
		for (auto& it:tree){
			double distance = get_dist(sample, it.config, numofDOFs);
		
			if(distance < max_dist){
				max_dist = distance;
				nearest_node = it;
			}
		};

		sum = 0;
		for(int i=0; i<numofDOFs;i++){
			diff[i] = sample[i] - nearest_node.config[i];
			sum = sum + (diff[i]*diff[i]);
		}		

		for(int i=0; i<numofDOFs;i++){
			unit_vect[i] = diff[i]/sqrt(sum);
		}

		for(int i=0; i<numofDOFs;i++){
			interpolated_node[i] = nearest_node.config[i] + (unit_vect[i]*epsilon);
		}
		is_valid_config = IsValidArmConfiguration(interpolated_node, numofDOFs, map, x_size, y_size);

		if(!is_valid_config){
			// std::cout<<"invalid"<<std::endl;
			continue;
		}

		else if(is_valid_config){
			if(get_dist(interpolated_node, armgoal_anglesV_rad, numofDOFs)<epsilon){
				// std::cout<<"is goal"<<std::endl;
				is_goal = 1;
			for(int i=0; i<numofDOFs;i++){
				dummy_node.config[i] = armgoal_anglesV_rad[i];
			}
			dummy_node.self_idx = tree.size();
			dummy_node.parent_idx = nearest_node.self_idx;
			dummy_node.cost = nearest_node.cost + get_dist(dummy_node.config, nearest_node.config, numofDOFs);
			}

			else{
			for(int i=0; i<numofDOFs;i++){
				dummy_node.config[i] = interpolated_node[i];
			}
			dummy_node.self_idx = tree.size();
			dummy_node.parent_idx = nearest_node.self_idx;
			dummy_node.cost = nearest_node.cost + get_dist(dummy_node.config, nearest_node.config, numofDOFs);
			}
			// new node has been created but not pushed yet

			// find closest neighbors within radius

			radius = std::min(pow(gamma*std::log(tree.size())/tree.size(), 1.0/max_count), epsilon);

			for (auto& it:tree){
				if(get_dist(it.config, dummy_node.config, numofDOFs)<radius){
					closest_nodes.push_back(it);
				}
			};

			// set of closest nodes have been found
			// iterate through all to find best path/connection
			// part 1 of re-wiring
			max_cost = FLT_MAX;
			for(auto& it:closest_nodes){
				distance = get_dist(it.config, dummy_node.config, numofDOFs);
				cost = distance + it.cost;

				if(cost<max_cost){
					max_cost = cost;
					dummy_node.parent_idx = it.self_idx;
					dummy_node.cost = cost;
				}
			}

			// part 2 of re-wiring
			// iterate through all closest nodes and update

			for(auto& it:closest_nodes){
				distance = get_dist(it.config, dummy_node.config, numofDOFs);
				cost = distance + dummy_node.cost;
				if(cost<it.cost){
					it.parent_idx = dummy_node.self_idx;
					it.cost = cost;
				}

			}

			tree.push_back(dummy_node);

			if(is_goal){
				break;
			}

			

		}
		// std::cout<<count<<std::endl;
		count++;
	}

	// for(int i=0; i<numofDOFs;i++){
	// 	std::cout<<tree.back().config[i]<<std::endl;
	// }

	*plan = NULL;
	*planlength = 0;
	int length = 0;
	int collision = 0;
	vector<rrtstar_node>forward_vect;	
	int goal_idx = tree.back().self_idx;
	int k = goal_idx;
	while(true){
		if(k==0){
			forward_vect.push_back(tree[k]);
			length++;
			break;
		}
		forward_vect.push_back(tree[k]);
		k = forward_vect.back().parent_idx;
		length++;

	}
	*planlength = length;

	*plan = (double**) malloc(length*sizeof(double*));
    for (int i = 0; i < length; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		// std::cout<<temp_node<<std::endl;
        for(int j = 0; j < numofDOFs; j++){

			(*plan)[i][j] = forward_vect[length-i-1].config[j];
			// std::cout<<(*plan)[i][j]<<std::endl;
        }	
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)){
			collision++;
		}
		// std::cout<<"~~~"<<std::endl;

		// temp_node = temp_node->parent_ptr;

	}
	// std::cout<<"length "<<length<<std::endl;
	// std::cout<<"collision "<<collision<<std::endl;
	// std::cout<<"iterations "<<count<<std::endl;
	// std::cout<<"graph "<<tree.size()<<std::endl;
	// std::cout<<"time "<<double((clock() - start)/CLOCKS_PER_SEC)<<std::endl;
    return;
}


static void PRM_planner(
			double* map,
			int x_size,
			int y_size,
			double* armstart_anglesV_rad,
			double* armgoal_anglesV_rad,
            int numofDOFs,
            double*** plan,
            int* planlength)
{
	clock_t start = clock();
	const int max_count = 40000;
	int count = 0;
	const int max_iter = 5;
	vector<prm_node>graph;
	vector<prm_node>closest_neighbors;
	graph.reserve(max_count);
	closest_neighbors.reserve(1000);
	double sample[10];
	prm_node dummy_node;
	bool is_valid_config;
	bool is_goal;
	double epsilon = 0.3;
	double radius = epsilon;
	bool near_goal_node = 0;
	double cost_to_dummy_node = 0;

	for(int i=0; i<numofDOFs; i++){
		dummy_node.config[i] = armstart_anglesV_rad[i];
	}
	dummy_node.connected_to_start = 1;
	dummy_node.connected_to_goal = 0;
	dummy_node.self_idx = graph.size();
	dummy_node.f = 0;
	graph.push_back(dummy_node);

	for(int i=0; i<numofDOFs; i++){
	dummy_node.config[i] = armgoal_anglesV_rad[i];
	}
	dummy_node.connected_to_start = 0;
	dummy_node.connected_to_goal = 10;
	dummy_node.self_idx = graph.size();
	dummy_node.f = FLT_MAX;
	graph.push_back(dummy_node);

	// preprocessing step
	while(true){

		closest_neighbors.clear();
		is_goal = 0;
		// bool near_goal_node = 0;

		if(gen_prob()>50 and !near_goal_node){
			for(int i=0; i<numofDOFs;i++){
				sample[i] = armgoal_anglesV_rad[i];
			}
			if(IsValidArmConfiguration(sample, numofDOFs, map, x_size, y_size)){
				near_goal_node = 1;
			}
			
			// std::cout<<"near goal node"<<std::endl;
			// break;
		}
		else{
			for(int i=0; i<numofDOFs;i++){
				sample[i] = gen_angle();
			}
		}

		// if(gen_prob()>80){
		// 	for(int i=0; i<numofDOFs;i++){
		// 		sample[i] = armgoal_anglesV_rad[i];
		// 	}
		// 	// std::cout<<"goal"<<std::endl;
		// }
		// else{
		// 	for(int i=0; i<numofDOFs;i++){
		// 		sample[i] = gen_angle();
		// 	}
		// }

		// for(int i=0; i<numofDOFs; i++){
		// 	sample[i] = gen_angle();
		// }

		// if(get_dist(sample, armgoal_anglesV_rad, numofDOFs)<epsilon){
		// 	for(int i=0; i<numofDOFs; i++){
		// 		sample[i] = armgoal_anglesV_rad[i];
		// 	}
		// 	std::cout<<"~~~~~~~~~~~~~~~~"<<std::endl;
		// }

		is_valid_config = IsValidArmConfiguration(sample, numofDOFs, map, x_size, y_size);

		// valid sample must be added to graph

		if(!is_valid_config){

			// near_goal_node = 0;
			continue;
		}

		else if (is_valid_config){

			for(int i=0; i<numofDOFs; i++){
				dummy_node.config[i] = sample[i];
			}	

			dummy_node.f = FLT_MAX;
			dummy_node.self_idx = graph.size();
			dummy_node.connected_to_start = 0;
			dummy_node.connected_to_goal = 0;
			dummy_node.neighbors.clear();


			for(auto& it:graph){
				if(get_dist(dummy_node.config, it.config, numofDOFs)<epsilon*2){
					closest_neighbors.push_back(it);
				}
			}// set of closesd neighbors created

			if(closest_neighbors.size()==0){
				graph.push_back(dummy_node);
				continue;
			}


			for(auto& it:closest_neighbors){
				dummy_node.neighbors.push_back(it.self_idx);
				int it_idx = it.self_idx;
				// it.neighbors.push_back(dummy_node.self_idx);
				graph[it_idx].neighbors.push_back(dummy_node.self_idx);
				// if(it.self_idx == 0){
				// 	std::cout<<"starting node@@@@@@@@@@@@@@"<<std::endl;
				// 	// std::cout<<"front--- "<<graph.front().neighbors.size()<<std::endl;
				// 	std::cout<<"front--- "<<graph[it.self_idx].neighbors.size()<<std::endl;
				// 	// return;
				// }

				// if(it.self_idx == 1){
				// 	std::cout<<"```````goal node"<<std::endl;
				// 	std::cout<<"back--- "<<graph.front().neighbors.size()<<std::endl;
				// 	// return;
				// }

				cost_to_dummy_node = it.f + get_dist(dummy_node.config, it.config, numofDOFs);
				if(cost_to_dummy_node<dummy_node.f){
					dummy_node.f = cost_to_dummy_node;
				}

				// if(!dummy_node.connected_to_start and it.connected_to_start){
				// 	dummy_node.connected_to_start = 1;
				// 	// std::cout<<"#########connected to start"<<std::endl;
				// }
				// if(!dummy_node.connected_to_goal and it.connected_to_goal){
				// 	dummy_node.connected_to_goal = 1;
				// 	// std::cout<<"~~~~~~~~~~connected to end~~~~~~~~~~"<<std::endl;
				// }

				if(dummy_node.connected_to_start or it.connected_to_start){
					dummy_node.connected_to_start = 1;
					graph[it_idx].connected_to_start = 1;
					// std::cout<<"#########connected to start"<<std::endl;
				}
				if(dummy_node.connected_to_goal or it.connected_to_goal){
					dummy_node.connected_to_goal = 1;
					graph[it_idx].connected_to_goal = 1;
					// std::cout<<"~~~~~~~~~~connected to end~~~~~~~~~~"<<std::endl;
				}

					// std::cout<<dummy_node.connected_to_start<<dummy_node.connected_to_goal<<std::endl;


			}

			if(dummy_node.connected_to_start and dummy_node.connected_to_goal){
				// std::cout<<"connection found "<<near_goal_node<<std::endl;
				// std::cout<<"connection found and pushed "<<std::endl;
				graph.push_back(dummy_node);
				break;
			}
			else{
				// std::cout<<"pushing"<<std::endl;
				graph.push_back(dummy_node);
				// std::cout<<graph.size()<<std::endl;

			}

		}

		// std::cout<<"count "<<count<<std::endl;

		count++;
	}// preprocessing finished

	// astar

	std::priority_queue<prm_node, std::vector<prm_node>, decltype(&compare)> openlist(compare);
	// std::unordered_set <int>closedlist;
	std::vector<int>closedlist;
	// std::cout<<"front "<<graph.front().neighbors.size()<<std::endl;
	// std::cout<<"back "<<graph.back().neighbors.size()<<std::endl;
	// std::cout<<"graph "<<graph.size()<<std::endl;
	// std::cout<<graph[0].connected_to_start<<graph[0].connected_to_goal<<graph[1].connected_to_start<<graph[1].connected_to_goal<<std::endl;

	// for (int i=0; i<numofDOFs; i++){
	// 	std::cout<<graph.front().config[i]<<std::endl;
	// }
	// std::cout<<"starting A star"<<std::endl;
	openlist.push(graph.front());
	prm_node to_expand;
	bool to_expand_in_closedlist = 0;
	bool found_in_closedlist = 0;
	bool goal_reached = 0;
	std::vector<int>::iterator it;
	// while(0)
	while(!openlist.empty())
	{	
		// std::cout<<openlist.size()<<std::endl;
		to_expand_in_closedlist = 0;
		to_expand = openlist.top();
		openlist.pop();
		it = std::find(closedlist.begin(), closedlist.end(), to_expand.self_idx);
		if(it != closedlist.end()){
			// found in closedlist
			// std::cout<<"to expand in closedlist"<<std::endl;
			continue;
		}

		// closedlist.insert(to_expand.self_idx);
		closedlist.push_back(to_expand.self_idx);
		// std::cout<<"distance "<<get_dist(to_expand.config, armgoal_anglesV_rad, numofDOFs)<<std::endl;
		// if (get_dist(to_expand.config, armgoal_anglesV_rad, numofDOFs)==0){
		// 	goal_reached=1;
		// 	std::cout<<"goal reached"<<std::endl;
		// 	break;
		// }
		// std::cout<<"checking neighbors"<<std::endl;
		for (auto& iter:to_expand.neighbors){
			// std::cout<<"iter "<<iter<<std::endl;
			it = std::find(closedlist.begin(), closedlist.end(), iter);
			if(it !=closedlist.end()){
				// is in closedlist
				// std::cout<<"neighbor discarded"<<std::endl;
				continue;
			}	

			double f = to_expand.f + get_dist(to_expand.config, graph[iter].config, numofDOFs);
			if(graph[iter].f>f){
				// std::cout<<"neighbor updated"<<std::endl;
				graph[iter].f = f;
				graph[iter].parent = to_expand.self_idx;
			}	

			openlist.push(graph[iter]);
		
		}


	}

	*plan = NULL;
	*planlength = 0;

	int temp = graph[1].self_idx;
	int length = 0;
	int collision= 0;
	vector<prm_node>forward_vect;
	while(temp !=-1){
		forward_vect.push_back(graph[temp]);
		temp = graph[temp].parent;
		length++;

	}


	// for (int i=0; i<numofDOFs; i++){
	// 	std::cout<<graph[1].config[i]<<std::endl;
	// }

	// std::cout<<"length "<<length<<std::endl;
	

	forward_vect.push_back(graph[0]);

	// for(int i=0; i<forward_vect.size();i++){
	// 	// std::cout<<"i "<<i<<std::endl;
	// 	std::cout<<get_dist(forward_vect[i].config, forward_vect[i+1].config, numofDOFs)<<std::endl;
	// }
	// for(auto& it:forward_vect){
	// 	for(int i=0; i<numofDOFs; i++){
	// 		std::cout<<it.config[i]<<std::endl;
	// 	}
	// 	std::cout<<"---"<<std::endl;
	// }
	// for(int i=0; i<numofDOFs; i++){
	// 	std::cout<<forward_vect.front().config[i]<<" "<<forward_vect.back().config[i]<<std::endl;
	// }
	length = length +1;
	*planlength = length;

	*plan = (double**) malloc(length*sizeof(double*));
    for (int i = 0; i < length+1; i++){
        (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
		// std::cout<<temp_node<<std::endl;
        for(int j = 0; j < numofDOFs; j++){

			(*plan)[i][j] = forward_vect[length-i-1].config[j];
			// std::cout<<(*plan)[i][j]<<std::endl;
        }	
		if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)){
			collision++;
		}
		// std::cout<<"~~~"<<std::endl;

		// temp_node = temp_node->parent_ptr;

	}
	// std::cout<<"length "<<length<<std::endl;
	// std::cout<<"collision "<<collision<<std::endl;
	// std::cout<<"iterations "<<count<<std::endl;
	// std::cout<<"graph "<<graph.size()<<std::endl;
	// std::cout<<"time "<<double((clock() - start)/CLOCKS_PER_SEC)<<std::endl;

return;
	


}

// static void planner(
// 			double* map,
// 			int x_size,
// 			int y_size,
// 			double* armstart_anglesV_rad,
// 			double* armgoal_anglesV_rad,
//             int numofDOFs,
//             double*** plan,
//             int* planlength)
// {

// 	//no plan by default
// 	*plan = NULL;
// 	*planlength = 0;
		
//     //for now just do straight interpolation between start and goal checking for the validity of samples

//     double distance = 0;
//     int i,j;
//     for (j = 0; j < numofDOFs; j++){
//         if(distance < fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]))
//             distance = fabs(armstart_anglesV_rad[j] - armgoal_anglesV_rad[j]);
//     }
//     int numofsamples = (int)(distance/(PI/20));
//     if(numofsamples < 2){
//         printf("The arm is already at the goal\n");
//         return;
//     }
// 	int countNumInvalid = 0;
//     *plan = (double**) malloc(numofsamples*sizeof(double*));
//     for (i = 0; i < numofsamples; i++){
//         (*plan)[i] = (double*) malloc(numofDOFs*sizeof(double)); 
//         for(j = 0; j < numofDOFs; j++){
//             (*plan)[i][j] = armstart_anglesV_rad[j] + ((double)(i)/(numofsamples-1))*(armgoal_anglesV_rad[j] - armstart_anglesV_rad[j]);
//         }
//         if(!IsValidArmConfiguration((*plan)[i], numofDOFs, map, x_size, y_size)) {
// 			++countNumInvalid;
//         }
//     }
// 	printf("Linear interpolation collided at %d instances across the path\n", countNumInvalid);
//     *planlength = numofsamples;
    
//     return;
// }


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
	srand(1);
	// srand(time(0));
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
	// config_generator(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	switch (whichPlanner)
	{
	case 0:
		std::cout<<"RRT Planner"<<std::endl;
		RRT_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
		break;
	case 1:
		std::cout<<"RRT Connect Planner"<<std::endl;
		RRTConnect_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
		break;
	case 2:
		std::cout<<"RRT Star Planner"<<std::endl;
		RRTStar_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
		break;
	case 3:
		std::cout<<"PRM Planner"<<std::endl;
		PRM_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
		break;
	default:
		break;
	}
	// RRT_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// RRTConnect_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// RRTStar_planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);
	// planner(map, x_size, y_size, startPos, goalPos, numOfDOFs, &plan, &planlength);

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
