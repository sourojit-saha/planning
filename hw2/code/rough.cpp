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
#include <queue>
#include <unordered_map>
#include <set>
#include <unordered_set>

class prm_node{
	public:
		double config[10];
		int self_idx;
		std::vector<int>neighbors;
		bool connected_to_start = 0; //1
		bool connected_to_goal = 0;  //2
		double g;
		double h;
		double f;


};

// bool compare(int a, int b, std::vector<prm_node>graph){
//     prm_node a1 = graph[a];
//     prm_node b1 = graph[b];
// 	return(a1.g > b1.g);
// };

bool compare(prm_node a, prm_node b){
	return(a.g > b.g);
};

// class hash_func{
//     public:

// }

std::priority_queue<prm_node, std::vector<prm_node>, decltype(&compare)> openlist(compare);
// std::priority_queue<int, std::vector<int>, decltype(&compare)> openlist(compare);

// using std::vector;
int main(){

    std::vector<prm_node>graph;
    // std::unordered_set<prm_node> closedlist;
    // std::priority_queue<int, decltype(&compare)> openlist(compare);
    prm_node dummy_node;

    for(int i=0; i<10;i++){
        dummy_node.g = rand();
        graph.push_back(dummy_node);
        std::cout<<graph.back().g<<std::endl;
    }
    std::cout<<"---"<<std::endl;
    for(int i=0; i<10;i++){
        openlist.push(graph[i]);
    }

    while(!openlist.empty()){
        std::cout<<openlist.top().g<<std::endl;
        openlist.pop();
    }



    return 0;
}

