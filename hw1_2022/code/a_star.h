#include <iostream>

class node{
    public:
        float x = 0.0;
        float y = 0.0;
        float parent_x = 0.0;
        float parent_y = 0.0;
        float g = 100000;
        float h = 0.0;
        float f = g + h;
        node *parent_ptr;
        node *child_ptr;
        int state = 0; // 0 = not assigned, 1 = open, 2 = closed
};


node * tie_breaker(node *ptr1, node *ptr2 ){
    node *result;
    std::cout<<"Executing tie-breaker:"<<std::endl;
    if (ptr1->f == ptr2->f){
        if (ptr1->h > ptr2->h){result = ptr1;}
        else if (ptr1->h < ptr2->h){result = ptr2;}
    }
    else {result = NULL;};

    return result;
}

int node_info(node node_0){

    std::cout<<"x:---------------> "<<node_0.x<<std::endl;
    std::cout<<"y:---------------> "<<node_0.y<<std::endl;
    std::cout<<"parent_x:--------> "<<node_0.parent_x<<std::endl;
    std::cout<<"parent_y:--------> "<<node_0.parent_y<<std::endl;
    std::cout<<"g:---------------> "<<node_0.g<<std::endl;
    std::cout<<"h:---------------> "<<node_0.h<<std::endl;
    std::cout<<"f:---------------> "<<node_0.f<<std::endl;
    std::cout<<"parent_ptr:------> "<<node_0.parent_ptr<<std::endl;
    std::cout<<"child_ptr:-------> "<<node_0.child_ptr<<std::endl;
    std::cout<<"state:-----------> "<<node_0.state<<std::endl;

    return 0;
};