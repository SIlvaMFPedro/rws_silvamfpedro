#include <iostream>
#include <ros/ros.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "player_psilva");
    ros::NodeHandle n;

    for(int i = 0; i < 10; i++){
        std::cout << i << std::endl;
        
    }
    std::cout << "Hello World" << std::endl;
    return 1;
}