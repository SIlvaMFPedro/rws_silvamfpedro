#include <iostream>
#include <ros/ros.h>

class Player{
public:
    //properties
    std::string player_name;
    std::string team_name = "";

    Player(std::string player_name_in){ //constructor
        player_name = player_name_in;
        team_name = "";
    }

    void setTeamName(std::string team_name_in){
        if(team_name_in == "red" || team_name_in == "blue" || team_name_in == "green"){
            team_name = team_name_in;
        }
        else{
            std::cout << "Invalid team name" << team_name_in << std::endl;
            team_name = "";
        }
    }
private:

};

int main(int argc, char** argv){
    //ros::init(argc, argv, "player_psilva");
    //ros::NodeHandle n;

    //for(int i = 0; i < 10; i++){
    //    std::cout << i << std::endl;
    //}
    std::cout << "Hello World" << std::endl;
    Player player("psilva");
    std::cout << "Hello World from psilva" << std::endl;
    player.setTeamName("blue");
    std::cout << "Hellow World from psilva of team blue" << std::endl;
    return 1;
}