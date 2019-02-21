#include <iostream>
#include <ros/ros.h>

class Player{
public:
    //properties
    std::string player_name;

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

    void setTeamName(int team_index){
        switch(team_index){
            case 0:
                setTeamName("red");
                break;
            case 1:
                setTeamName("blue");
                break;
            case 2:
                setTeamName("green");
                break;
            default:
                setTeamName("");
                break;
        }
    }

    std::string getTeamName(){
        return team_name;
    }
private:
    std::string team_name = "";

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
    std::cout << "Hello World from psilva of team " << player.getTeamName() << std::endl;
    player.setTeamName(0);
    std::cout << "Hello World from psilva of team " << player.getTeamName() << std::endl;
    return 1;
}