#include <iostream>
#include <ros/ros.h>

using namespace std;

class Player{
public:
    //properties
    string player_name;

    Player(string player_name_in){ //constructor
        player_name = player_name_in;
        team_name = "";
    }

    void setTeamName(string team_name_in){
        if(team_name_in == "red" || team_name_in == "blue" || team_name_in == "green"){
            team_name = team_name_in;
        }
        else{
            cout << "Invalid team name" << team_name_in << endl;
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

class MyPlayer : public Player
{
public:
    MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in){
        setTeamName(team_name_in);
    }
private:
};

int main(int argc, char** argv){
    //ros::init(argc, argv, "player_psilva");
    //ros::NodeHandle n;

    //for(int i = 0; i < 10; i++){
    //    std::cout << i << std::endl;
    //}
    cout << "Hello World" << endl;
    Player player("psilva");
    cout << "Hello World from psilva" << endl;
    player.setTeamName("blue");
    cout << "Hello World from psilva of team " << player.getTeamName() << endl;
    player.setTeamName(0);
    cout << "Hello World from psilva of team " << player.getTeamName() << endl;

    MyPlayer my_player("silvamfpedro", "blue");
    cout << "Hello world from " << my_player.player_name << " of team " << my_player.getTeamName() << endl;
    return 1;
}