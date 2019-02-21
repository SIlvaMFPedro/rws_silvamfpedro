#include <iostream>
#include <vector>
#include <ros/ros.h>

using namespace std;

namespace rws_silvamfpedro {

    class Player {
    public:
        //properties
        string player_name;

        Player(string player_name_in) { //constructor
            player_name = player_name_in;
            team_name = "";
        }

        void setTeamName(string team_name_in) {
            if (team_name_in == "red" || team_name_in == "blue" || team_name_in == "green") {
                team_name = team_name_in;
            } else {
                cout << "Invalid team name" << team_name_in << endl;
                team_name = "";
            }
        }

        void setTeamName(int team_index) {
            switch (team_index) {
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

        std::string getTeamName() {
            return team_name;
        }

    private:
        std::string team_name = "";

    };

    class MyPlayer : public Player {
    public:
        MyPlayer(string player_name_in, string team_name_in) : Player(player_name_in) {
            setTeamName(team_name_in);
        }

    private:
    };

    class Team{
    public:
        string team_name;
        vector<string> player_names;
        Team(string team_name_in){ //constructor
            team_name = team_name_in;
        }
        void printTeamInfo(){
            cout << "Team " << team_name << " has players: " << endl;
            for(size_t i = 0; i < player_names.size(); i++){
                cout << player_names.at(i) << endl;
            }
        }
    private:

    };
};

int main(int argc, char** argv){
    //ros::init(argc, argv, "player_psilva");
    //ros::NodeHandle n;

    //for(int i = 0; i < 10; i++){
    //    std::cout << i << std::endl;
    //}
    cout << "Hello World" << endl;
    rws_silvamfpedro::Player player("psilva");
    cout << "Hello World from psilva" << endl;
    player.setTeamName("blue");
    cout << "Hello World from psilva of team " << player.getTeamName() << endl;
    player.setTeamName(0);
    cout << "Hello World from psilva of team " << player.getTeamName() << endl;

    rws_silvamfpedro::MyPlayer my_player("silvamfpedro", "blue");
    cout << "Hello world from " << my_player.player_name << " of team " << my_player.getTeamName() << endl;

    rws_silvamfpedro::Team red_team("red");
    red_team.player_names.emplace_back("psilva1");
    red_team.player_names.emplace_back("psilva2");
    red_team.player_names.emplace_back("psilva3");
    red_team.player_names.emplace_back("psilva4");



    red_team.printTeamInfo();

    return 1;
}