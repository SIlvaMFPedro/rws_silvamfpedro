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
        ros::NodeHandle nh;
        Team(string team_name_in){ //constructor
            team_name = team_name_in;
            //read team players
            nh.getParam("/team_" + team_name, player_names);

        }
        void printTeamInfo(){
            cout << "Team " << team_name << " has players: " << endl;
            for(size_t i = 0; i < player_names.size(); i++){
                cout << player_names.at(i) << endl;
            }
        }
        void checkTeamInfo(string player_name){
            cout << player_name << " belongs to red team: " << playerBelongsToTeam(player_name) << endl;
        }
        bool playerBelongsToTeam(string player_name){
            bool teamplayer = false;
            for(size_t i = 0; i < player_names.size(); i++){
                if(player_names.at(i) == player_name) {
                    teamplayer = true;
                    break;
                }
            }
            return teamplayer;
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
//    cout << "Hello World" << endl;
//    rws_silvamfpedro::Player player("psilva");
//    cout << "Hello World from psilva" << endl;
//    player.setTeamName("blue");
//    cout << "Hello World from psilva of team " << player.getTeamName() << endl;
//    player.setTeamName(0);
//    cout << "Hello World from psilva of team " << player.getTeamName() << endl;
//
//    rws_silvamfpedro::MyPlayer my_player("silvamfpedro", "blue");
//    cout << "Hello world from " << my_player.player_name << " of team " << my_player.getTeamName() << endl;
//
//    rws_silvamfpedro::Team red_team("red");
//    red_team.player_names.emplace_back("psilva1");
//    red_team.player_names.emplace_back("psilva2");
//    red_team.player_names.emplace_back("psilva3");
//    red_team.player_names.emplace_back("psilva4");
//
//    red_team.checkTeamInfo("psilva5");

    ros::init(argc, argv, "player_silvamfpedro");
    ros::NodeHandle nh;

    rws_silvamfpedro::MyPlayer player("silvamfpedro", "blue");
    cout << "Hello World from " << player.player_name << " of team " << player.getTeamName() << endl;
    rws_silvamfpedro::Team blue_team("blue");
    rws_silvamfpedro::Team green_team("green");
    rws_silvamfpedro::Team red_team("red");
    //blue_team.player_names.emplace_back("psilva1");
    //blue_team.player_names.emplace_back("psilva2");
    //blue_team.printTeamInfo();

    while(ros::ok()){
//        if(blue_team.playerBelongsToTeam("psilva1") == 1){
//            cout << "The player belongs to the blue team" << endl;
//        }
//        else{
//            cout << "The player does not belong to the blue team" << endl;
//        }
        blue_team.printTeamInfo();
        cout << " " << endl;
        green_team.printTeamInfo();
        cout << " " << endl;
        red_team.printTeamInfo();
        cout << " " << endl;
        cout << "psilva1 belongs to the red team " << red_team.playerBelongsToTeam("psilva1") << endl;
        cout << " " << endl;
        cout << "silvamfpedro belongs to the green team " << green_team.playerBelongsToTeam("silvamfpedro") << endl;
        cout << " " << endl;
        cout << "pedrosilva2 belongs to the blue team " << blue_team.playerBelongsToTeam("pedrosilva2") << endl;
        cout << " " << endl;
        ros::Duration(1).sleep();
    }
    //ros::Duration(1).sleep();
    return 1;
}