#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace boost;
using namespace ros;

namespace rws_silvamfpedro {
    class Team{
        public:
            explicit Team(string team_name){ //constructor
                this->team_name = team_name;
                //read team players
                nh.getParam("/team_" + team_name, player_names);
            }
            bool playerBelongsToTeam(const string &player_name){
                bool team_player = false;
                for(string& player : player_names){
                    if(player == player_name){
                        team_player = true;
                    }
                }
                return team_player;
            }
            string getName(){
                return this->team_name;
            }
            void addPlayer(string player_name){
                player_names.push_back(player_name);
            }

        private:
            string team_name;
            vector<string> player_names;
            NodeHandle nh;
    };

    class Player {
        public:
            explicit Player(string player_name) { //constructor
                    this->player_name = player_name;
                    this->team_name = "";
            }
            void setTeamName(const string &team_name_in) {
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
            string getPlayerName(){
                return this->player_name;
            }
            string getTeamName() {
                return team_name;
            }

        private:
            //properties
            string player_name;
            string team_name;

    };

    class MyPlayer : public Player {
        public:
            MyPlayer(const string &player_name, string team_name) : Player(player_name) {
                //create team
                team_red = (boost::shared_ptr<Team>) new Team("red");
                team_blue = (boost::shared_ptr<Team>) new Team("blue");
                team_green = (boost::shared_ptr<Team>) new Team("green");
                //create hunter teams
                if (team_red->playerBelongsToTeam(player_name)){
                    team_mine = team_red;
                    team_preys = team_green;
                    team_hunters = team_blue;
                }
                else if(team_green->playerBelongsToTeam(player_name)){
                    team_mine = team_green;
                    team_preys = team_blue;
                    team_hunters = team_red;
                }
                else if(team_blue->playerBelongsToTeam(player_name)){
                    team_mine = team_blue;
                    team_preys = team_red;
                    team_hunters = team_green;
                }
                else{
                    cout << "Wrong Team Info" << endl;
                }
                setTeamName(team_mine->getName());
                //printInfo();
            }
            void printInfo(){
                ROS_INFO_STREAM("My name is " << this->getPlayerName() << " and my team is " << this->getTeamName());
                ROS_WARN_STREAM("My name is " << this->getPlayerName() << " and my hunters are " << team_hunters->getName());
                ROS_ERROR_STREAM("My name is " << this->getPlayerName() << " and my preys are " << team_preys->getName());
            }
            void makeAPlayCallBack(rws2019_msgs::MakeAPlayConstPtr msg){
                ROS_INFO("Received a new ROS message");
                //public transform
                tf::Transform transform1;
                transform1.setOrigin( tf::Vector3(5,5,0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, 0);
                transform1.setRotation(q);
                tb.sendTransform(tf::StampedTransform(transform1, ros::Time::now(), "world", this->getPlayerName()));
            }

        private:
            //teams
            boost::shared_ptr<Team> team_red;
            boost::shared_ptr<Team> team_green;
            boost::shared_ptr<Team> team_blue;
            boost::shared_ptr<Team> team_mine;
            boost::shared_ptr<Team> team_hunters;
            boost::shared_ptr<Team> team_preys;
            tf::TransformBroadcaster tb;
            tf::Transform transform;
    };


};

int main(int argc, char** argv){
    //Initialize ROS node
    init(argc, argv, "player_psilva");
    NodeHandle nh;

    //Create Instance of Class MyPlayer
    rws_silvamfpedro::MyPlayer player("psilva", "blue");
    cout << "Hello World from " << player.getPlayerName() << " of team " << player.getTeamName() << endl;

    //Create Team
    rws_silvamfpedro::Team green_team("green");

    //Create ROS Subscriber
    Subscriber sub = nh.subscribe("/make_a_play", 100, &rws_silvamfpedro::MyPlayer::makeAPlayCallBack, &player);

    while(ros::ok()){
        ros::Duration(1).sleep();
        player.printInfo();
        spinOnce();
    }

    return 1;
}