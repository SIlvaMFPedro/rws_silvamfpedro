#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>


using namespace std;
using namespace boost;
using namespace ros;

float randomizePosition()
{
    srand(6526*time(NULL)); // set initial seed value to 5323
    return (((double)rand() / (RAND_MAX)) - 0.5) * 10;
}

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

                //define intial position
                float sx = randomizePosition();
                float sy = randomizePosition();
                tf::Transform T1;
                T1.setOrigin( tf::Vector3(sx/2, sy/2, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, M_PI);
                T1.setRotation(q);

                //define global movement
                tf::Transform Tglobal = T1;
                tb.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getPlayerName()));

                ros::Duration(0.1).sleep();
                printInfo();
            }
            void printInfo(){
                ROS_INFO_STREAM("My name is " << this->getPlayerName() << " and my team is " << this->getTeamName());
                ROS_WARN_STREAM("My name is " << this->getPlayerName() << " and my hunters are " << team_hunters->getName());
                ROS_ERROR_STREAM("My name is " << this->getPlayerName() << " and my preys are " << team_preys->getName());
            }
            void makeAPlayCallBack(rws2019_msgs::MakeAPlayConstPtr msg){
                ROS_INFO("Received a new ROS message");

                //STEP 1: Find out where I am
                tf::StampedTransform T0;
                try{
                    tl.lookupTransform("/world", this->getPlayerName(), ros::Time(0), T0);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.1).sleep();
                }

                //STEP 2: define how I want to move
                float dx = 1.25;
                float angle = M_PI/6;

                //STEP 3: define local movement
                tf::Transform T1;
                T1.setOrigin( tf::Vector3(dx, 0.0, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, angle);
                T1.setRotation(q);

                //STEP 4: define global movement
                tf::Transform Tglobal = T0*T1;
                tb.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getPlayerName()));
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
            tf::TransformListener tl;
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

    ros::Rate r(20);
    while(ros::ok()){
        //ros::Duration(1).sleep();
        player.printInfo();
        spinOnce();
    }

    return 1;
}