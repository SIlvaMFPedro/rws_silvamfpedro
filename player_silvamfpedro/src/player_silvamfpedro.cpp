#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <rws2019_msgs/MakeAPlay.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

using namespace std;
using namespace boost;
using namespace ros;

float randomize_Position(int n){
    srand(n*time(NULL)); // set initial seed value to 5323
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
            vector<string> getPlayerNames(){
                return this->player_names;
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

                NodeHandle n;
                vis_pub = (boost::shared_ptr<Publisher>) new Publisher;
                (*vis_pub) = n.advertise<visualization_msgs::Marker>("player_names", 0);
                bocas_pub = (boost::shared_ptr<ros::Publisher>) new ros::Publisher;
                (*bocas_pub) = n.advertise<visualization_msgs::Marker>("bocas", 0);
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
                float sx = randomize_Position(6526);
                float sy = randomize_Position(3213);
                tf::Transform T1;
                T1.setOrigin( tf::Vector3(sx/2, sy/2, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, M_PI);
                T1.setRotation(q);

                //define global movement
                tf::Transform Tglobal = T1;
                tb.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getPlayerName()));
                ros::Duration(0.1).sleep();
                tb.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getPlayerName()));
                printInfo();
            }
            void printInfo(){
                ROS_INFO_STREAM("My name is " << this->getPlayerName() << " and my team is " << this->getTeamName());
                ROS_WARN_STREAM("My name is " << this->getPlayerName() << " and my hunters are " << team_hunters->getName());
                ROS_ERROR_STREAM("My name is " << this->getPlayerName() << " and my preys are " << team_preys->getName());
            }
            std::tuple<float,float> getDistanceAndAngleToPlayer(string other_player)
            {
                tf::StampedTransform T0;
                try{
                    tl.lookupTransform(this->getPlayerName(), other_player, ros::Time(0), T0);
                }
                catch (tf::TransformException ex){
                    ROS_ERROR("%s",ex.what());
                    ros::Duration(0.01).sleep();
                    return {1000, 0};
                }

                float d = sqrt(T0.getOrigin().x() * T0.getOrigin().x() + T0.getOrigin().y() * T0.getOrigin().y() );
                float a = atan2(T0.getOrigin().y(), T0.getOrigin().x());
                return {d, a};
            }
            std::tuple<float, float> getDistanceAndAngleToWorld(){
                return getDistanceAndAngleToPlayer("world");
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

                vector<float> distance_to_preys;
                vector<float> angle_to_preys;

                vector<float> distance_to_hunters;
                vector<float> angle_to_hunters;

                //For each prey find the closest. Then follow it
                for (size_t i =0; i< team_preys->getPlayerNames().size(); i++)
                {
                    ROS_WARN_STREAM("team_preys = " << team_preys->getPlayerNames().at(i));
                    std::tuple<float, float> tuple = getDistanceAndAngleToPlayer(team_preys->getPlayerNames().at(i));
                    distance_to_preys.push_back(std::get<0>(tuple));
                    angle_to_preys.push_back(std::get<1>(tuple));

                }
                //For each hunter find the closest. Then run away
                for (size_t i = 0; i < team_hunters->getPlayerNames().size(); i++){
                    ROS_WARN_STREAM("team_hunters = " << team_hunters->getPlayerNames().at(i));
                    std::tuple<float, float> tuple = getDistanceAndAngleToPlayer(team_hunters->getPlayerNames().at(i));
                    distance_to_hunters.push_back(std::get<0>(tuple));
                    angle_to_hunters.push_back(std::get<1>(tuple));
                }

                //Compute closest prey and closest hunter;
                int idx_closest_prey = 0;
                int idx_closest_hunter = 0;
                float distance_closest_prey = 1000;
                float distance_closest_hunter = 1000;
                //Compute closest prey
                for(size_t i = 0; i  < distance_to_preys.size(); i++) {
                    if (distance_to_preys[i] < distance_closest_prey) {
                        idx_closest_prey = i;
                        distance_closest_prey = distance_to_preys[i];
                    }
                }
                //Compute closest hunter
                for(size_t i = 0; i < distance_to_hunters.size(); i++){
                    if (distance_to_hunters[i] < distance_closest_hunter){
                        idx_closest_hunter = i;
                        distance_closest_hunter = distance_to_hunters[i];
                    }
                }

                //STEP 2: define how I want to move
                float dx = 10;
                float angle = angle_to_preys[idx_closest_prey];
                float angle_hunter = angle_to_hunters[idx_closest_hunter];
                //Compare prey and hunter distance
                if(distance_closest_hunter < distance_closest_prey){
                    //angle = -angle_hunter;
                    angle = -angle_to_hunters[idx_closest_prey];
                }
                else{
                    angle = angle_to_preys[idx_closest_prey];
                }
                string prey = team_preys->getPlayerNames().at(idx_closest_prey);
                string hunter = team_hunters->getPlayerNames().at(idx_closest_hunter);
                string boca = "I'm coming for you " + prey + " and i am running away from " + hunter;

                float distance_to_arena_center;
                float angle_to_arena_center;

                // get arena center coordinates
                std::tuple<float, float> t = getDistanceAndAngleToWorld();
                distance_to_arena_center = std::get<0>(t);
                angle_to_arena_center = std::get<1>(t);

                if (distance_to_arena_center > 7)
                {
                    angle = angle * M_PI/30;
                }

//                float angle = angle_to_preys[idx_closest_prey];


                //STEP 2.5: check values
                float dx_max = msg->cheetah;
                dx > dx_max ? dx = dx_max : dx = dx;

                double angle_max = M_PI/30;
                fabs(angle) > fabs(angle_max) ? angle = angle_max*angle/fabs(angle) : angle = angle;

                //STEP 3: define local movement
                tf::Transform T1;
                T1.setOrigin( tf::Vector3(dx, 0.0, 0.0) );
                tf::Quaternion q;
                q.setRPY(0, 0, angle);
                T1.setRotation(q);

                //STEP 4: define global movement
                tf::Transform Tglobal = T0*T1;
                tb.sendTransform(tf::StampedTransform(Tglobal, ros::Time::now(), "world", this->getPlayerName()));

                visualization_msgs::Marker marker;
                marker.header.frame_id = this->getPlayerName();
                marker.header.stamp = ros::Time();
                marker.ns = this->getPlayerName();
                marker.id = 0;
                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.action = visualization_msgs::Marker::ADD;
//                marker.pose.position.x = 1;
//                marker.pose.position.y = 1;
//                marker.pose.position.z = 1;
//                marker.pose.orientation.x = 0.0;
//                marker.pose.orientation.y = 0.0;
//                marker.pose.orientation.z = 0.0;
//                marker.pose.orientation.w = 1.0;
//                marker.scale.x = 1;
//                marker.scale.y = 0.1;
                marker.scale.z = 0.6;
                marker.color.a = 1.0; // Don't forget to set the alpha!
                marker.color.r = 0.0;
                marker.color.g = 1.0;
                marker.color.b = 0.0;
                marker.text = "TROLOLOLOLOLOL";
                //only if using a MESH_RESOURCE marker type:
                //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";
                vis_pub->publish( marker );

                visualization_msgs::Marker bocas_marker;
                bocas_marker.header.frame_id = this->getName();
                bocas_marker.header.stamp = ros::Time();
                bocas_marker.ns = this->getName();
                bocas_marker.id = 0;
                bocas_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                bocas_marker.action = visualization_msgs::Marker::ADD;
                bocas_marker.scale.z = 0.4;
                bocas_marker.color.a = 1.0; // Don't forget to set the alpha!
                bocas_marker.pose.position.y = 0.4;
                bocas_marker.color.r = 0.0;
                bocas_marker.color.g = 0.0;
                bocas_marker.color.b = 0.0;
                bocas_marker.text = boca;

                bocas_pub->publish( bocas_marker );
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
            boost::shared_ptr<Publisher> vis_pub;
            boost::shared_ptr<Publisher> bocas_pub;
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