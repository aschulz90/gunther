/*** INCLUDES ***/
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Float32MultiArray.h"
#include "math.h"
#include "std_msgs/Bool.h"
#include "stdlib.h"

// Sockets
#include <stdio.h>
#include <sys/types.h>
#include <netdb.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>


/*** PRE-PROZESSOR VARIABLEN ***/
// auto oder manuelles fahren
#define _key_manual_on 1
#define _key_manual_off 0

// grad pro schritt im ranges array
#define _degrees_per_step 0.3515625
// benötigte breite des roboters
#define _gunther_width 0.6
// unterer fehlerwert des laserscanners
#define _ranges_min_error 0.05

// variablen zum berechnen ob der weg vor dem roboter frei ist
// distanz die betrachtet wird
#define _gapfront_dist 0.7
// rückgabewert wenn ein hindernis links/rechts im weg steht
#define _gapfront_left 2
#define _gapfront_right 3
// rückgabewert wenn kein hindernis im weg steht
#define _gapfront_none 0

// distanz zur betrachtung ob unmittelbar vor dem roboter ein hindernis ist
#define _obstaclefront_dist 0.3

// marker topic array indizes
// x wert
#define _marker_x 0
// y wert
#define _marker_y 1
// distanz
#define _marker_dist 2
// winkel im kamerabild
#define _marker_angle 3
// marker aktuell sichtbar
#define _marker_visible 4
// marker jemals sichtbar
#define _marker_seen_once 5
// winkel im kamerabild bei dem keine richtungskrrektur vorgenommen wird
#define _marker_angle_error 0.2

// verhalten des roboter (status)
// marker niemals gefunden
#define _state_never_found 0
// folge marker
#define _state_follow_marker 1
// hindernis links/rechts im weg
#define _state_obstacle_left 2
#define _state_obstacle_right 3
// manuelle steuerung
#define _state_manual 4
// follow laser
#define _state_follow_laser 5

/*** CLASS DEFINITION ***/
class gunther
{
public:
    ros::NodeHandle nh;
    // subscriber und publisher
    ros::Subscriber laser_sub;
    ros::Publisher motor_pub;
	ros::Subscriber cam_sub;
	ros::Subscriber key_sub;
	ros::Subscriber key_manual_sub;

    // globale variablen
    geometry_msgs::Twist motor_data;
	std_msgs::Float32MultiArray cam_data;
	geometry_msgs::Twist key_data;

	int current_state;
	float gapfront_angle;
	float obstaclefront_angle;
	float obstacle_angle_left;
	float obstacle_angle_right;
	int marker_counter;
	int laser_counter;
	int wait_for_marker_counter;
	int wait_for_marker_invi_counter;
	bool explorer_wait;
	float angle_dec;
	float last_middle;
	
	// Socket
	int sockfd;

    // funktionen
    void init(char** argv);
    void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
	void camCallback(const std_msgs::Float32MultiArray::ConstPtr& msg);
	void keyCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void follow_marker(const sensor_msgs::LaserScan::ConstPtr& msg);
	void follow_laser(const sensor_msgs::LaserScan::ConstPtr& msg);
	int is_obstacle(const sensor_msgs::LaserScan::ConstPtr& msg);
	int obstacle_near(const sensor_msgs::LaserScan::ConstPtr& msg);
	void key_manualCallback(const std_msgs::Bool::ConstPtr& msg);
};

/*** MAIN FUNCTION ***/
int main(int argc, char** argv)
{
    ROS_INFO("main");
    ros::init(argc, argv, "gunther");

    // instanz der klasse erstellen und init ausführen
    gunther t;
    t.init(argv);

    ros::spin();
	close(t.sockfd);
}

/*** INIT FUNCTION ***/
void gunther::init(char** argv)
{
	ROS_INFO("init");
    // initialisierung der subscriber und publisher
    laser_sub = nh.subscribe("scan", 1, &gunther::laserCallback, this);
    motor_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	cam_sub = nh.subscribe("marker_data", 1, &gunther::camCallback, this);
	key_sub = nh.subscribe("key_vel", 1, &gunther::keyCallback, this);
	key_manual_sub = nh.subscribe("key_manual", 1, &gunther::key_manualCallback, this);

	ROS_INFO("pub&sub finished");

    // füllen des vektors für die kameradaten
	for(int i=0; i<6; i++) {
		cam_data.data.push_back(0);
	
	}

	ROS_INFO("cam data push finished");

    // winkelberechnung für berechnung ob weg vor dem roboter frei ist
	gapfront_angle = atan(_gunther_width / (2 * _gapfront_dist)) * (180 / M_PI);
	// winkelberechnung für berechnung ob unmittelbar vor dem roboter ein hindernis ist
	obstaclefront_angle = atan(_gunther_width / (2 * _obstaclefront_dist)) * (180 / M_PI);
    
	// aktuellen status setzten
	current_state = _state_never_found;
	//current_state = _state_follow_laser;	
	explorer_wait = false;

	ROS_INFO("calc finshed");

	sockfd = socket(AF_INET, SOCK_STREAM, 0);
        
	if(sockfd == -1) {
		    perror("Socket konnte nicht erstellt werden");
			return;		 	
			//   return -1;
	}    

	// Remote-Adresse
	struct sockaddr_in raddr;
	raddr.sin_family = AF_INET;
	raddr.sin_port = htons(55555);
	inet_aton(argv[1], &raddr.sin_addr);
		
	// Verbindung herstellen
	int ret = connect(sockfd, (struct sockaddr*) &raddr, sizeof(raddr));

	if(ret == -1) {
		    perror("Verbindung konnte nicht hergestellt werden");
			return;		   
			// return -1;
	} 

	ROS_INFO("tcp finished");

	/*
	// senden
	int n = write(sockfd,"1",1);
	if (n < 0) perror("ERROR writing to socket");  
	*/
	ROS_INFO("init finished");
}



/*** CALLBACK FUNCTION OF CAMERA ***/
void gunther::camCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    // falls marker sichtbar und vorher schon #-mal sichtbar war werden neue markerdaten übernommen
	if(msg->data.at(_marker_visible) == 1 && msg->data.at(_marker_y) > -60 && msg->data.at(_marker_y) < 60) {
		marker_counter++;
		if(marker_counter >= 6) {
			cam_data = *msg;
			cam_data.data.at(_marker_angle) *= -1;
			marker_counter = 0;
		}
	} else {
        // marker ist nicht sichtbar und nur angaben über aktuelle sichtbarkeit und winkel(=0) werden übernommen
		cam_data.data.at(_marker_visible) = msg->data.at(_marker_visible);
		//if (angle_dec == 0) angle_dec = cam_data.data.at(_marker_angle) / 8;
		cam_data.data.at(_marker_angle) = msg->data.at(_marker_angle);
	}
    // wenn marker in einem bestimmten bereich um die mitte des kamerabildes gefunden wurde behalte die aktuelle richtung bei
	if(cam_data.data.at(_marker_angle) < _marker_angle_error && cam_data.data.at(_marker_angle) > -_marker_angle_error) {
		cam_data.data.at(_marker_angle) = 0;
	}
    // falls der marker das erste mal gefunden wurde wechsle in den marker folgen status
	//ROS_INFO("cam: %f", cam_data.data.at(_marker_seen_once));
	if(current_state == _state_never_found && cam_data.data.at(_marker_seen_once) == 1) {
		current_state = _state_follow_marker;
	}
    // falls roboter im umfahrungsstatus ist und der marker sichtbar ist wechsle in den marker folgen status
	if((current_state == _state_obstacle_left || current_state == _state_obstacle_right) && cam_data.data.at(_marker_visible) == 1) {
		current_state = _state_follow_marker;
	}
}

/*** CALLBACK FUNCTION OF LASERSCANNER ***/
void gunther::laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
	if(cam_data.data.at(_marker_visible) == 0) {
		wait_for_marker_counter++;
		wait_for_marker_invi_counter++;
	}
	if (cam_data.data.at(_marker_visible) == 1) {
		wait_for_marker_counter = 0;
		wait_for_marker_invi_counter = 0;
		if(explorer_wait) {
			int n = write(sockfd,"0",1);
			if (n < 0) perror("ERROR writing to socket");
			explorer_wait = !explorer_wait;
		}
	}
	if(wait_for_marker_counter > 60) {
		wait_for_marker_counter = 0;
		if(!explorer_wait) {
			int n = write(sockfd,"1",1);
			if (n < 0) perror("ERROR writing to socket");
			explorer_wait = !explorer_wait;
		}
	}
	//ROS_INFO("links %f  #  links-mitte %f", msg->ranges[511], msg->ranges[(int) ((msg->ranges.size() / 2) + (5 / _degrees_per_step))]);
 	//ROS_INFO("mitte %f", msg->ranges[(int) (msg->ranges.size() / 2)]);
	//ROS_INFO("rechts-mitte %f  #  rechts %f", msg->ranges[(int) ((msg->ranges.size() / 2) - (5 / _degrees_per_step))], msg->ranges[0]);
	ROS_INFO("state %i  #  marker %f", current_state, cam_data.data.at(_marker_visible));
    // je nach aktuellem status auszuführenden code bestimmen
	switch(current_state) {
	case _state_never_found:
        // marker niemals gefunden -> drehe weiter im kreis
		motor_data.angular.z = -1.0;
		break;
	case _state_follow_marker:
        // marker folgen -> rufe marker folgen methode auf
		follow_marker(msg);
		break;
	case _state_obstacle_left:
		if(obstacle_near(msg)) {
			motor_data.angular.z = -0.5;
			motor_data.linear.x = 0;
			break;
		}
        // hindernis links -> umfahre hindernis (falls nah genug) mit linker hand regel
		if(msg->ranges[msg->ranges.size()-50] > (float) 0.6 || msg->ranges[msg->ranges.size()-50] < (float) 0.05) {
		    motor_data.angular.z = 0.5;
		    motor_data.linear.x = 0.4;
		} else if(msg->ranges[msg->ranges.size()-50] < (float) 0.5) {
		    motor_data.angular.z = -0.5;
		    motor_data.linear.x = 0.4;
		} else {
		    motor_data.angular.z = 0;
		    motor_data.linear.x = 0.4;
		}
		break;
	case _state_obstacle_right:
		if(obstacle_near(msg)) {
			motor_data.angular.z = 0.5;
			motor_data.linear.x = 0;
			break;
		}
        // hindernis rechts -> umfahre hindernis (falls nah genug) mit rechter hand regel
		if(msg->ranges[50] > (float) 0.6 || msg->ranges[50] < (float) 0.05) {
		    motor_data.angular.z = -0.5;
		    motor_data.linear.x = 0.4;
		} else if(msg->ranges[50] < (float) 0.5) {
		    motor_data.angular.z = 0.5;
		    motor_data.linear.x = 0.4;
		} else {
		    motor_data.angular.z = 0;
		    motor_data.linear.x = 0.4;
		}
		break;
	case _state_manual:
        // manuelle steuerung -> sende daten von der tastatur an den motor
		motor_data.linear.x = key_data.linear.x;
		motor_data.angular.z = key_data.angular.z;
		break;
	case _state_follow_laser:
		follow_laser(msg);
		break;
	default:
		ROS_INFO("FAIL");
		break;
	}
    // veröffentliche motordaten
	motor_pub.publish(motor_data);
}

/*** State 'FOLLOW LASER' FUNCTTION ***/
void gunther::follow_laser(const sensor_msgs::LaserScan::ConstPtr& msg){
	// Winkel zurücksetzen	
	motor_data.angular.z = 0;
	// Wenn der mittlere Strahl verloren wird, suche neue Mitte 
	if(abs(last_middle - msg->ranges[(int) (msg->ranges.size() / 2)]) > 0.15 || msg->ranges[(int) (msg->ranges.size() / 2)] == 0){
		if (cam_data.data.at(_marker_visible) == 1 && cam_data.data.at(_marker_y) < 60 && cam_data.data.at(_marker_y) > -60) 
			motor_data.angular.z = cam_data.data.at(_marker_angle); 
		else for (int i = 4; i < (msg->ranges.size() / 2); i++){
			if (msg->ranges[(int) (msg->ranges.size() / 2) - i] != 0 && 
				msg->ranges[(int) (msg->ranges.size() / 2) - i] < last_middle + 0.3){
				motor_data.angular.z = -1;
				break;
				}
			if (msg->ranges[(int) (msg->ranges.size() / 2) + i] != 0 && 
                                msg->ranges[(int) (msg->ranges.size() / 2) + i] < last_middle + 0.3){
                                motor_data.angular.z = 1;
                                break;
                                }

		}
	}

	// Abstand mit Hilfe des mittleren Strahls einhalten
	if(msg->ranges[(int) (msg->ranges.size() / 2)] > 0.70) {
		motor_data.linear.x = 0.3;
		last_middle = msg->ranges[(int) (msg->ranges.size() / 2)];
	} else {
		motor_data.linear.x = 0;
	}

	if(explorer_wait && msg->ranges[(int) (msg->ranges.size() / 2)] < 0.80) {
			int n = write(sockfd,"0",1);
			if (n < 0) perror("ERROR writing to socket");
			explorer_wait = !explorer_wait;
		}

	if(!explorer_wait && msg->ranges[(int) (msg->ranges.size() / 2)] > 1.5) {
			int n = write(sockfd,"1",1);
			if (n < 0) perror("ERROR writing to socket");
			explorer_wait = !explorer_wait;
		}

	// Wenn der linke oder rechte Strahl verloren wird, entsprechend einlenken
	if(msg->ranges[(int) ((msg->ranges.size() / 2) - (8 / _degrees_per_step))] > msg->ranges[(int) (msg->ranges.size() / 2)] + 0.60 ||
			msg->ranges[(int) ((msg->ranges.size() / 2) - (8 / _degrees_per_step))] == 0.0 &&
			motor_data.angular.z == 0) {
		motor_data.angular.z = 0.8;
		//if (cam_data.data.at(_marker_visible) == 1) motor_data.angular.z = cam_data.data.at(_marker_angle);
	} else if (msg->ranges[(int) ((msg->ranges.size() / 2) - (8 / _degrees_per_step))] < msg->ranges[(int) (msg->ranges.size() / 2)] - 0.60 &&
			motor_data.angular.z == 0) {
		motor_data.angular.z = -0.8;
		//if (cam_data.data.at(_marker_visible) == 1) motor_data.angular.z = cam_data.data.at(_marker_angle);
	}
	if((msg->ranges[(int) ((msg->ranges.size() / 2) + (8 / _degrees_per_step))] > msg->ranges[(int) (msg->ranges.size() / 2)] + 0.60 ||
			msg->ranges[(int) ((msg->ranges.size() / 2) + (8 / _degrees_per_step))] == 0.0) &&
			motor_data.angular.z == 0) {
		motor_data.angular.z = -0.8;
		//if (cam_data.data.at(_marker_visible) == 1) motor_data.angular.z = cam_data.data.at(_marker_angle);
	} else if (msg->ranges[(int) ((msg->ranges.size() / 2) - (8 / _degrees_per_step))] < msg->ranges[(int) (msg->ranges.size() / 2)] - 0.60 &&
			motor_data.angular.z == 0){
		motor_data.angular.z = 0.8;
		//if (cam_data.data.at(_marker_visible) == 1) motor_data.angular.z = cam_data.data.at(_marker_angle);
	}
	//if(msg->ranges[0] < 0.8 && msg->ranges[0] != 0.0) {
	//	motor_data.angular.z = 1;
	//}
	//if(msg->ranges[511] < 0.8 && msg->ranges[511] != 0) {
	//	motor_data.angular.z = -1;
	//}
}

/*** STATE 'FOLLOW MARKER' FUNCTION ***/
void gunther::follow_marker(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // setze bewegungswinkel
	//if (cam_data.data.at(_marker_angle) < -0.03 || cam_data.data.at(_marker_angle) > 0.03) cam_data.data.at(_marker_angle) -= angle_dec; 
	//	else angle_dec = 0;
	motor_data.angular.z = cam_data.data.at(_marker_angle);
    // falls der marker sichtbar und näher als 1 meter, dann stoppe, sonst fahre weiter
	if(cam_data.data.at(_marker_dist) < 1000 && cam_data.data.at(_marker_visible) == 1) {
        	motor_data.linear.x = 0;
		motor_data.angular.z /= 2;
    	} else if(wait_for_marker_invi_counter >= 20 || cam_data.data.at(_marker_visible) == 1){
    	motor_data.linear.x = 0.51;
	} else {
		motor_data.linear.x = 0;
	}


    // berechne ob der weg vor dem roboter frei ist und führe entsprechenden code aus
	switch(is_obstacle(msg)) {
	case _gapfront_left:
        // links vor roboter ist ein hindernis -> halte an und wechsle in umfahrungsmodus(linke hand)
		current_state = _state_obstacle_left;
		motor_data.linear.x = 0;
		break;
	case _gapfront_right:
        // rechts vor roboter ist ein hindernis -> halte an und wechsle in umfahrungsmodus(rechte hand)
		current_state = _state_obstacle_right;
		motor_data.linear.x = 0;
		break;
	case _gapfront_none:
        // vor roboter ist kein hindernis -> tue nichts
		break;
	}
	if(cam_data.data.at(_marker_dist) < 1100) {
		current_state = _state_follow_laser;
	}
}

/*** LOOK FOR OBSTACLE FUNCTION ***/
int gunther::is_obstacle(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // mitte des ranges array bestimmen
	int rangesmid = msg->ranges.size() / 2;
    // untersuche die werte des ranges array in einem bestimmten winkel ob ein hindernis vor dem roboter vorhanden ist und gebe dementsprechend einen wert zurück
	for(int i = 0; i < (int)(gapfront_angle/_degrees_per_step) - 2; i++) {
		if(msg->ranges[rangesmid+i] < _gapfront_dist && msg->ranges[rangesmid+i] > _ranges_min_error &&
			msg->ranges[rangesmid+i+1] < _gapfront_dist && msg->ranges[rangesmid+i+1] > _ranges_min_error &&
			msg->ranges[rangesmid+i+2] < _gapfront_dist && msg->ranges[rangesmid+i+2] > _ranges_min_error) {
			return _gapfront_left;
		} else if(msg->ranges[rangesmid-i] < _gapfront_dist && msg->ranges[rangesmid-i] > _ranges_min_error &&
			msg->ranges[rangesmid-i-1] < _gapfront_dist && msg->ranges[rangesmid-i-1] > _ranges_min_error &&
			msg->ranges[rangesmid-i-2] < _gapfront_dist && msg->ranges[rangesmid-i-2] > _ranges_min_error) {
			return _gapfront_right;
		}
	}
	return _gapfront_none;
}

/*** CALLBACK FUNCTION FOR KEYBOARD ***/
void gunther::keyCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    // übernehme die daten von der tastatur
	key_data = *msg;
}

void gunther::key_manualCallback(const std_msgs::Bool::ConstPtr& msg)
{
	if(msg->data) {
		current_state = _state_manual;
	} else {
		current_state = _state_never_found;
	}
}

/*** LOOK FOR OBSTACLE DIRECT IN FRONT ***/
int gunther::obstacle_near(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    // mitte des ranges array bestimmen
	int rangesmid = msg->ranges.size() / 2;
    // untersuche die werte des ranges array in einem bestimmten winkel ob ein hindernis vor dem roboter vorhanden ist und gebe dementsprechend einen wert zurück
	for(int i = 0; i < (int)(obstaclefront_angle/_degrees_per_step) - 2; i++) {
		if(msg->ranges[rangesmid-i] < _gapfront_dist && msg->ranges[rangesmid-i] > _ranges_min_error &&
			msg->ranges[rangesmid-i+1] < _gapfront_dist && msg->ranges[rangesmid-i+1] > _ranges_min_error &&
			msg->ranges[rangesmid-i+2] < _gapfront_dist && msg->ranges[rangesmid-i+2] > _ranges_min_error) {
			return 1;
		} else if(msg->ranges[rangesmid+i] < _gapfront_dist && msg->ranges[rangesmid+i] > _ranges_min_error &&
			msg->ranges[rangesmid+i+1] < _gapfront_dist && msg->ranges[rangesmid+i+1] > _ranges_min_error &&
			msg->ranges[rangesmid+i+2] < _gapfront_dist && msg->ranges[rangesmid+i+2] > _ranges_min_error) {
			return 1;
		}
	}
	return 0;
}
