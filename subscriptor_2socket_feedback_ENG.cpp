 /*  
   @Developer José Luis Samper Escudero <Jluisse@gmail.com>, 
	      Joaquín Macanás Valera <macanas_92@hotmail.com
   @date   Thu 10 Jul 2014 10:00:00


    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

/**********
****	As example, this subscriptor gives a treatment to the feedback information.
****	In this case, sensor's data are advertised on the control topic.
**********/

#include <cstdlib>
#include <iostream>

#include "/home/samper/arduino_ros/src/Librerias_2Socket.h"  //Contains all the classes that have been developed for this project.


int main(int argc, char **argv)
  {
     ros::init(argc, argv, "feedback_sub");
     ros::NodeHandle nh;   
     sub_feedback_coche sub(nh);

     ros::spin();

   return 0;
  }

sub_feedback_coche::sub_feedback_coche(ros::NodeHandle nh){

       ros::Subscriber Sub_Feedback = nh.subscribe("car_feedback", 1, &sub_feedback_coche::callbackFeedback,this);
  	   ros::Publisher Feedback_A_Control = nh.advertise<std_msgs::String>("car_control",1, true);

   }



void sub_feedback_coche::callbackFeedback(const std_msgs::String::ConstPtr& msg){

	Feedback_A_Control.publish(msg);
}
