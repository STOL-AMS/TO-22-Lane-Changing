/*==========================================================================*/
/*  DriverModel.cpp                                  DLL Module for VISSIM  */
/*                                                                          */
/*  Interface module for external driver models.                            */
/*  Implement CAV control model and export CAV trajectory information       */
/*==========================================================================*/

#include "DriverModel.h"
#include <stdio.h>
#include <iostream> 
#include <fstream>
#include <list>
#include <math.h>
#include <ctime>
#include <map>  
#include <string> 
#include <stdlib.h>
#include <vector>
#include <cmath>
#include <iomanip>
#define random(a,b) (rand()%(b-a+1)+a)

using namespace std;
using std::vector;
vector<long>::iterator iter;

vector<vector<float>> veh_data(1000);
int www = 0;


/*==========================================================================*/
float k_1 = 0.2; //ACC param1
float k_2 = 0.68; //ACC param2
float react_time = 0.9;//tau
float b_av = 4.5;//CAV max dec
float b_hv = 4.2;//max dec of preceding vehicle
float w_hv = 4.0;//max acc of following vehicle
float s_0 = 4.0;//min following gap
float time_gap=1.3;//time gap
float g_cav = 1.53;// CAV desired time gap
float veh_length = 4.6; //vehicle length for CAV and HV
float b = 0.2;//used for estimate LC duration
float lw = 3.5;//lane width
float av_off_main_rate = 0.2;//mainline CAV diverging rate
float av_off_ramp_rate = 0.2;//on ramp CAV diverging rate
float coop_rate = 0.5;// CAV cooperation rate
float d_a = 0.1;// incentive check threshold
float d_bias = 0.6;//incentive check bias
/*HV diverging rate; CAV penetration rate; Zone A and B length can be customized in PTV VISSIM user interface*/


double  current_speed        = 0.0;
double  desired_acceleration = 0.0;
double  desired_lane_angle   = 0.0;
long    active_lane_change   = 0;
long    rel_target_lane      = 0;
double  desired_velocity     = 0.0;
long    turning_indicator    = 0;
long    vehicle_color        = RGB(0,0,0);
double  lead_vehicle_distance         = 61;
double  lead_vehicle_speed_difference = -9.0;
double  lead_vehicle_length           =   0.0;
double time_run;
long current_lane;
double lateral_distance;
double net_distance;
double lead_vehicle_speed;
double desired_distance; /* times 1 s */
double veh_odometer;
long current_link;
long inter_model = 0;
double veh_angle;

long down_vehicle_type;
long up_vehicle_type;
long down_left_vehicle_type;
long down_right_vehicle_type;
long up_left_vehicle_type;
long up_right_vehicle_type;
long veh_type;


long veh_id;

double down_left_veh_distance;
double down_right_veh_distance;
double up_left_veh_distance;
double up_right_veh_distance;
double up_vehicle_distance;

long up_vehicle_lane_change;
long down_left_vehicle_lane_change;
long down_right_vehicle_lane_change;
long up_left_vehicle_lane_change;
long up_right_vehicle_lane_change;

double down_right_veh_sp_diff;
double down_left_veh_sp_diff;
double up_left_veh_sp_diff;
double up_right_veh_sp_diff;

double down_right_veh_sp;
double down_left_veh_sp;
double up_left_veh_sp;
double up_right_veh_sp;

double veh_x_po;
double veh_y_po;

float acc_follow_own_lane;
float acc_follow_nveh_lane;
int change_right_condition = 0;
int change_left_condition = 0;
float angle = 0.05;
map<long, int> target_lane;
map<long, float> active_change_time;
map<long, float> change_lane_period;
map<long, long> last_link;
map<long, float> last_angel;
float start_pos_1 = 0;
float dist_adjust = 0;
map<long, int>veh_aim;
map<long, int>coop;


float right_change_follow;
float left_change_follow;
float suiji;

float right_future_fol_dist = 88;
float right_future_fol_acc = -10;
float left_future_fol_dist = 88;
float left_future_fol_acc = -10;

ofstream fout,fav_out;
ifstream fin;
/*==========================================================================*/

BOOL APIENTRY DllMain (HANDLE  hModule,
                       DWORD   ul_reason_for_call,
                       LPVOID  lpReserved)
{
  switch (ul_reason_for_call) {
      case DLL_PROCESS_ATTACH:
      case DLL_THREAD_ATTACH:
      case DLL_THREAD_DETACH:
      case DLL_PROCESS_DETACH:
         break;
  }
  return TRUE;
}

/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelSetValue (long   type,
                                           long   index1,
                                           long   index2,
                                           long   long_value,
                                           double double_value,
                                           char   *string_value)
{
  /* Sets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, to <long_value>, <double_value> or            */
  /* <*string_value> (object and value selection depending on <type>).    */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_PATH                   :
    case DRIVER_DATA_TIMESTEP               :
    case DRIVER_DATA_TIME                   :
		time_run = double_value;
      return 1;
    case DRIVER_DATA_USE_UDA                :
      return 0; /* doesn't use any UDAs */
                /* must return 1 for desired values of index1 if UDA values are to be sent from/to Vissim */
    case DRIVER_DATA_VEH_ID:
		veh_id = long_value;
      lead_vehicle_distance         = 70.0;
	  down_left_veh_distance=70;
	  down_right_veh_distance=70;
	  up_left_veh_distance = -70;
	  up_right_veh_distance = -70;
	  up_vehicle_distance = -70;
      lead_vehicle_speed_difference = 0;
	  lead_vehicle_speed = 32;
      lead_vehicle_length           =   4.6;
	  down_right_veh_sp=32;
	  down_left_veh_sp = 32;
	  up_left_veh_sp = 29;
	  up_right_veh_sp = 29;
      return 1;
    case DRIVER_DATA_VEH_LANE               :
		current_lane = long_value;
    case DRIVER_DATA_VEH_ODOMETER           :
		veh_odometer = double_value;
    case DRIVER_DATA_VEH_LANE_ANGLE         :
		veh_angle = double_value;
    case DRIVER_DATA_VEH_LATERAL_POSITION   :
		lateral_distance = double_value;
      return 1;
    case DRIVER_DATA_VEH_VELOCITY           :
      current_speed = double_value;
      return 1;
    case DRIVER_DATA_VEH_ACCELERATION       :
    case DRIVER_DATA_VEH_LENGTH             :
    case DRIVER_DATA_VEH_WIDTH              :
    case DRIVER_DATA_VEH_WEIGHT             :
    case DRIVER_DATA_VEH_MAX_ACCELERATION   :
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR  :
      turning_indicator = long_value;     //LC signal
      return 1;
    case DRIVER_DATA_VEH_CATEGORY           :
    case DRIVER_DATA_VEH_PREFERRED_REL_LANE :
    case DRIVER_DATA_VEH_USE_PREFERRED_LANE :
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
      desired_velocity = double_value;
      return 1;
    case DRIVER_DATA_VEH_X_COORDINATE       :
		veh_x_po = double_value;
		return 1;
    case DRIVER_DATA_VEH_Y_COORDINATE       :
		veh_y_po = double_value;
		return 1;
    case DRIVER_DATA_VEH_Z_COORDINATE       :
    case DRIVER_DATA_VEH_REAR_X_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Y_COORDINATE  :
    case DRIVER_DATA_VEH_REAR_Z_COORDINATE  :
    case DRIVER_DATA_VEH_TYPE               :
		veh_type = long_value;
      return 1;
    case DRIVER_DATA_VEH_COLOR              :
      vehicle_color = long_value;
      return 1;
    case DRIVER_DATA_VEH_CURRENT_LINK       :
		current_link = long_value;
      return 0; /* (To avoid getting sent lots of DRIVER_DATA_VEH_NEXT_LINKS messages) */
                /* Must return 1 if these messages are to be sent from VISSIM!         */
    case DRIVER_DATA_VEH_NEXT_LINKS         :
    case DRIVER_DATA_VEH_ACTIVE_LANE_CHANGE :
    case DRIVER_DATA_VEH_REL_TARGET_LANE    :
    case DRIVER_DATA_VEH_INTAC_STATE        :
    case DRIVER_DATA_VEH_INTAC_TARGET_TYPE  :
    case DRIVER_DATA_VEH_INTAC_TARGET_ID    :
    case DRIVER_DATA_VEH_INTAC_HEADWAY      :
    case DRIVER_DATA_VEH_UDA                :
    case DRIVER_DATA_NVEH_ID                :
    case DRIVER_DATA_NVEH_LANE_ANGLE        :
    case DRIVER_DATA_NVEH_LATERAL_POSITION  :
      return 1;
    case DRIVER_DATA_NVEH_DISTANCE          :
      if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
        lead_vehicle_distance = double_value;
      }
	  else if (index1 == 0 && index2 == -1)
	  {
		  up_vehicle_distance = double_value;
	  }
	  else if (index1 == -1 && index2 == 1) { /* right lane leading veh */
		  down_right_veh_distance = double_value;

	  }
	  else if (index1 == -1 && index2 == -1) { /* right lane following veh */
		  up_right_veh_distance = double_value;

	  }
	  else if (index1 == 1 && index2 == 1) { /* left lane leading veh */
		  down_left_veh_distance = double_value;

	  }
	  else if (index1 == 1 && index2 == -1) { /* left lane following veh */
		  up_left_veh_distance = double_value;

	  }
      return 1;
    case DRIVER_DATA_NVEH_REL_VELOCITY      :
		/* double: speed difference [m/s] (veh. speed - nveh. speed) */
      if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
        lead_vehicle_speed_difference = double_value;
		lead_vehicle_speed = current_speed - lead_vehicle_speed_difference;
      }
	  else if (index1 == -1 && index2 == 1) { /* right lane leading veh */
		  down_right_veh_sp_diff = double_value;
		  down_right_veh_sp = current_speed - down_right_veh_sp_diff;
	  }
	  else if (index1 == -1 && index2 == -1) { /* right lane follwoing veh */
		  up_right_veh_sp_diff = double_value;
		  up_right_veh_sp = current_speed - up_right_veh_sp_diff;
	  }
	  else if (index1 == 1 && index2 == 1) { /* left lane leading veh */
		  down_left_veh_sp_diff = double_value;
		  down_left_veh_sp = current_speed - down_left_veh_sp_diff;
	  }
	  else if (index1 == 1 && index2 == -1) { /* left lane follwoing veh */
		  up_left_veh_sp_diff = double_value;
		  up_left_veh_sp = current_speed - up_left_veh_sp_diff;
	  }
      return 1;
    case DRIVER_DATA_NVEH_ACCELERATION      :
      return 1;
    case DRIVER_DATA_NVEH_LENGTH            :
      if (index1 == 0 && index2 == 1) { /* leading vehicle on own lane */
        lead_vehicle_length = double_value;
      }
      return 1;
    case DRIVER_DATA_NVEH_WIDTH             :
    case DRIVER_DATA_NVEH_WEIGHT            :
    case DRIVER_DATA_NVEH_TURNING_INDICATOR :
    case DRIVER_DATA_NVEH_CATEGORY          :
    case DRIVER_DATA_NVEH_LANE_CHANGE       :
		if (index1 == 0 && index2 == -1) { /* following vehicle on own lane */
			up_vehicle_lane_change = long_value;
		}
		else if (index1 == -1 && index2 == 1) { /* right lane leading veh */
			down_right_vehicle_lane_change = long_value;
		}
		else if (index1 == -1 && index2 == -1) { /* right lane following veh */
			up_right_vehicle_lane_change = long_value;
		}
		else if (index1 == 1 && index2 == 1) { /* left lane leading veh */
			down_left_vehicle_lane_change = long_value;
		}
		else if (index1 == 1 && index2 == -1) { /* left lane following veh */
			up_left_vehicle_lane_change = long_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_TYPE              :
		if (index1 == 0 && index2 == 1) { /* leading veh on own lane */
			down_vehicle_type = long_value;
		}
		else if (index1 == 0 && index2 == -1) { /* following vehicle on own lane */
			up_vehicle_type = long_value;
		}
		else if (index1 == -1 && index2 == 1) { /* right lane leading veh */
			down_right_vehicle_type = long_value;
		}
		else if (index1 == -1 && index2 == -1) { /* right lane following veh */
			up_right_vehicle_type = long_value;
		}
		else if (index1 == 1 && index2 == 1) { /* left lane leading veh */
			down_left_vehicle_type = long_value;
		}
		else if (index1 == 1 && index2 == -1) { /* left lane following veh */
			up_left_vehicle_type = long_value;
		}
		return 1;
    case DRIVER_DATA_NVEH_UDA               :
    case DRIVER_DATA_NVEH_X_COORDINATE      :
    case DRIVER_DATA_NVEH_Y_COORDINATE      :
    case DRIVER_DATA_NVEH_Z_COORDINATE      :
    case DRIVER_DATA_NVEH_REAR_X_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Y_COORDINATE :
    case DRIVER_DATA_NVEH_REAR_Z_COORDINATE :
    case DRIVER_DATA_NO_OF_LANES            :
    case DRIVER_DATA_LANE_WIDTH             :
    case DRIVER_DATA_LANE_END_DISTANCE      :
    case DRIVER_DATA_CURRENT_LANE_POLY_N    :
    case DRIVER_DATA_CURRENT_LANE_POLY_X    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Y    :
    case DRIVER_DATA_CURRENT_LANE_POLY_Z    :
    case DRIVER_DATA_RADIUS                 :
    case DRIVER_DATA_MIN_RADIUS             :
    case DRIVER_DATA_DIST_TO_MIN_RADIUS     :
    case DRIVER_DATA_SLOPE                  :
    case DRIVER_DATA_SLOPE_AHEAD            :
    case DRIVER_DATA_SIGNAL_DISTANCE        :
    case DRIVER_DATA_SIGNAL_STATE           :
    case DRIVER_DATA_SIGNAL_STATE_START     :
    case DRIVER_DATA_SPEED_LIMIT_DISTANCE   :
    case DRIVER_DATA_SPEED_LIMIT_VALUE      :
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION :
      desired_acceleration = double_value;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      desired_lane_angle = double_value;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      active_lane_change = long_value;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
      rel_target_lane = long_value;
      return 1;
    default :
      return 0;
  }
}

/*--------------------------------------------------------------------------*/

DRIVERMODEL_API  int  DriverModelGetValue (long   type,
                                           long   index1,
                                           long   index2,
                                           long   *long_value,
                                           double *double_value,
                                           char   **string_value)
{
  /* Gets the value of a data object of type <type>, selected by <index1> */
  /* and possibly <index2>, and writes that value to <*double_value>,     */
  /* <*float_value> or <**string_value> (object and value selection       */
  /* depending on <type>).                                                */
  /* Return value is 1 on success, otherwise 0.                           */

  switch (type) {
    case DRIVER_DATA_STATUS :
      *long_value = 0;
      return 1;
    case DRIVER_DATA_VEH_TURNING_INDICATOR :
      *long_value = turning_indicator;
      return 1;
    case DRIVER_DATA_VEH_DESIRED_VELOCITY   :
		desired_velocity = 27;
		*double_value = desired_velocity + (rand() % 6 - 3);//-3~3 random number
      return 1;
    case DRIVER_DATA_VEH_COLOR :
      *long_value = vehicle_color;
      return 1;
    case DRIVER_DATA_VEH_UDA :
      return 0; /* doesn't set any UDA values */
    case DRIVER_DATA_WANTS_SUGGESTION :
      *long_value = 1;  
      return 1;
    case DRIVER_DATA_DESIRED_ACCELERATION : 
      *double_value = desired_acceleration;
      return 1;
    case DRIVER_DATA_DESIRED_LANE_ANGLE :
      *double_value = desired_lane_angle;
      return 1;
    case DRIVER_DATA_ACTIVE_LANE_CHANGE :
      *long_value = active_lane_change;
      return 1;
    case DRIVER_DATA_REL_TARGET_LANE :
		if (rel_target_lane*desired_lane_angle < 0)
		{
			fout << time_run << "  " << veh_id << "  " << veh_aim[veh_id] << "  " << current_link << "----" << current_lane << "  " << active_lane_change << "<LC direction，angle>" << desired_lane_angle << "  " << change_right_condition << "  " << change_left_condition << "??????????999999" << endl;
		}
      *long_value = rel_target_lane;
      return 1;
    case DRIVER_DATA_SIMPLE_LANECHANGE :
      *long_value = 0;  
      return 1;
    case DRIVER_DATA_USE_INTERNAL_MODEL:
	 *long_value = 0;/* must be set to 1 if the internal behavior model of Vissim is to be applied */
    case DRIVER_DATA_WANTS_ALL_NVEHS:
      *long_value = 0; /* must be set to 1 if data for more than 2 nearby vehicles per lane and upstream/downstream is to be passed from Vissim */
      return 1;
    case DRIVER_DATA_ALLOW_MULTITHREADING:
      *long_value = 0; /* must be set to 1 to allow a simulation run to be started with multiple cores used in the simulation parameters */
      return 1;
    default:
      return 0;
  }
}





void follow_turn()
{
	
	if (current_speed > (desired_velocity + 0.2))
	{
		acc_follow_own_lane = -(current_speed - desired_velocity);
	}
	else
	{
		if (lead_vehicle_distance > 69)
		{
			acc_follow_own_lane = 0.5;
		}
		else
		{
			acc_follow_own_lane = k_1 * (lead_vehicle_distance - lead_vehicle_length - dist_adjust - current_speed * g_cav) - k_2 * lead_vehicle_speed_difference;
		}
		
	}


	//fout << time_run << "  " << veh_id << "  " << current_link << "--" << current_lane << "  " << current_speed << "   " << acc_follow_own_lane << "  " << desired_acceleration << "  " << lead_vehicle_distance << "  " << lead_vehicle_speed_difference << "  " << active_lane_change << "+++111111" << endl;
	//fout << time_run << "  " << veh_id << "  " << current_link << "--" << current_lane << acc_follow_own_lane << "  " << desired_acceleration << "  " << lead_vehicle_distance << "  " << lead_vehicle_speed_difference << "  " << active_lane_change << endl;

	if (active_lane_change == 0)
	{
		if ((down_right_vehicle_lane_change > 0) && (down_right_vehicle_type == 630) && (coop[veh_id] == 1))
		{
			left_change_follow = k_1 * (down_right_veh_distance - lead_vehicle_length - dist_adjust - current_speed * g_cav) - k_2 * down_right_veh_sp_diff;
		}
		else
		{
			left_change_follow = 4.5;
		}
		if ((down_left_vehicle_lane_change < 0) && (down_left_vehicle_type == 630) && (coop[veh_id] == 1))
		{
			right_change_follow = k_1 * (down_left_veh_distance - lead_vehicle_length - dist_adjust - current_speed * g_cav) - k_2 * down_left_veh_sp_diff;
		}
		else
		{
			right_change_follow = 4.5;
		}


		desired_acceleration = min(min(left_change_follow, right_change_follow), acc_follow_own_lane);
		//desired_acceleration = acc_follow_own_lane;
	}
	else
	{
		if (current_lane != target_lane[veh_id])
		{

			if (active_lane_change > 0)
			{
				//acc_follow_nveh_lane = k_1 * (down_left_veh_distance - current_speed * react_time) - k_2 * down_left_veh_sp_diff;
				
				acc_follow_nveh_lane = k_1 * (down_left_veh_distance - lead_vehicle_length - dist_adjust - current_speed * g_cav) - k_2 * down_left_veh_sp_diff;

			}
			else
			{
				//acc_follow_nveh_lane = k_1 * (down_right_veh_distance - current_speed * react_time) - k_2 * down_right_veh_sp_diff;
				
				acc_follow_nveh_lane = k_1 * (down_right_veh_distance - lead_vehicle_length - dist_adjust - current_speed * g_cav) - k_2 * down_right_veh_sp_diff;
			}

		}
		else
		{
			
			if ((active_lane_change > 0) && (lateral_distance < (-1.75 - 1.3*sin(veh_angle))))
			{
				//acc_follow_nveh_lane = k_1 * (down_right_veh_distance - current_speed * react_time) - k_2 * down_right_veh_sp_diff;
				acc_follow_nveh_lane = k_1 * (down_right_veh_distance - lead_vehicle_length - dist_adjust - current_speed * g_cav) - k_2 * down_right_veh_sp_diff;
				
			}
			else if ((active_lane_change < 0) && (lateral_distance > (1.75 + 1.3*sin(veh_angle))))
			{
				//acc_follow_nveh_lane = k_1 * (down_left_veh_distance - current_speed * react_time) - k_2 * down_left_veh_sp_diff;
				acc_follow_nveh_lane = k_1 * (down_left_veh_distance - lead_vehicle_length - dist_adjust - current_speed * g_cav) - k_2 * down_left_veh_sp_diff;
				
			}
				else
			{
				acc_follow_nveh_lane = 9;
			}
		}

		desired_acceleration = max(min(acc_follow_own_lane, acc_follow_nveh_lane), -b_av);

	}

	//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << acc_follow_own_lane << "  " << desired_acceleration << "  " << lead_vehicle_distance << "  " << lead_vehicle_speed_difference << "  " << active_lane_change << endl;
	//fout << time_run << "  " << veh_id << "  " << current_link << "--" << current_lane << "  " << current_speed << "   " << acc_follow_own_lane << "  " << desired_acceleration << "  " << lead_vehicle_distance << "  " << lead_vehicle_speed_difference << "  " << active_lane_change << "+++111111" << endl;
}

void turn_judge()
{
	//if ((right_future_fol_dist <= (down_right_veh_distance - veh_length)) && (right_future_fol_acc >= -b_hv) && (up_right_veh_distance < -5) && ((down_right_veh_distance - lead_vehicle_distance) > -1.5) && ((down_right_veh_distance - lead_vehicle_distance) < 1.5))
	//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << "  " << change_left_condition << "  " << left_future_fol_dist << "  " << down_left_veh_distance << "  " << left_future_fol_acc << "  " << down_left_veh_sp << "  " << up_left_veh_sp << "====11111" << endl;
	//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << "  " << change_right_condition << "  " << right_future_fol_dist << "  " << down_right_veh_distance << "  " << right_future_fol_acc << "  " << down_right_veh_sp << "  " << up_right_veh_sp << "====11111" << endl;


	if (((current_lane > 1) && (current_link != 1) && (current_link != 3) && (current_link != 10002)) || ((current_lane > 3) && ((current_link == 3) || (current_link == 10002))) || ((current_lane > 2) && (current_link ==1 || current_link == 10000)))
	{
		right_future_fol_dist = current_speed * react_time + current_speed * current_speed*0.5 / b_av - down_right_veh_sp * down_right_veh_sp*0.5 / b_hv;
		right_future_fol_acc = w_hv * (1 - pow((up_right_veh_sp / down_right_veh_sp), 4) - pow(((s_0 + max(0, (up_right_veh_sp*time_gap + up_right_veh_sp * (down_right_veh_sp - up_right_veh_sp)*0.5 / sqrt(w_hv*b_hv)))) / right_future_fol_dist), 2));
		if (current_link == 6 && current_lane > 1 && veh_aim[veh_id] == 1)
		
		{
	

			right_future_fol_acc = -3;
		}
	}
	else
	{
		right_future_fol_dist = 200;
		right_future_fol_acc = -20;
	}




	if ((((veh_aim[veh_id] != 1) && (current_lane < 3) && (current_link != 2) && (current_link != 10001) && (current_link != 10) && (current_link != 10007))) || ((current_link == 3 || current_link == 10003) && current_lane == 1))
	{
	
		left_future_fol_dist = current_speed * react_time + current_speed * current_speed*0.5 / b_av - down_left_veh_sp * down_left_veh_sp*0.5 / b_hv;
		left_future_fol_acc = w_hv * (1 - pow((up_left_veh_sp / down_left_veh_sp), 4) - pow(((s_0 + max(0, (up_left_veh_sp*time_gap + up_left_veh_sp * (down_left_veh_sp - up_left_veh_sp)*0.5 / sqrt(w_hv*b_hv)))) / left_future_fol_dist), 2));

		//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << "  " << change_left_condition << "  " << left_future_fol_dist << "  " << down_left_veh_distance << "  " << left_future_fol_acc << "  " << down_left_veh_sp << "  " << up_left_veh_sp <<"==333333"<< endl;
	}
	else
	{
		left_future_fol_dist = 200;
		left_future_fol_acc = -20;
	}

	//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << "  " << change_right_condition << "  " << right_future_fol_dist << "  " << down_right_veh_distance << "  " << right_future_fol_acc << "  " << down_right_veh_sp << "  " << up_right_veh_sp << "====22222" << endl;

	if (down_left_veh_distance > 69 || down_right_veh_distance > 69)
	{
		down_left_veh_sp = up_left_veh_sp;
		left_future_fol_acc = -3;
		down_right_veh_sp = up_right_veh_sp;
		right_future_fol_acc = -3;
		//left_future_fol_dist = 8;
		//right_future_fol_dist = 8;
	}
	
	

	
	if ((right_future_fol_dist <= (down_right_veh_distance - veh_length)) && (right_future_fol_acc >= -b_hv))
	{

		if ((current_link == 3 || current_link == 10002) && current_lane == 2)
		{
				change_right_condition = 0;
				
		}
		else
		{
			
			if (down_right_veh_distance > 6 && up_right_veh_distance < -10)
			{
				if (current_link == 6 && current_lane > 1)
				{
					change_right_condition = 1;
				}
				else
				{
					//incentive check
					acc_follow_nveh_lane = k_1 * (down_right_veh_distance - lead_vehicle_length - dist_adjust - current_speed * react_time) - k_2 * down_right_veh_sp_diff;
					if ((acc_follow_nveh_lane - acc_follow_own_lane) > (d_a - d_bias))
					{
						change_right_condition = 1;
					}
					else
					{
						change_right_condition = 0;
					}
				}

			}
			else
			{
				change_right_condition = 0;
			}
			
		}

	}
	else if ((left_future_fol_dist <= (down_left_veh_distance - veh_length)) && (left_future_fol_acc >= -b_hv))
	{
	
		if (down_left_veh_distance > 8  && (up_left_veh_distance < -10 || down_left_veh_sp < 1))
		{
			if (current_link == 3 && current_lane == 1)
			{
				change_left_condition = 1;
			}
			else
			{
				//incentive check
				acc_follow_nveh_lane = k_1 * (down_left_veh_distance - lead_vehicle_length - dist_adjust - current_speed * react_time) - k_2 * down_left_veh_sp_diff;
				if ((acc_follow_nveh_lane - acc_follow_own_lane) > (d_a + d_bias))
				{
					change_left_condition = 1;
				}
				else
				{
					change_left_condition = 0;
				}

			}
		}
		else
		{
			change_left_condition = 0;
		}

		//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << "  " << change_left_condition << "  " << active_lane_change <<"   5555555"<< endl;
	}
	else
	{
		change_right_condition = 0;
		change_left_condition = 0;
	}

	//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << "  " << change_left_condition << "  " << left_future_fol_dist << "  " << down_left_veh_distance << "  " << left_future_fol_acc << "  " << down_left_veh_sp << "  " << up_left_veh_sp << "====666666"<<endl;
	//fout << time_run << "  " << veh_id << "  " << current_link << "----" << current_lane << "  " << change_right_condition << "  " << right_future_fol_dist << "  " << down_right_veh_distance << "  " << right_future_fol_acc << "  " << down_right_veh_sp << "  " << up_right_veh_sp << "====3333333" << endl;
}


void acc_angle()
{
	if (active_lane_change*desired_lane_angle < 0 || rel_target_lane * desired_lane_angle < 0)
	{
		fout << time_run << "  " << veh_id << "  " << veh_aim[veh_id] << "  " << current_link << "----" << current_lane << "  " << active_lane_change << "  " << rel_target_lane << "  " << target_lane[veh_id] << "<LC direction，angle>" << desired_lane_angle << "  " << change_right_condition << "  " << change_left_condition << "????????????111111" << endl;
	}



	//fout << time_run << "  " << veh_id << "  " << current_link << "--" << current_lane << "  " << active_lane_change << "  " << change_left_condition << "  " << desired_lane_angle << "  " << lateral_distance << endl;
	//fout << time_run << "  " << veh_id << "  " << current_lane << "  " << active_lane_change << "  " << desired_lane_angle << "  " << veh_angle << "  " << "3333333" << endl;
	//fout << time_run << "  " << veh_id << "  " << veh_aim[veh_id] << "  " << current_link << "----" << current_lane << "  " << active_lane_change << "<LC direction，angle>" << desired_lane_angle << "  " << change_right_condition << "  " << change_left_condition << "===1111111" << endl;
	if (active_lane_change == 0)
	{
		//if ((change_right_condition == 1) && (right_hv_con == 1))
		if (change_right_condition == 1)
		{

			active_lane_change = -1;
			rel_target_lane = -1;

			target_lane[veh_id] = current_lane + rel_target_lane;
			desired_lane_angle = active_lane_change * 0.05;
			active_change_time[veh_id] = time_run;
			change_lane_period[veh_id] = b * current_speed;


		}
		//else if ((change_left_condition == 1) && (left_hv_con == 1))
		else if (change_left_condition == 1)
		{
			active_lane_change = 1;
			rel_target_lane = 1;
			//change_left_condition = 0;

			target_lane[veh_id] = current_lane + rel_target_lane;
			desired_lane_angle = active_lane_change * 0.05;
			active_change_time[veh_id] = time_run;
			change_lane_period[veh_id] = b * current_speed;

		}
		else
		{
			active_lane_change = 0;
			rel_target_lane = 0;
			target_lane[veh_id] = 0;
			desired_lane_angle = 0;
		}

	}
	else
	{

		if ((active_lane_change == -1) && (target_lane[veh_id] == current_lane) && (lateral_distance < 0.03))
		{
			//LC to right ends
			active_lane_change = 0;
			rel_target_lane = 0;
			target_lane[veh_id] = 0;

		}
		else if ((active_lane_change == 1) && (target_lane[veh_id] == current_lane) && (lateral_distance > -0.03))
		{
			//LC to left ends 
			active_lane_change = 0;
			rel_target_lane = 0;
			target_lane[veh_id] = 0;

		}
		else if ((active_lane_change != 0) && (target_lane[veh_id] == 0))
		{

			active_lane_change = 0;
			rel_target_lane = 0;
			desired_lane_angle = 0;	
		}
		
		else if ((target_lane[veh_id] < current_lane) && (active_lane_change == -1))
		{
			if (change_right_condition == 0)
			{
				active_lane_change = 0;
				rel_target_lane = 0;
				target_lane[veh_id] = 0;
				desired_lane_angle = 0;
			}
			else
			{
				//angle = (lw + lateral_distance) / ((change_lane_period[veh_id] - time_run)*current_speed + 0.5*desired_acceleration*(change_lane_period[veh_id] - time_run)*(change_lane_period[veh_id] - time_run));
				angle = (lw + lateral_distance) / (down_right_veh_distance - right_future_fol_dist + down_right_veh_sp * (change_lane_period[veh_id] - time_run + active_change_time[veh_id]));
				desired_lane_angle = active_lane_change * atan(angle);
				//historical angle value
				if (active_lane_change*desired_lane_angle < 0)
				{
					desired_lane_angle = last_angel[veh_id];
				}
			}
		}
		else if ((target_lane[veh_id] > current_lane) && (active_lane_change == 1))
		{
			if (change_left_condition == 0)
			{
				active_lane_change = 0;
				rel_target_lane = 0;
				target_lane[veh_id] = 0;
				desired_lane_angle = -0.03;
				//vehicle_color = RGB(128, 128, 128) + 255 * 16777216;
			}
			else
			{
				angle = (lw - lateral_distance) / (down_left_veh_distance - left_future_fol_dist + down_left_veh_sp * (change_lane_period[veh_id] - time_run + active_change_time[veh_id]));
				desired_lane_angle = active_lane_change * atan(angle);
				//historical angle value
				if (active_lane_change*desired_lane_angle < 0)
				{
					desired_lane_angle = last_angel[veh_id];
				}
			
			}
		}
		else if (target_lane[veh_id] == current_lane)
		{
			if ((active_lane_change == -1) && (lateral_distance > (1.75 + 1.3*sin(veh_angle))))
			{
				// LC to right
				right_future_fol_dist = current_speed * react_time + current_speed * current_speed*0.5 / b_av - lead_vehicle_speed * lead_vehicle_speed*0.5 / b_hv;
				angle = lateral_distance / (lead_vehicle_distance - right_future_fol_dist + lead_vehicle_speed * (change_lane_period[veh_id] - time_run + active_change_time[veh_id]));
				desired_lane_angle = active_lane_change * atan(angle);

				//historical angle value
				if (active_lane_change*desired_lane_angle < 0)
				{
					desired_lane_angle = last_angel[veh_id];
				}

			}
			else if ((active_lane_change == 1) && (lateral_distance < (-1.75 - 1.3*sin(veh_angle))))
			{
				//LC to left
				left_future_fol_dist = current_speed * react_time + current_speed * current_speed*0.5 / b_av - lead_vehicle_speed * lead_vehicle_speed*0.5 / b_hv;
				angle = -lateral_distance / (lead_vehicle_distance - left_future_fol_dist + lead_vehicle_speed * (change_lane_period[veh_id] - time_run + active_change_time[veh_id]));
				desired_lane_angle = active_lane_change * atan(angle);

				//historical angle value
				if (active_lane_change*desired_lane_angle < 0)
				{
					desired_lane_angle = last_angel[veh_id];
				}
			}
		}
		else
		{
		active_lane_change = 0;
		rel_target_lane = 0;
		target_lane[veh_id] = 0;
		desired_lane_angle = 0;

		}

	}

	last_angel[veh_id] = desired_lane_angle;

	//fout << time_run << "  " << veh_id << "  " << current_link << "--" << current_lane << "  " << active_lane_change << "  " << change_left_condition << "  " << desired_lane_angle << "  " << lateral_distance << endl;
	//fout << time_run << "  " << veh_id << "  "  << active_lane_change << "<LC direction，angle>" << desired_lane_angle << "  " << down_left_veh_distance << "  " << left_future_fol_dist << "  " << down_left_veh_sp << "  " << lead_vehicle_speed << "===222222" << endl;

	//fout << right_future_fol_dist << "  " << current_speed << "  " << lead_vehicle_speed << "  " << lateral_distance << "  " << lead_vehicle_distance << "  " << change_lane_period[veh_id] << "  " << active_change_time[veh_id] << "  " << angle << endl;
	//fout << time_run << "  " << veh_id << "  " << veh_aim[veh_id] << "  " << current_link << "----" << current_lane << "  " << active_lane_change << "<LC direction，angle>" << desired_lane_angle << "  " << change_right_condition << "  " << change_left_condition <<"===3333333"<<endl;
	//==========================================================================
	if (active_lane_change*desired_lane_angle < 0 || rel_target_lane * desired_lane_angle < 0)
	{
		fout << time_run << "  " << veh_id << "  " << veh_aim[veh_id] << "  " << current_link << "----" << current_lane << "  " << active_lane_change << "  " << rel_target_lane << "  " << target_lane[veh_id] << "<LC direction，angle>" << desired_lane_angle << "  " << change_right_condition << "  " << change_left_condition << "????????????555555" << endl;
	}

}


//=====================================correct the LC angle
void correction_angle()
{
	
	if (active_lane_change == 0)
	{
		if (lateral_distance > 0.5)
		{
			desired_lane_angle = -0.1;
			//rel_target_lane = 0;
			target_lane[veh_id] = current_lane;
		}
		else if (lateral_distance < -0.5)
		{
			desired_lane_angle = 0.1;
			//rel_target_lane = 0;
			target_lane[veh_id] = current_lane;
		}
		else
		{
			desired_lane_angle = 0;
		}
	}
}







/*==========================================================================*/

DRIVERMODEL_API  int  DriverModelExecuteCommand (long number)
{
  /* Executes the command <number> if that is available in the driver */
  /* module. Return value is 1 on success, otherwise 0.               */

	switch (number) {
	case DRIVER_COMMAND_INIT:
		fout.open("data_put_out_AV.txt", std::ios_base::out);
		fout.flush();
		for (int i = 0; i < veh_data.size(); i++)
			veh_data[i].resize(0);
		return 1;
	case DRIVER_COMMAND_CREATE_DRIVER:
		target_lane[veh_id] = 0;
		active_change_time[veh_id] = 0;
		change_lane_period[veh_id] = 0;
		
		veh_aim[veh_id] = 0;
		last_link[veh_id] = 0;
		last_angel[veh_id] = 0;
		coop[veh_id] = 0;
		return 1;
	case DRIVER_COMMAND_KILL_DRIVER:
		target_lane.erase(veh_id);
		active_change_time.erase(veh_id);
		change_lane_period.erase(veh_id);
		
		veh_aim.erase(veh_id);
		last_link.erase(veh_id);
		last_angel.erase(veh_id);
		coop.erase(veh_id);
		return 1;
	case DRIVER_COMMAND_MOVE_DRIVER:

		net_distance = lead_vehicle_distance - lead_vehicle_length;
		desired_distance = 15 + lead_vehicle_length; /* times 1 s */
		change_left_condition = 0;
		change_right_condition = 0;

		//right_future_fol_dist = 999;
		//right_future_fol_acc = -8;
		//left_future_fol_dist = 999;
		//left_future_fol_acc = -8;
		

		//for checking states==========================================================================
		if (active_lane_change*desired_lane_angle < 0)
		{
			fout << time_run << "  " << veh_id << "  " << veh_aim[veh_id] << "  " << current_link << "----" << current_lane << "  " << active_lane_change << "<LC direction，angle>" << desired_lane_angle << "  " << change_right_condition << "  " << change_left_condition << "??????????888888" << endl;
		}



		if (current_link == 1 && veh_y_po < -250 && veh_aim[veh_id] == 0)
		{
			if ((time_run - floor(time_run)) < av_off_main_rate)
			{
				veh_aim[veh_id] = 1;//diverge
				vehicle_color = RGB(0, 255, 255) + 255 * 16777216;
				
			}
			else
			{
				veh_aim[veh_id] = 2;//through
				vehicle_color = RGB(0, 180, 255) + 255 * 16777216;
			}


			suiji = random(0, 100);
			if (suiji < coop_rate *100)
			{
				coop[veh_id] = 1;
			}
			else
			{
				coop[veh_id] = 2;
			}


		}
		else if (current_link == 2 && veh_y_po < 460 && veh_aim[veh_id] == 0)
		{
			if ((time_run - floor(time_run)) < av_off_ramp_rate)
			{
				veh_aim[veh_id] = 1;
				vehicle_color = RGB(0, 255, 255) + 255 * 16777216;
			}
			else
			{
				veh_aim[veh_id] = 2;
				vehicle_color = RGB(0, 180, 255) + 255 * 16777216;
			}
			suiji = random(0, 100);
			if (suiji < coop_rate * 100)
			{
				coop[veh_id] = 1;
			}
			else
			{
				coop[veh_id] = 2;
			}
		}



	
		if (active_lane_change != 0)
		{
			if ((current_link != last_link[veh_id]) && (current_link == 3 || current_link == 7) && (active_change_time[veh_id]<time_run))
			{
				target_lane[veh_id] = target_lane[veh_id] + 1;
			}
			else if ((current_link != last_link[veh_id]) && (current_link == 10003 || current_link == 10006) && (active_change_time[veh_id] < time_run))
			{
				target_lane[veh_id] = target_lane[veh_id] - 1;
			}
		}




		if ((current_link == 3 || current_link == 10002) && current_lane == 1)
		{
			//on ramp merging check
			if (veh_y_po < 720)
			{
				follow_turn();
				turn_judge();
				acc_angle();

			}
			else
			{
				follow_turn();
				turn_judge();
				if (change_left_condition == 1)
				{

					//active_lane_change = 1;
					//rel_target_lane = 1;
					//target_lane[veh_id] = 2;
					//desired_lane_angle = 0.1;
					//turn_judge();
					desired_acceleration = 3;
					acc_angle();
				}
				else
				{
					desired_acceleration = -4;
				}
			}
		}
		//else if ((current_link == 8 || current_link == 10005) && current_lane == 2)
		//else if ((((current_link == 7 || current_link == 10005) && current_lane == 2) || ((current_link == 6 || current_link == 10004) && current_lane == 1)) && veh_x_po > 4700)
		else if ((current_link == 7 || current_link == 10005) && current_lane == 2 && veh_y_po > 4420)
		{
			//diverging
			if (down_right_veh_distance > 6 && veh_aim[veh_id] == 1)
			{
				
				change_right_condition = 1;
				//vehicle_color = RGB(255, 188, 0) + 255 * 16777216;
				follow_turn();
				//turn_judge();
				right_future_fol_dist = 7;
				right_future_fol_acc = -3;
				acc_angle();
				//active_lane_change = -1;
				//rel_target_lane = -1;
				//target_lane[veh_id] = 1;
				//desired_lane_angle = -0.1;

			}
			else
			{
				follow_turn();
				turn_judge();
				acc_angle();
			}

		}
		else
		{
			follow_turn();
			turn_judge();
			acc_angle();
		}


		correction_angle();




		//for checking==========================================================================
		if (active_lane_change*desired_lane_angle < 0)
		{
			fout << time_run << "  " << veh_id << "  " << veh_aim[veh_id] << "  " << current_link << "----" << current_lane << "  " << active_lane_change << "<LC direction，angle>" << desired_lane_angle << "  " << change_right_condition << "  " << change_left_condition << "??????????999999" << endl;
		}


	

		
		last_link[veh_id] = current_link;


		veh_data[veh_id].push_back(current_speed);
		fout.precision(4);
		if (time_run == 300 && www == 0)
		{
			for (int i = 0; i < veh_data.size(); i++)
			{

				//fout << "630   ";
				if (veh_data[i].empty())
				{
					;
				}
				else
				{
					fout << veh_type << "   ";
					for (int j = 0; j < veh_data[i].size(); j++)
					{
						fout << veh_data[i][j] << "  ";
					}
					fout << endl;
				}
			}

			www = 1;
		}

	
		
      return 1;
    default :
      return 0;
    }
}

/*==========================================================================*/
/*  End of DriverModel.cpp                                                  */
/*==========================================================================*/