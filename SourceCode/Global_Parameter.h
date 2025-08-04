#pragma once
#include<algorithm>
#include<iostream>
#include<fstream>
#include<vector>
#include<string>
#include<ctime>
  
class GlobalParameter
{
public:
	//include all the global parameter

	clock_t time_begin;
	clock_t time_first_better;

	bool better_done = false;


	const double Precision = 1e-5;
	std::string File_Name;
	double d_orig[1000][2] = {  };		//whichpoint+delivery
	double p_orig[1000][2] = {  };

	double total_d;
	double total_p;

	double d[1000] = {  };			//pair's delivery demand
	double p[1000] = {  };			//pair's pickup demand
	int Which[1000] = {  };		//record which point

	double Dist[110][110];
	double Position[110][2];	//	x y location information 
	int Vehicle;
	double Best;
	double Time;
	int Dimension;				//number of points
	int Num;					//number of pairs,including depot pair
	double Capacity;

	int near_numb;		// number of the neareast point. never use this



	int ifBetter = 0;		//	0-> lose		 1-> draw		2-> win
	int count_win = 0;
	int count_draw = 0;
	int count_lose = 0;


	std::string Node_mode;
	const int MaxNoChangeBest = 3e4;	//	default
	double mu = 0.4;					// the coefficient in the "mu"  of the article
	int DisturbTimes = 5;				// "pi"  of the article
	double alpha_pop = 0.6;				// "alpha" of the article
	int Initial_Pop_Size = 10;			//

  };
