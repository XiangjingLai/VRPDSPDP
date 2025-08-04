#ifndef   Individual_H       
#define   Individual_H       

#pragma once
#include"Global_Parameter.h"
#include<vector>
class Individual
{
public:
	int index;
	double cost;		//	target.   total cost 
	std::vector<int >Chrom;					//	Giant tour   dimention:Num-1
	std::vector<std::vector<int > >Route;	//	respectively display each vehicle according to Chrom
	clock_t time;							// store the best result time of this indiv
	bool need_elimination;					//	flag to judge whether or not to eliminate routes in local search
	double Fitness;							//	to evaluate the contribution of this individual
	int count_cont;
	std::vector<std::vector<int>>ContriMatrix;//	matrix records the contir of Chrom of this indiv
	void Get_Contri(const GlobalParameter& para);	// obtain contri matrix value
	Individual(const GlobalParameter&para);
	Individual() = default;
};
#endif