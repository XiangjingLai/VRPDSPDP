#pragma once
#include"Global_Parameter.h"
#include"Individual.h"
#include<vector>
struct ClientSplit
{
	double demandD;
	double demandP;
	double d0_x;
	double dnext;
	bool PbiggerD;
	ClientSplit() : demandD(0.), demandP(0.), d0_x(0.), dnext(0.), PbiggerD(false) {};
};
class Split
{
public:
	GlobalParameter* the_para;
	std::vector < ClientSplit > cliSplit;
	std::vector < double > potential;  // Potential vector
	std::vector < int > pred;  // Indice of the predecessor in an optimal path
	std::vector < double > sumDistance; // sumDistance[i] for i > 1 contains the sum of distances : sum_{k=1}^{i-1} d_{k,k+1}
	std::vector < double > sumDemandD;
	std::vector < double > sumDemandP;

	int max_vehicle;


	void run(Individual& indiv, const GlobalParameter& para);
	int doSplit(Individual& indiv, const GlobalParameter& para);	////if K of this solution is bigger than para.Vehicle, do Limited Fleet Split

	Split(GlobalParameter& para);
	Split() = default;
};
