#pragma once
#include"Global_Parameter.h"
#include"Individual.h"
//#include"Split.h"
#include"Split.h"
#include"Population.h"
#include"LocalSearch.h"
class Genetic
{
public:
	Population Pop;						// population
	Split Sp;							// linear split algorithm
	LocalSearch Ls;						// local search to education
	Individual* p1;						// parent 1
	Individual* p2;						// parent 2
	Individual* Son;					// offspring
	GlobalParameter* the_para;			// parameter
	std::vector<Node>Node_Vec;			// vector to store pairs
	std::vector<Node*>Zeros;				// vector of depots  of  all the individuals  or the offspring

	int best_index = -1;

	int NoChangeBest = 1;				// the number of times in which the best solution has not been changed

	void ToNode(GlobalParameter& para);		// combine 
	void NoNode(GlobalParameter& para);		// nocombine
	void GetNodeInformFromRoute(const Individual& indiv);		
	void RenewChrom(Individual& Indiv);
	void Output_oneFile(GlobalParameter& para);


	void Initialization(Population& pop, GlobalParameter& para);
	void SelectParents(GlobalParameter& para);
	void CrossOver(Individual& son, Individual* parent1, Individual* parent2);
	void PopManagement();

	Genetic(GlobalParameter&para);		//  main running function
};