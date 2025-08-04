#pragma once
#include"Global_Parameter.h"
#include"Individual.h"
#include"Split.h"

class Population
{
public:
	double mincost = 1e30;
	double maxcost = 0;
	double maxcont = 0;
	double mincont = 1e30;
	int maxcontri = 0;
	int mincontri = 9999999;
	double popalpha = 0.6;					// Coefficient in  population update, set to 0.6
	std::vector<Individual*> the_indiv;
	GlobalParameter* the_para;


	void FirstFitness();
	void SonFitness(Individual& son);
	void InsertIndiv(Individual& oldone, Individual& newone);
	void Renewcount(Individual& s);
	Population(int numb,GlobalParameter& para)
	{	// Initialize data
		the_indiv = std::vector<Individual*>(numb);
		mincost = 1e30;
		maxcost = 0;
		maxcont = 0;
		mincont = 1e30;
		the_para = &para;
		popalpha = para.alpha_pop;
	}
	Population() {};
};
