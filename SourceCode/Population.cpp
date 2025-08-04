#include"Population.h"
#include<numeric>
#include<deque>

void Population::FirstFitness()
{	// the first time to get the fitness;	1. get all contirbution of all indivs		2. get fitness
	for (int i = 0; i < the_indiv.size(); i++)the_indiv[i]->Get_Contri(*the_para);
	for (int i = 0; i < the_indiv.size(); i++)
	{
		int _count = 0;
		for (int j = 0; j < the_indiv.size(); j++)
		{
			int count = 0;
			if (j == i)continue;
			for (int x = 0; x < the_para->Dimension; x++)
				for (int y = 0; y < the_para->Dimension; y++)
					count += std::abs(the_indiv[i]->ContriMatrix[x][y] - the_indiv[j]->ContriMatrix[x][y]);
			_count += count;
		}
		maxcost = maxcost > the_indiv[i]->cost ? maxcost : the_indiv[i]->cost;
		mincost = mincost < the_indiv[i]->cost ? mincost : the_indiv[i]->cost;
		maxcontri = maxcontri > _count ? maxcontri : _count;
		mincontri = mincontri < _count ? mincontri : _count;
		the_indiv[i]->count_cont = _count;
	}
	for (int i = 0; i < the_indiv.size(); i++)
		the_indiv[i]->Fitness = std::max(1e-10,popalpha * (maxcost - the_indiv[i]->cost) / (maxcost - mincost + 1) + (1-popalpha) * (the_indiv[i]->count_cont - mincontri) / (maxcontri - mincontri + 1));
}
void Population::SonFitness(Individual& son)
{	// get son fitness
	son.Get_Contri(*the_para);
	int count = 0;
	for (int i = 0; i < the_indiv.size(); i++)
	{
		for (int x = 0; x < the_para->Dimension; x++)
		{
			for (int y = 0; y < the_para->Dimension; y++)
			{
				count += std::abs(the_indiv[i]->ContriMatrix[x][y] - son.ContriMatrix[x][y]);
			}
		}
	}
	count *= (the_indiv.size() - 1) / the_indiv.size();
	// son need to be scaled to (n - 1) / n 
	// for the reason that it cumulate n individual's abs value,
	// but each indiv in the pop only  cumulate (n-1)
	son.count_cont = count;
	son.Fitness = std::max(1e-10, popalpha * (maxcost - son.cost) / (maxcost - mincost + 1) + (1 - popalpha) * (count - mincontri) / (maxcontri - mincontri + 1));
}
void Population::Renewcount(Individual& s)
{	//update count value
	int _count = 0;
	for (int j = 0; j < the_indiv.size(); j++)
	{
		if (j == s.index)continue;
		for (int x = 0; x < the_para->Dimension; x++)
			for (int y = 0; y < the_para->Dimension; y++)
				_count += std::abs(s.ContriMatrix[x][y] - the_indiv[j]->ContriMatrix[x][y]);
	}
	s.count_cont = _count;
}

void Population::InsertIndiv(Individual& oldone, Individual& newone)
{	// use the newone indiv to replace the oldone
	newone.index = oldone.index;
	delete the_indiv[oldone.index];
	the_indiv[newone.index] = &newone;
	bool changed = false;
	for (int i = 0; i < the_indiv.size(); i++)
	{
		Renewcount(*the_indiv[i]);
		if (maxcontri < the_indiv[i]->count_cont)
		{
			maxcontri = the_indiv[i]->count_cont;
			changed = true;
		}
		else if (mincontri > the_indiv[i]->count_cont)
		{
			mincontri = the_indiv[i]->count_cont;
			changed = true;
		}
	}
	if (maxcost < newone.cost)
	{
		maxcost = newone.cost;
		changed = true;
	}
	else if (mincost > newone.cost)
	{
		mincost = newone.cost;
		changed = true;
	}
	if (changed)
		for (int j = 0; j < the_indiv.size(); j++)
			the_indiv[j]->Fitness = std::max(1e-10, popalpha * (maxcost-the_indiv[j]->cost) / (maxcost - mincost + 1) + (1 - popalpha) * (the_indiv[j]->count_cont-mincontri) / (maxcontri - mincontri + 1));
}