#include"Genetic.h"
#include<algorithm>
#include<deque>
#include<fstream>
void Genetic::ToNode(GlobalParameter& para)
{	// combine the single demand to pair
	para.time_begin = clock();
	para.time_first_better = para.time_begin;
	Node_Vec.clear();
	int i = 0, j = 0, whichpoint = 0, index = 0;		//index record the index of which pair
	while (1)
	{
		std::deque<double> d_vec;
		std::deque<double> p_vec;

		//1.put all the d and p demand of 'whichpoint' point in the vector
		while (para.d_orig[i][0] == whichpoint)
		{// put if they are from the same point
			if (para.d_orig[i][1] == 0)
			{
				i++;
				break;
			}
			//didx_vec.push_back(i);
			d_vec.push_back(para.d_orig[i++][1]);
		}
		while (para.p_orig[j][0] == whichpoint)
		{
			if (para.p_orig[j][1] == 0)
			{
				j++;
				break;
			}
			p_vec.push_back(para.p_orig[j++][1]);
		}
		//2.combine pairs
		double accum_d = 0, accum_p = 0;
		for (auto it = d_vec.begin(); it != d_vec.end(); it++)
		{
			accum_d += *it;
		}
		for (auto it = p_vec.begin(); it != p_vec.end(); it++)
		{
			accum_p += *it;
		}
		if (accum_d >= accum_p)
		{//Delivery is the main demand
			if (d_vec.size() == 0)
			{	
				if (whichpoint == 0)
				{
					para.d[index] = 0, para.p[index] = 0;
					Node_Vec.push_back(Node(index++, 0, whichpoint++));
					Node_Vec.back().demand_D = 0, Node_Vec.back().demand_P = 0;
					Node_Vec.back().isDepot = true;
				}
				continue;
			}
			while (d_vec.size() > 0 || p_vec.size() > 0)
			{
				int d_size = (int)d_vec.size(), p_size = (int)p_vec.size();
				if (d_size == 0)para.d[index] = 0;
				else
				{
					para.d[index] = d_vec.front();
					d_vec.pop_front();
				}
				if (p_size == 0)para.p[index] = 0;
				else
				{
					if (p_vec.front() > para.d[index]) {
						d_vec.push_front(para.d[index]);
						para.p[index] = p_vec.front();
						para.d[index] = 0;
						p_vec.pop_front();
					}
					else
					{
						while (p_vec.size() > 0 && para.p[index] + p_vec.front() <= para.d[index])
						{
							para.p[index] += p_vec.front();
							p_vec.pop_front();
						}
					}
				}
				Node_Vec.push_back(Node(index, 0, whichpoint));
				para.Which[index] = whichpoint;
				Node_Vec.back().demand_D = para.d[index], Node_Vec.back().demand_P = para.p[index];
				Node_Vec.back().LoadDelta = para.p[index] - para.d[index];
				if (Node_Vec.back().demand_D < Node_Vec.back().demand_P)Node_Vec.back().PbiggerD = true;
				else Node_Vec.back().PbiggerD = false;
				index++;
			}
		}
		else
		{//P is the main demand
			if (p_vec.size() == 0)
			{	
				if (whichpoint == 0)
				{
					para.d[index] = 0, para.p[index] = 0;
					Node_Vec.push_back(Node(index++, 0, whichpoint++));
					Node_Vec.back().demand_D = 0, Node_Vec.back().demand_P = 0;
					Node_Vec.back().isDepot = true;						
				}
				continue;
			}
			while (d_vec.size() > 0 || p_vec.size() > 0)
			{
				std::vector<int>tmp;
				int d_size = (int)d_vec.size(), p_size = (int)p_vec.size();
				if (p_size == 0)para.p[index] = 0;
				else
				{
					para.p[index] = p_vec.front();
					p_vec.pop_front();
				}
				if (d_size == 0)para.d[index] = 0;
				else
				{
					if (d_vec.front() > para.p[index]) {
						p_vec.push_front(para.p[index]);
						para.d[index] = d_vec.front();
						para.p[index] = 0;
						d_vec.pop_front();
					}
					else
					{
						while (d_vec.size() > 0 && para.d[index] + d_vec.front() <= para.p[index])
						{
							para.d[index] += d_vec.front();
							d_vec.pop_front();
						}
					}
					
				}
				Node_Vec.push_back(Node(index, 0, whichpoint));
				para.Which[index] = whichpoint;
				Node_Vec.back().demand_D = para.d[index], Node_Vec.back().demand_P = para.p[index];
				Node_Vec.back().LoadDelta = para.p[index] - para.d[index];
				if (Node_Vec.back().demand_D < Node_Vec.back().demand_P)Node_Vec.back().PbiggerD = true;
				else Node_Vec.back().PbiggerD = false;
				index++;
			}
		}
		whichpoint++;
		if (para.d_orig[i][0] == 0 && para.p_orig[j][0] == 0)
		{
			para.Num = index;
			return;
		}
	}
}
void Genetic::NoNode(GlobalParameter& para) {
	para.time_begin = clock();
	para.time_first_better = para.time_begin;
	Node_Vec.clear();
	int i = 0, j = 0, whichpoint = 0, index = 0;		
	while (1)
	{
		std::deque<double> d_vec;
		std::deque<double> p_vec;
		while (para.d_orig[i][0] == whichpoint)
		{
			if (para.d_orig[i][1] == 0)
			{
				i++;
				break;
			}
			d_vec.push_back(para.d_orig[i++][1]);
		}
		while (para.p_orig[j][0] == whichpoint)
		{
			if (para.p_orig[j][1] == 0)
			{
				j++;
				break;
			}
			//pidx_vec.push_back(i);
			p_vec.push_back(para.p_orig[j++][1]);
		}
		//2.combine couple

		if (d_vec.size() == 0)
		{	
			if (whichpoint == 0)
			{
				para.d[index] = 0, para.p[index] = 0;
				Node_Vec.push_back(Node(index++, 0, whichpoint++));
				Node_Vec.back().demand_D = 0, Node_Vec.back().demand_P = 0;
				Node_Vec.back().isDepot = true;
			}
			continue;
		}
		while (d_vec.size() > 0)
		{
			para.d[index] = d_vec.front();
			d_vec.pop_front();
			Node_Vec.push_back(Node(index, 0, whichpoint));
			para.Which[index] = whichpoint;
			Node_Vec.back().demand_D = para.d[index], Node_Vec.back().demand_P = para.p[index];
			Node_Vec.back().LoadDelta = para.p[index] - para.d[index];
			if (Node_Vec.back().demand_D < Node_Vec.back().demand_P)Node_Vec.back().PbiggerD = true;
			else Node_Vec.back().PbiggerD = false;
			index++;
		}
		while (p_vec.size() > 0)
		{
			para.p[index] = p_vec.front();
			p_vec.pop_front();
			Node_Vec.push_back(Node(index, 0, whichpoint));
			para.Which[index] = whichpoint;
			Node_Vec.back().demand_D = para.d[index], Node_Vec.back().demand_P = para.p[index];
			Node_Vec.back().LoadDelta = para.p[index] - para.d[index];
			if (Node_Vec.back().demand_D < Node_Vec.back().demand_P)Node_Vec.back().PbiggerD = true;
			else Node_Vec.back().PbiggerD = false;
			index++;
		}
		whichpoint++;
		if (para.d_orig[i][0] == 0 && para.p_orig[j][0] == 0)
		{
			para.Num = index;
			return;
		}
	}
}
void Genetic::GetNodeInformFromRoute(const Individual& indiv)
{	// get the algorithm auxiliary information from Route
	for (int i = 0; i < indiv.Route.size(); i++)
	{
		int routesize = indiv.Route[i].size();
		if (routesize > 0)
		{
			//Node* zero = new Node(0, 0, 0);
			Node*& zero = Zeros[i];
			zero->isDepot = true;
			double c_d = 0, c_p = 0;
			c_d += Node_Vec[indiv.Route[i][0]].demand_D, c_p += Node_Vec[indiv.Route[i][0]].demand_P;
			Node_Vec[indiv.Route[i][0]].cumulate_D = c_d, Node_Vec[indiv.Route[i][0]].cumulate_P = c_p;
			Node_Vec[indiv.Route[i][0]].pred = zero;
			zero->next = &Node_Vec[indiv.Route[i][0]];
			Node_Vec[indiv.Route[i][0]].position = 0;
			if (routesize == 1)
			{
				Node_Vec[indiv.Route[i][0]].next = zero;
				zero->pred = &Node_Vec[indiv.Route[i][0]];
				continue;
			}
			else Node_Vec[indiv.Route[i][0]].next = &Node_Vec[indiv.Route[i][1]];
			for (int j = 1; j < routesize - 1; j++)
			{
				c_d += Node_Vec[indiv.Route[i][j]].demand_D, c_p += Node_Vec[indiv.Route[i][j]].demand_P;
				Node_Vec[indiv.Route[i][j]].cumulate_D = c_d, Node_Vec[indiv.Route[i][j]].cumulate_P = c_p;
				Node_Vec[indiv.Route[i][j]].pred = &Node_Vec[indiv.Route[i][j - 1]];
				Node_Vec[indiv.Route[i][j]].next = &Node_Vec[indiv.Route[i][j + 1]];
				Node_Vec[indiv.Route[i][j]].position = j;
				
			}
			c_d += Node_Vec[indiv.Route[i][routesize - 1]].demand_D, c_p += Node_Vec[indiv.Route[i][routesize - 1]].demand_P;
			Node_Vec[indiv.Route[i][routesize - 1]].cumulate_D = c_d, Node_Vec[indiv.Route[i][routesize - 1]].cumulate_P = c_p;
			Node_Vec[indiv.Route[i][routesize - 1]].pred = &Node_Vec[indiv.Route[i][routesize - 2]];
			Node_Vec[indiv.Route[i][routesize - 1]].next = zero;
			zero->pred = &Node_Vec[indiv.Route[i][routesize - 1]];
			Node_Vec[indiv.Route[i][routesize - 1]].position = routesize - 1;
		}

	}
}
void Genetic::RenewChrom(Individual& indiv)
{	// get chrom from Routes
	indiv.Chrom.clear();
	for (int i = 0; i < indiv.Route.size(); i++)
		indiv.Chrom.insert(indiv.Chrom.end(), indiv.Route[i].begin(), indiv.Route[i].end());

	if (indiv.Chrom.size() >= the_para->Num)system("pause");
}
void Genetic::Output_oneFile(GlobalParameter& para) {
	// output in one file std::ios::app
	/*
	1.name: 
	2.Mycost: 
	3.Mysize: 
	4.Benchmark: 
	5.BestSize: 
	6.NoWorseTime: 
	7.BestTime: 
	8.TimeConstrain: 
	9.Routes: # 
		^ Route1: 10[4100,0]-13[250,0]-DEPOT-Maxload: 4350-Capacity: 4500 ^
		^ Route2: 10[4100,0]-13[250,0]-DEPOT-Maxload: 4350-Capacity: 4500 ^ #
	10.other info

	
	
	*/
	// output the the detailed information of the best solution  .   also check the feasibility
	std::ofstream out("ComputeResult.txt", std::ios::app);
	double mincost = 1e30;
	int minrt = 99;
	int mark = -1;
	clock_t mark_time = 99999999;
	for (int i = 0; i < para.Initial_Pop_Size; i++)
	{
		if ((Pop.the_indiv[i]->Route.size() < minrt) || (Pop.the_indiv[i]->Route.size() == minrt && Pop.the_indiv[i]->cost < mincost - 1.e-6))
		{
			mincost = Pop.the_indiv[i]->cost;
			minrt = Pop.the_indiv[i]->Route.size();
			mark = i;
			mark_time = Pop.the_indiv[i]->time;
			continue;
		}
		if ((Pop.the_indiv[i]->Route.size() == minrt) && (abs(Pop.the_indiv[i]->cost - mincost) < 1e-6) && Pop.the_indiv[i]->time < mark_time)
		{
			mincost = Pop.the_indiv[i]->cost;
			minrt = Pop.the_indiv[i]->Route.size();
			mark = i;
			mark_time = Pop.the_indiv[i]->time;
		}
	}

	if (para.ifBetter < 2)
	{
		if ((Pop.the_indiv[mark]->Route.size() == para.Vehicle && (Pop.the_indiv[mark]->cost - para.Best) / para.Best < -0.00001)
			|| (Pop.the_indiv[mark]->Route.size() < para.Vehicle))para.ifBetter = 2;
		if (para.ifBetter == 0)
		{
			if (Pop.the_indiv[mark]->Route.size() == para.Vehicle && abs(Pop.the_indiv[mark]->cost - para.Best) / para.Best < 0.00001)
				para.ifBetter = 1;
		}
	}

	GetNodeInformFromRoute(*Pop.the_indiv[mark]);
	double abc1 = 0, abc2 = 0;
	for (int i = 0; i < Pop.the_indiv[mark]->Route.size(); i++)
	{
		for (Node* mynode = &Node_Vec[Pop.the_indiv[mark]->Route[i][0]]; !mynode->isDepot; mynode = mynode->next)
		{
			abc1 += mynode->demand_D;
			abc2 += mynode->demand_P;
		}
	}
	if (abs(abc1 - para.total_d) > para.Precision || abs(abc2 - para.total_p) > para.Precision)
	{
		std::cout << "Less order" << std::endl;
		system("pause");
	}
	out << para.File_Name
		<< " Mycost: " << Pop.the_indiv[mark]->cost<<"\t"
		<< " Mysize: " << Pop.the_indiv[mark]->Route.size() << "\t"
		<< " Benchmark: " << para.Best << "\t"
		<< " BestSize: " << para.Vehicle << "\t"
		<< " NoWorseTime: " << ((double)para.time_first_better - (double)para.time_begin) / (double)CLOCKS_PER_SEC << "\t"
		<< " BestTime: " << ((double)Pop.the_indiv[mark]->time - (double)para.time_begin) / (double)CLOCKS_PER_SEC << "\t"
		<< " TimeConstrain: " << para.Time << "\t"
		<< " Routes: # ";

	Node* mynode;
	for (int i = 0; i < Pop.the_indiv[mark]->Route.size(); i++)
	{
		Node_Vec[Pop.the_indiv[mark]->Route[i].back()].next = &Node_Vec[0];
		double load = Node_Vec[Pop.the_indiv[mark]->Route[i].back()].cumulate_D;
		double max = load;
		int thispoint = 0;
		double thisload_d = 0;
		double thisload_p = 0;
		out << "^ Route" << i + 1 << ": ";
		for (mynode = &Node_Vec[Pop.the_indiv[mark]->Route[i][0]], thispoint = mynode->whichpoint;; mynode = mynode->next)
		{
			load += mynode->LoadDelta;
			max = max >= load ? max : load;
			if (mynode->whichpoint != thispoint)
			{
				out << thispoint << "[" << thisload_d << "," << thisload_p << "]" << "-";
				thispoint = mynode->whichpoint;
				if (thispoint == 0)break;
				thisload_d = mynode->demand_D, thisload_p = mynode->demand_P;
				continue;
			}
			thisload_d += mynode->demand_D;
			thisload_p += mynode->demand_P;
		}

		if (max > para.Capacity + para.Precision)
		{
			std::cout << "out of capacity: " << max << "	" << para.Capacity << std::endl;
			system("pause");
		}
		//out << thispoint << "[ " << thisload_d << " , " << thisload_p << " ]" << std::endl;
		out << "DEPOT"
			<< " Maxload: " << max
			<< " Capacity: " << para.Capacity
			<< " ^ " << "\t";
	}
	out << "#";
	out << " mu: " << para.mu
		<< " pi: " << para.DisturbTimes
		<< " alpha: " << para.alpha_pop;
	out << std::endl;

	double length = 0;
	for (int i = 0; i < Pop.the_indiv[mark]->Route.size(); i++)
	{
		length += para.Dist[Node_Vec[Pop.the_indiv[mark]->Route[i][0]].whichpoint][0];
		for (int j = 0; j < Pop.the_indiv[mark]->Route[i].size(); j++)
		{
			length += para.Dist[Node_Vec[Pop.the_indiv[mark]->Route[i][j]].whichpoint][Node_Vec[Pop.the_indiv[mark]->Route[i][j]].next->whichpoint];
		}
	}
	if (abs(length - Pop.the_indiv[mark]->cost) > para.Precision)
	{
		std::cout << "Wrong length" << std::endl;
		system("pause");
	}


	// delete all the zero  points
	for (int i = 0; i < Zeros.size(); i++) {
		if (Zeros[i] != nullptr) {
			delete Zeros[i];
			Zeros[i] = nullptr;
		}

	}

}
void Genetic::Initialization(Population& pop, GlobalParameter& para)
{	// initialize the population
	// can be larger
	for (int i = 0; i < 200; i++) {
		Node* zero = new Node(0,0,0);
		zero->isDepot = true;
		Zeros.emplace_back(zero);
	}

	
	double cost = 1e10;
	int rt = 1000;
	int index = -1;
	pop = Population(para.Initial_Pop_Size, para);
	std::vector<std::pair<double, int>>sequence(para.Num - 1, std::pair<double, int>(0., -1));
	for (int i = 1; i < para.Num; i++)
	{
		sequence[i - 1].first = -Node_Vec[i].LoadDelta;
		sequence[i - 1].second = i;
	}
	// sort according the -load_delta of each pair
	std::sort(sequence.begin(), sequence.end());
	std::vector<std::vector<int>>ROUTE(1, std::vector<int>());
	std::vector<double>start(1, 0.);
	for (int j = 0; j < sequence.size(); j++)
	{
		bool flag = false;
		for (int the_route = 0; the_route < ROUTE.size() && !flag; the_route++)
		{
			if (start[the_route] + Node_Vec[sequence[j].second].demand_D >= the_para->Capacity + the_para->Precision)continue;
			ROUTE[the_route].push_back(sequence[j].second);
			start[the_route] += Node_Vec[sequence[j].second].demand_D;
			double current_load = 0;
			current_load = start[the_route];
			bool the_flag = false;
			for (int k = 0; k < ROUTE[the_route].size(); k++)
			{
				current_load += Node_Vec[ROUTE[the_route][k]].LoadDelta;
				if (current_load >= para.Precision + para.Capacity)
				{
					ROUTE[the_route].pop_back();
					start[the_route] -= Node_Vec[sequence[j].second].demand_D;
					the_flag = true;
					break;
				}
			}
			if (the_flag)continue;
			flag = true;
		}
		if (!flag)
		{
			ROUTE.push_back(std::vector<int>());
			ROUTE.back().push_back(sequence[j].second);
			start.push_back(Node_Vec[sequence[j].second].demand_D);
		}
	}
	for (int this_indiv = 0; this_indiv < para.Initial_Pop_Size; this_indiv++)
	{
		Individual* indiv = new Individual(para);
		for (int i = 0, count = 0; i < ROUTE.size(); i++)
		{
			for (int j = 0; j < ROUTE[i].size(); j++)
			{
				indiv->Chrom[count++] = ROUTE[i][j];
			}
		}
		// disturb para.DisturbTimes times to get different initial indivials
		for (int i = 0; i < para.DisturbTimes; i++)
		{
			int x = rand() % indiv->Chrom.size();
			int y = rand() % indiv->Chrom.size();
			while (x == y)y = rand() % indiv->Chrom.size();
			std::swap(indiv->Chrom[x], indiv->Chrom[y]);
		}


		// use split alg. to get route information
		Sp.run(*indiv, para);
		indiv->time = clock();
		indiv->index = this_indiv;
		pop.the_indiv[this_indiv] = indiv;
		GetNodeInformFromRoute(*indiv);
		// education each individual
		Ls.run(*indiv, Node_Vec, para);
		RenewChrom(*indiv);
		if (indiv->Route.size() < rt || indiv->Route.size() == rt && indiv->cost < cost - para.Precision)
		{
			rt = indiv->Route.size();
			cost = indiv->cost;
			index = this_indiv;
		}
		// delete indiv;
		indiv = nullptr;
	}
	best_index = index;
}
void Genetic::SelectParents(GlobalParameter& para)
{	// randonly select two parents
	int a = rand() % Pop.the_indiv.size();
	p1 = Pop.the_indiv[a];
	int b = rand() % Pop.the_indiv.size();
	while(b==a)b= rand() % Pop.the_indiv.size();
	p2 = Pop.the_indiv[b];
}
void Genetic::CrossOver(Individual& son, Individual* parent1, Individual* parent2)
{	// crossover to get one offspring
	int a = rand() % (the_para->Num - 1);
	int b= rand() % (the_para->Num - 1);
	while (b == a)b = rand() % (the_para->Num - 1);
	if (b < a)std::swap(a, b);
	std::vector<bool>mark(the_para->Num - 1, true);
	int i = 0, count = a;
	for (i = a; i < b; i++)
	{
		int tmp = parent1->Chrom[i];
		son.Chrom[count++] = tmp;
		mark[tmp - 1] = false;
	}
	for (i = b; i < the_para->Num-1; i++)
	{
		int tmp = parent2->Chrom[i];
		if (mark[tmp - 1])
		{
			son.Chrom[count++] = tmp;
			mark[tmp - 1] = false;
		}
	}
	for (i = 0; i < a; i++)
	{
		int tmp = parent2->Chrom[i];
		count %= the_para->Num - 1;
		if (mark[tmp - 1])
		{
			son.Chrom[count++] = tmp;
			mark[tmp - 1] = false;
		}
	}
	for (i = a; i < b; i++)
	{
		int tmp = parent2->Chrom[i];
		count %= the_para->Num - 1;
		if (mark[tmp - 1])
		{
			son.Chrom[count++] = tmp;
			mark[tmp - 1] = false;
		}
	}
	//5% chance to swap 3 points in chrom [variation]
	if (rand() % 20 == 0)
	{
		for (int i = 0; i < 3; i++)
		{
			int x = rand() % son.Chrom.size();
			int y = rand() % son.Chrom.size();
			while (x == y)y = rand() % son.Chrom.size();
			std::swap(son.Chrom[x], son.Chrom[y]);
		}
	}
	Sp.run(son, *the_para);
	son.time = clock();	// update the time 
	GetNodeInformFromRoute(son);
}
void Genetic::PopManagement()
{	// management to population
	bool changed = false;
	int index = -1;
	double fit = 1e10;
	int rt = 0;
	// obtain the worse    indiv in the population
	for (int i = 0; i < Pop.the_indiv.size(); i++)
	{
		if (Pop.the_indiv[i]->Route.size() > rt || ((Pop.the_indiv[i]->Route.size() == rt) && (Pop.the_indiv[i]->Fitness < fit - the_para->Precision)))
		{
			index = i;
			fit = Pop.the_indiv[i]->Fitness;
			rt = Pop.the_indiv[i]->Route.size();
		}
	}
	if (Son->cost < Pop.the_indiv[best_index]->cost - the_para->Precision && Son->Route.size() <= Pop.the_indiv[best_index]->Route.size())
	{	
		NoChangeBest = 0;
		Pop.InsertIndiv(*Pop.the_indiv[index], *Son);
		best_index = index;
		changed = true;
	}
	else
	{
		if ((Son->Route.size() < Pop.the_indiv[index]->Route.size())
			|| ((Son->Fitness > Pop.the_indiv[index]->Fitness + the_para->Precision) && (Son->Route.size() == Pop.the_indiv[index]->Route.size())))
		{
			NoChangeBest++;
			Pop.InsertIndiv(*Pop.the_indiv[index], *Son);
			changed = true;
		}
	}
	if (!changed)
	{
		NoChangeBest++;
		delete Son;
	}

}
Genetic::Genetic(GlobalParameter& para):the_para(&para)
{
	// combine the single demand to pairs
	ToNode(para);
	//NoNode(para);
	// set Initial_Pop_Size according to Num (overall number of the pairs)
	para.Initial_Pop_Size = std::max<int>(2, (int)(para.mu * sqrt(para.Num)));

	Sp = Split(para);

	Ls = LocalSearch(Node_Vec, para);
	// initialization of the population
	Initialization(Pop, para);
	Pop.FirstFitness();
	while ((((double)clock() - (double)para.time_begin) / (double)CLOCKS_PER_SEC < para.Time) && (NoChangeBest <= para.MaxNoChangeBest))
	{	// conditions for continue the overall alg.
		// creat Son
		Son = new Individual(para);
		// choose parents
		SelectParents(para);
		// crossover the get son data
		CrossOver(*Son, p1, p2);
		// education Son
		Ls.run(*Son, Node_Vec, para);
		// population management 
		RenewChrom(*Son);
		Pop.SonFitness(*Son);
		PopManagement();
	}

	double mincost = 1e30;
	int minrt = 99;
	// get the best solution's  f and K 
	for (int i = 0; i < para.Initial_Pop_Size; i++)
	{
		if ((Pop.the_indiv[i]->Route.size() < minrt) || (Pop.the_indiv[i]->Route.size() == minrt && Pop.the_indiv[i]->cost < mincost))
		{
			mincost = Pop.the_indiv[i]->cost;
			minrt = Pop.the_indiv[i]->Route.size();
		}
	}
	std::cout << para.File_Name << '\t';
	std::cout << "My result: " << mincost << '\t' << "Best result: " << para.Best << '\t';
	std::cout << "My size: " << minrt << '\t' << "Best size: " << para.Vehicle << std::endl;
	Output_oneFile(para);
}
