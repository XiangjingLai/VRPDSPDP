#include"Global_Parameter.h"
#include"Individual.h"
#include"Split.h"
#include"Population.h"
#include"LocalSearch.h"
#include"Genetic.h"
#include<iostream>
#include<ctime>
#include <cstring>

using namespace std;
 


bool Read_File(const string& file_name,GlobalParameter& para)
{	// read the instance information

	//set all the matrix to Zero
	memset(para.d_orig, 0, sizeof(para.d_orig));
	memset(para.p_orig, 0, sizeof(para.p_orig));
	memset(para.d, 0, sizeof(para.d));
	memset(para.p, 0, sizeof(para.p));
	memset(para.Which, 0, sizeof(para.Which));
	memset(para.Dist, 0, sizeof(para.Dist));
	memset(para.Position, 0, sizeof(para.Position));
	para.total_d = 0, para.total_p = 0;
	para.ifBetter = 0;
	
	ifstream Data(file_name+".txt", ios::out);
	if (!Data) {  
        std::cerr << "Can not open: "<<file_name+".txt" << std::endl;
        std::exit(EXIT_FAILURE);  // exit 
    }
	string tmp;
	while (tmp != "NAME:")Data >> tmp;
	Data >> para.File_Name;
	//std::cout << para.File_Name << "	";
	while (tmp != "COMMENT:")Data >> tmp; 
	Data >> para.Best;
	while (tmp != "VEHICLE:")Data >> tmp;
	Data >> para.Vehicle;
	while (tmp != "TIME:")Data >> tmp;
	Data >> para.Time;
	//para.Time /= 10;
	while (tmp != "DIMENSION:")Data >> tmp;
	Data >> para.Dimension;
	while (tmp != "CAPACITY:")Data >> tmp;
	Data >> para.Capacity;
	while (tmp != "DELIVERY_SECTION")Data >> tmp;
	Data >> tmp;
	for (int i=0;tmp != "PICKUP_SECTION"; Data >> tmp,i++)
	{// get delivery demands for each d order
		Data >> para.d_orig[i][0] >> para.d_orig[i][1];
		para.total_d += para.d_orig[i][1];
	}
	Data >> tmp;
	for (int i = 0; tmp != "NODE_SECTION"; Data >> tmp, i++)
	{// get pickup demands for each p order
		Data >> para.p_orig[i][0] >> para.p_orig[i][1];
		para.total_p += para.p_orig[i][1];
	}

	para.near_numb =	para.Dimension;
	Data >> tmp;
	if (tmp != "DISTANCE_SECTION")
	{// get position information for each node 
		para.Position[0][0] = atoi(tmp.c_str());
		Data >> para.Position[0][1];
		for (int i = 1; i < para.Dimension; i++)
		{
			Data >> para.Position[i][0] >> para.Position[i][1];
		}

		for (int i = 0; i < para.Dimension; i++)
		{
			for (int j = 0; j < para.Dimension; j++)
			{
				para.Dist[i][j] = std::sqrt(
					(para.Position[i][0] - para.Position[j][0]) * (para.Position[i][0] - para.Position[j][0]) +
					(para.Position[i][1] - para.Position[j][1]) * (para.Position[i][1] - para.Position[j][1])
				);
			}
		}
	}
	else 
	{	
		for (int i = 0; i < para.Dimension - 1; i++)
		{
			for (int j = i + 1; j < para.Dimension; j++)
			{
				Data >> para.Dist[i][j];
				para.Dist[j][i] = para.Dist[i][j];
			}
		}
	}
	while (tmp != "DEPOT_SECTION")Data >> tmp;
	Data >> tmp;
	if (tmp != "0")cout << "Depot is not Zero!" << endl;
	Data >> tmp;
	if (tmp != "EOF")cout << "More data after depot section" << endl;

	string pre = file_name.substr(0,3);
	if(pre == "set"){
		para.Time = para.Time > 60 ? 60 : para.Time;
	}

	return true;
}





void release(Genetic& run, GlobalParameter&para)
{	
	// release the memory
	para.better_done = false;
	for (int i = 0; i < para.Initial_Pop_Size; i++)
	{
		delete run.Pop.the_indiv[i];
		run.Pop.the_indiv[i] = nullptr;
	}
}


int main(int argc, char *argv[])
{
	
	string file_name = argv[1];
	int run_times = atoi(argv[2]);
	
	// INPUT: FileName   times   mu=0.4   pi=5   alpha=0.7 (default)
	
	for (int i = 0; i < run_times; i++) {
		GlobalParameter para;
		if (argc > 3)para.mu = stod(argv[3]);
		if (argc > 4)para.DisturbTimes = atoi(argv[4]);
		if (argc > 5)para.alpha_pop = stod(argv[5]);
		Read_File(file_name,para);
		srand(time(NULL));
		Genetic run(para);
		release(run,para);
	}
	
	
	return 0;
}
