#include"Individual.h"
void Individual::Get_Contri(const GlobalParameter& para)
{	//Obtain the individual's own contri matrix value based on its route
	std::vector<std::vector<int>>(para.Dimension, std::vector<int>(para.Dimension, 0)).swap(ContriMatrix);
	for (int i = 0; i < Route.size(); i++)
	{
		ContriMatrix[0][para.Which[Route[i][0]]]++;
		for (int j = 0; j < Route[i].size()-1; j++)
		{
			ContriMatrix[para.Which[Route[i][j]]][para.Which[Route[i][j + 1]]]++;
		}
		ContriMatrix[para.Which[Route[i].back()]][0]++;
	}
	for (int i = 0; i < para.Dimension; i++)ContriMatrix[i][i] = 0;
}



Individual::Individual(const GlobalParameter& para)
{	//constructor of Individual
	Chrom = std::vector<int>(para.Num - 1);
	Route = std::vector<std::vector<int > >(para.Vehicle);
	need_elimination = false;
	time = clock();
	count_cont = 0;
}