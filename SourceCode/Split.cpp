#include"Split.h"
int Split::doSplit(Individual& indiv, const GlobalParameter& para)
{	
	//constructor of split
	for (int i = 1; i < para.Num; i++)
	{
		cliSplit[i].demandD = para.d[indiv.Chrom[i - 1]];
		cliSplit[i].demandP = para.p[indiv.Chrom[i - 1]];
		cliSplit[i].PbiggerD = cliSplit[i].demandP >= cliSplit[i].demandD ? true : false;

		cliSplit[i].d0_x = para.Dist[0][para.Which[indiv.Chrom[i - 1]]];
		if (i < para.Num - 1) cliSplit[i].dnext = para.Dist[para.Which[indiv.Chrom[i - 1]]][para.Which[indiv.Chrom[i]]];
		else cliSplit[i].dnext = -1e30;
		sumDemandD[i] = sumDemandD[i - 1] + cliSplit[i].demandD;
		sumDemandP[i] = sumDemandP[i - 1] + cliSplit[i].demandP;
		sumDistance[i] = sumDistance[i - 1] + cliSplit[i - 1].dnext;
	}

	std::vector<double>(para.Num, 1e30).swap(potential);
	for (int i = 0; i < para.Num; i++)
	{
		potential[i] = 1e30;
	}
	potential[0] = 0;


	int i = 0;
	int flag = i;
	for (int j = 1; j < para.Num; j++) {
		i = flag;
		while (i < j) {
			double dist = potential[i] + cliSplit[i + 1].d0_x+ cliSplit[j].d0_x + sumDistance[j] - sumDistance[i + 1];
			if (dist < potential[j]-para.Precision) {
				potential[j] = dist;
				pred[j] = i;
			}
			i = i + 1;
		}

		i = flag;
		while (i < j + 1&& j+1< para.Num) {
			
			double load = sumDemandD[j + 1] - sumDemandD[i];
			while (load <= para.Capacity + para.Precision && i<j+1) {
				i = i + 1;
				load = sumDemandD[j + 1] - sumDemandD[i] + sumDemandP[i] - sumDemandP[flag];
			}
			if (load > para.Capacity + para.Precision) {
				flag = flag + 1;
				i = flag;
			}
			else break;
		}

	}

	int count = 0;
	for (int i = para.Num - 1; i != 0; i = pred[i], count++);

	indiv.cost = potential[para.Num - 1];

	std::vector<std::vector<int > >(count, std::vector<int >()).swap(indiv.Route);
	int end = para.Num - 1;
	for (int i = count - 1; i >= 0; i--)
	{
		int begin = pred[end];
		for (int j = begin; j < end; j++)
		{
			indiv.Route[i].push_back(indiv.Chrom[j]);

		}
		end = begin;
	}

	return count <= para.Vehicle;
}


void Split::run(Individual& indiv, const GlobalParameter& para)
{
	if (!doSplit(indiv, para))indiv.need_elimination = true;
}

Split::Split(GlobalParameter& para) :the_para(&para)
{
	//constructor: initialize the structure of split algorithm
	cliSplit = std::vector <ClientSplit>(para.Num);
	sumDistance = std::vector<double>(para.Num, 0);
	sumDemandD = std::vector<double>(para.Num, 0);
	sumDemandP = std::vector<double>(para.Num, 0);
	potential = std::vector < double >(para.Num, 1e30);
	pred = std::vector <int>(para.Num, 0);
};
