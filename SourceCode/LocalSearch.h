#pragma once
#include"Global_Parameter.h"
#include"Individual.h"
#include"Split.h"
class Nearby
{
public:
	std::vector<int >neighbor;	//store the index of the nearest couple 
	int whichPoint;		//mark the index of the point
	Nearby(int i, const GlobalParameter& para);

	inline::std::vector<int> get_near(const GlobalParameter& para);
};

class TwoRoute;
class Node
{	//used in normal operator in local search
public:
	int index;		//index in couple
	int whichpoint;
	bool isDepot = false;
	bool PbiggerD = false;
	double demand_D = 0;
	double demand_P = 0;
	double LoadDelta = 0;//p-d
	//
	
	int position = -1 ;	//index in route
	//int whichroute;
	TwoRoute* route=nullptr;	//point to the trace 
	
	Node* next=nullptr;
	Node* pred=nullptr;
	

	double cumulate_D =0 ;
	double cumulate_P =0 ;
	int Record_Last_Test = -1;		


	Node(int idx, int pos, int wp) :index(idx), position(pos), whichpoint(wp) { };
	Node()
	{
		index = -1, whichpoint = -1;
	}

};
class TwoRoute
{
public:
	int index = -1;	//index of all the route 
	Node* DepotNode = NULL;		//point to depot of node
	double MaxLoad = 0;
	int Record_Last_Change = 0;	//every time operating this trace, it changes
	TwoRoute(int idx, Node* first) :index(idx)
	{
		DepotNode = first->pred;
	};
	TwoRoute() = default;
};
class LocalSearch
{
public:
	std::vector<Nearby> near;	//vector of nearby    Dimention:Num (including depot)
	std::vector<std::vector<Node* > >NodeGroup;		//Dimention*each_numb	dimention1:whichpoint i		 	dimention2:point to all the Nodes belong to point i
	
	Individual* the_indiv;
	int operate_num = 0;
	int count_size = 0;
	std::vector<int>U_vector;//denotes the sequence of U  which needs to be checked

	std::vector<TwoRoute*>RouteVector;		//record depot information of each route
	std::vector<std::vector<int>>valid;		//represent all points  which PbiggerD is ture
	double capacity_precision;

	int UIndex, VIndex, XIndex, YIndex;
	int UWhich, VWhich, XWhich, YWhich;
	Node* NodeX;
	Node* NodeY; 
	Node* NodeU; 
	Node* NodeV;

	Node* Xhead;//represent the first of Node which belongs to the same Point with NodeX's. following are same use
	Node* Xtail;
	Node* Yhead;
	Node* Ytail;
	Node* Uhead;
	Node* Utail;
	Node* Vhead;
	Node* Vtail;

	double UDelete=0., VDelete=0., XDelete=0., YDelete=0.;	//<0
	bool SamePoint = false;
	bool SameRoute = false;
	bool markpointifchanged = false;
	Node* tmpnode;
	void updateRoute(TwoRoute* Route, std::vector<Node>& Node_Vec, const GlobalParameter& para);

	bool MOVE1(std::vector<Node>& Node_Vec, const GlobalParameter& para);//insert_ u after v			
	bool MOVE2(std::vector<Node>& Node_Vec, const GlobalParameter& para);//swap_ u and v				  
	bool MOVE4(std::vector<Node>& Node_Vec, const GlobalParameter& para);//2opt_ intra				
	bool MOVE5(std::vector<Node>& Node_Vec, const GlobalParameter& para);//2opt*_1 inter			
	bool MOVE6(std::vector<Node>& Node_Vec, const GlobalParameter& para);//2opt*_2 inter				
	bool elimination(std::vector<Node>& Node_Vec, const GlobalParameter& para);
	bool GetNodeV();

	void GetRouteInformFromNode(Individual& indiv, std::vector<Node>& Node_Vec, const GlobalParameter& para);

	double delta;		//	store change of cost     delta<0 denotes improvement

	void Swap(Node* u, Node* v);
	void Swap(Node* head1, Node* tail1, Node* head2, Node* tail2);
	void Insert(Node* u, Node* v);
	void Insert(Node* head1, Node* tail1, Node* tail2);

	void run(Individual& indiv, std::vector<Node>& Node_Vec, GlobalParameter& para);		//main running function
	LocalSearch(std::vector<Node>& Node_Vec, const GlobalParameter& para);
	LocalSearch(){	};
};