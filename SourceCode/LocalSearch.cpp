#include"LocalSearch.h"
#include<functional>
#include <random>  
#include <algorithm> 
//basic operators of ls
std::vector<int>Nearby::get_near(const GlobalParameter& para)
{
	std::vector<std::pair<double, int > > tmp(para.Dimension);		//dist+index
	for (int i = 0; i < para.Dimension; i++)
	{
		tmp[i].first = para.Dist[whichPoint][i];
		tmp[i].second = i;
	}
	std::sort(tmp.begin(), tmp.end());
	std::vector<int>tmp1(para.near_numb);
	for (int i = 0; i < para.near_numb; i++)
	{
		tmp1[i] = tmp[i].second;
	}
	return tmp1;
}
Nearby::Nearby(int i, const GlobalParameter& para)
{
	whichPoint = i;
	neighbor = get_near(para);
}

void LocalSearch::Swap(Node* u, Node* v)
{	// swap pair-u and pair-v
	Node* Upred = u->pred;
	Node* Unext = u->next;
	Node* Vpred = v->pred;
	Node* Vnext = v->next;
	TwoRoute* URoute = u->route;
	TwoRoute* VRoute = v->route;

	Upred->next = v;
	Unext->pred = v;
	Vpred->next = u;
	Vnext->pred = u;

	u->pred = Vpred;
	u->next = Vnext;
	v->pred = Upred;
	v->next = Unext;

	u->route = VRoute;
	v->route = URoute;
}
void LocalSearch::Swap(Node* head1, Node* tail1, Node* head2, Node* tail2)
{	// swap sub-sequence (head1, tail1) and (head2,tail2)
	TwoRoute* route1 = head1->route;
	TwoRoute* route2 = head2->route;
	Node* head1pred = head1->pred;
	Node* tail1next = tail1->next;

	head1->pred = head2->pred;
	tail1->next = tail2->next;
	head2->pred->next = head1;
	tail2->next->pred = tail1;
	head2->pred = head1pred;
	tail2->next = tail1next;
	head1pred->next = head2;
	tail1next->pred = tail2;

	for (Node* mynode = head1; mynode->index != tail1->next->index; mynode = mynode->next)mynode->route = route2;
	for (Node* mynode = head2; mynode->index != tail2->next->index; mynode = mynode->next)mynode->route = route1;

}
void LocalSearch::Insert(Node* u, Node* v)
{
	u->pred->next = u->next;
	u->next->pred = u->pred;
	u->pred = v;
	u->next = v->next;
	v->next->pred = u;
	v->next = u;
	u->route = v->route;
}
void LocalSearch::Insert(Node* head1, Node* tail1, Node* tail2)
{
	head1->pred->next = tail1->next;
	tail1->next->pred = head1->pred;
	head1->pred = tail2;
	tail1->next = tail2->next;
	tail2->next->pred = tail1;
	tail2->next = head1;
	for (Node* mynode = head1; mynode->index != tail1->next->index; mynode = mynode->next)mynode->route = tail2->route;
}

void LocalSearch::run(Individual& indiv, std::vector<Node>& Node_Vec, GlobalParameter& para)
{	// main function of local search
	//inilitialization 
	operate_num = 0;
	count_size = 0;
	std::random_shuffle(U_vector.begin(), U_vector.end());
	the_indiv = &indiv;
	valid = std::vector<std::vector<int>>(indiv.Route.size(), std::vector<int>());
	RouteVector.clear();
	for (int i = 0; i < indiv.Route.size(); i++)
	{
		Node* mynode = &Node_Vec[indiv.Route[i][0]];
		TwoRoute* myroute = new TwoRoute(i, mynode);
		RouteVector.push_back(myroute);
		double load = Node_Vec[indiv.Route[i].back()].cumulate_D;
		myroute->MaxLoad = load;
		for (; !mynode->isDepot; mynode = mynode->next)
		{
			mynode->Record_Last_Test = -1;
			mynode->route = myroute;
			myroute->MaxLoad = myroute->MaxLoad > (load - mynode->cumulate_D + mynode->cumulate_P) ? myroute->MaxLoad : load - mynode->cumulate_D + mynode->cumulate_P;
		}
		Node_Vec[indiv.Route[i][0]].pred->route = myroute;
	}
	double mark_cost = the_indiv->cost;
	// three 'for' loops
	double bef = the_indiv->cost;
	for (int i = 0; i < para.Num - 1; i++)
	{
		// try to decrease the number of vehicles
		elimination(Node_Vec, para);

		// initialization of valid
		for (int j = 0; j < RouteVector.size(); j++)
		{
			valid[j].clear();
			if (RouteVector[j]->DepotNode->next->isDepot)continue;
			Node* mynode = RouteVector[j]->DepotNode->next;
			for (; !mynode->isDepot; mynode = mynode->next)
				if (mynode->PbiggerD)valid[j].push_back(mynode->index);
		}

		// get pair U
		NodeU = &Node_Vec[U_vector[i]];	//U cannot be depot, but V may be
		UWhich = NodeU->whichpoint, UIndex = NodeU->index; //URoute = NodeU->whichroute;
		int UTestedNumb = NodeU->Record_Last_Test;
		NodeU->Record_Last_Test = operate_num;
		for (int k = 0; k < near[UWhich].neighbor.size(); k++)
		{	
			markpointifchanged = true;
			tmpnode = &Node_Vec[0];
			for (int j = 0; j < NodeGroup[near[UWhich].neighbor[k]].size(); j++)
			{
				// get pair V
				NodeV = NodeGroup[near[UWhich].neighbor[k]][j];
				if (GetNodeV())continue;
				// if Record has no change, continue to accelerate 
				if (std::max(NodeU->route->Record_Last_Change, NodeV->route->Record_Last_Change) <= UTestedNumb)continue;

				// 5 local search operators 
				if (MOVE1(Node_Vec, para)){
					continue;
				}
				if (MOVE2(Node_Vec, para)){
					continue;	
				}
				if (SameRoute && MOVE4(Node_Vec, para)){
					continue;
				}
				if (!SameRoute && MOVE5(Node_Vec, para)){
					continue;
				}
				if (!SameRoute && MOVE6(Node_Vec, para)){
					continue;
				}
				markpointifchanged = false;
			}
		}

	}


	if (mark_cost != the_indiv->cost)
	{	//update time of this indiv
		the_indiv->time = clock();
		mark_cost = the_indiv->cost;
	}
	if (!para.better_done && ((the_indiv->cost - para.Best) / para.Best <= 0.00001 && !the_indiv->need_elimination))
	{
		para.time_first_better = clock();
		para.better_done = true;
	}
	GetRouteInformFromNode(indiv, Node_Vec, para);
  
	for (int i = 0; i < RouteVector.size(); i++)
	{
		delete RouteVector[i];
	}
}
bool LocalSearch::MOVE1(std::vector<Node>& Node_Vec, const GlobalParameter& para)
{//insert u's after v's 
	if (SameRoute && Uhead->pred->index == Vtail->index)return false;
	delta = -para.Dist[Uhead->pred->whichpoint][UWhich] - para.Dist[UWhich][Utail->next->whichpoint]
		- para.Dist[VWhich][Vtail->next->whichpoint]
		+ para.Dist[Uhead->pred->whichpoint][Utail->next->whichpoint]
		+ para.Dist[VWhich][UWhich] + para.Dist[UWhich][Vtail->next->whichpoint];
	if (SameRoute && Utail->next->index == Vhead->index)delta = -para.Dist[Uhead->pred->whichpoint][UWhich] - para.Dist[VWhich][Vtail->next->whichpoint]
		+ para.Dist[Uhead->pred->whichpoint][VWhich] + para.Dist[UWhich][Vtail->next->whichpoint];
	//if (delta > -para.Precision && !(Utail->next->isDepot && Uhead->pred->isDepot))return false;
	if (delta > -para.Precision || (Utail->next->isDepot && Uhead->pred->isDepot))return false;
	if (SameRoute)
	{	//need to check the load between head and tail
		double test = 0;
		Node* mynode;
		if (NodeU->position < NodeV->position)
		{
			test = NodeU->route->DepotNode->pred->cumulate_D - Uhead->pred->cumulate_D + Uhead->pred->cumulate_P;
			if (valid[NodeU->route->index].size() > 0)
			{
				for (mynode = Utail->next; mynode->index != Vtail->next->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
				for (mynode = Uhead; mynode->index != Utail->next->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
			}
		}
		else
		{
			test = NodeU->route->DepotNode->pred->cumulate_D - Vtail->cumulate_D + Vtail->cumulate_P;
			if (valid[NodeU->route->index].size() > 0)
			{
				for (mynode = Uhead; mynode->index != Utail->next->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
				for (mynode = Vtail->next; mynode->index != Uhead->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
			}
		}
		if (Utail->next->index == Vhead->index)
		{
			Uhead->pred->next = Vhead;
			Vhead->pred = Uhead->pred;
			Vtail->next->pred = Utail;
			Utail->next = Vtail->next;
			Uhead->pred = Vtail;
			Vtail->next = Uhead;
		}
		else
		{
			Uhead->pred->next = Utail->next;
			Utail->next->pred = Uhead->pred;
			Vtail->next->pred = Utail;
			Utail->next = Vtail->next;
			Vtail->next = Uhead;
			Uhead->pred = Vtail;
		}
		operate_num++;
		updateRoute(NodeU->route, Node_Vec, para);
		the_indiv->cost += delta;
		return true;
	}
	else
	{	//just need to check the route which includes node v
		double test = NodeV->route->DepotNode->pred->cumulate_D + Utail->cumulate_D - Uhead->pred->cumulate_D;
		double testp = NodeV->route->DepotNode->pred->cumulate_P + Utail->cumulate_P - Uhead->pred->cumulate_P;
		if (test > capacity_precision || testp > capacity_precision)return false;
		Node* mynode = NULL;
		if (valid[NodeU->route->index].size() > 0 || valid[NodeV->route->index].size() > 0)
		{
			for (mynode = NodeV->route->DepotNode->next; mynode->index != Vtail->next->index; mynode = mynode->next)
			{
				test += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (test > capacity_precision)return false;
			}
			for (mynode = Uhead; mynode->index != Utail->next->index; mynode = mynode->next)
			{
				test += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (test > capacity_precision)return false;
			}
			for (mynode = Vtail->next; !mynode->isDepot; mynode = mynode->next)
			{
				test += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (test > capacity_precision)return false;
			}
		}
		TwoRoute* myroute = NodeU->route;
		Insert(Uhead, Utail, Vtail);
		operate_num++;
		updateRoute(NodeV->route, Node_Vec, para);
		updateRoute(myroute, Node_Vec, para);
		the_indiv->cost += delta;
		return true;
	}
}
bool LocalSearch::MOVE2(std::vector<Node>& Node_Vec, const GlobalParameter& para)
{//swap u's v's
	if (SameRoute && SamePoint)return false;
	delta = -para.Dist[UWhich][Uhead->pred->whichpoint] - para.Dist[UWhich][Utail->next->whichpoint]
		- para.Dist[VWhich][Vhead->pred->whichpoint] - para.Dist[VWhich][Vtail->next->whichpoint]
		+ para.Dist[UWhich][Vhead->pred->whichpoint] + para.Dist[UWhich][Vtail->next->whichpoint]
		+ para.Dist[VWhich][Uhead->pred->whichpoint] + para.Dist[VWhich][Utail->next->whichpoint];
	Node* head1 = Uhead;
	Node* tail1 = Utail;
	Node* head2 = Vhead;
	Node* tail2 = Vtail;
	if (SameRoute && NodeU->position > NodeV->position)
	{
		head1 = Vhead;
		tail1 = Vtail;
		head2 = Uhead;
		tail2 = Utail;
	}
	if (SameRoute && tail1->next->index == head2->index)
	{
		delta =
			-para.Dist[head1->whichpoint][head1->pred->whichpoint] - para.Dist[tail2->whichpoint][tail2->next->whichpoint]
			+ para.Dist[head1->pred->whichpoint][head2->whichpoint] + para.Dist[tail1->whichpoint][tail2->next->whichpoint];
	}
	if (delta > -para.Precision)return false;
	if (SameRoute)
	{//sameroute, just check load between these two pairs
		Node* mynode = NULL;
		if (tail1->next->index == head2->index)
		{
			double test = NodeU->route->DepotNode->pred->cumulate_D - head1->pred->cumulate_D + head1->pred->cumulate_P;
			if (valid[NodeU->route->index].size() > 0)
			{
				for (mynode = head2; mynode->index != tail2->next->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
				for (mynode = head1; mynode->index != head2->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
			}
			head2->pred = head1->pred;
			tail1->next = tail2->next;
			head1->pred->next = head2;
			tail2->next->pred = tail1;
			head1->pred = tail2;
			tail2->next = head1;
			operate_num++;
			updateRoute(NodeU->route, Node_Vec, para);
			the_indiv->cost += delta;
			
			return true;
		}
		else
		{
			double test = NodeU->route->DepotNode->pred->cumulate_D - head1->pred->cumulate_D + head1->pred->cumulate_P;
			if (valid[NodeU->route->index].size() > 0)
			{
				for (mynode = head2; mynode->index != tail2->next->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
				for (mynode = tail1->next; mynode->index != head2->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
				for (mynode = head1; mynode->index != tail1->next->index; mynode = mynode->next)
				{
					test += mynode->LoadDelta;
					if (!mynode->PbiggerD)continue;
					if (test > capacity_precision)return false;
				}
			}
			Swap(head1, tail1, head2, tail2);
			operate_num++;
			updateRoute(NodeU->route, Node_Vec, para);
			the_indiv->cost += delta;
			
			return true;
		}
	}
	else
	{
		Node* mynode = NULL;
		double loadu = NodeU->route->DepotNode->pred->cumulate_D - tail1->cumulate_D + head1->pred->cumulate_D + tail2->cumulate_D - head2->pred->cumulate_D;
		double loadv = NodeV->route->DepotNode->pred->cumulate_D + tail1->cumulate_D - head1->pred->cumulate_D - tail2->cumulate_D + head2->pred->cumulate_D;
		if (loadu > capacity_precision || loadv > capacity_precision)return false;
		if (valid[NodeU->route->index].size() > 0 || valid[NodeV->route->index].size() > 0)
		{
			for (mynode = NodeU->route->DepotNode->next; mynode->index != head1->index; mynode = mynode->next)
			{
				loadu += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (loadu > capacity_precision)return false;
			}
			for (mynode = head2; mynode->index != tail2->next->index; mynode = mynode->next)
			{
				loadu += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (loadu > capacity_precision)return false;
			}
			for (mynode = tail1->next; !mynode->isDepot; mynode = mynode->next)
			{
				loadu += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (loadu > capacity_precision)return false;
			}
			for (mynode = NodeV->route->DepotNode->next; mynode->index != head2->index; mynode = mynode->next)
			{
				loadv += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (loadv > capacity_precision)return false;
			}
			for (mynode = head1; mynode->index != tail1->next->index; mynode = mynode->next)
			{
				loadv += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (loadv > capacity_precision)return false;
			}
			for (mynode = tail2->next; !mynode->isDepot; mynode = mynode->next)
			{
				loadv += mynode->LoadDelta;
				if (!mynode->PbiggerD)continue;
				if (loadv > capacity_precision)return false;
			}
		}
		Swap(head1, tail1, head2, tail2);
		operate_num++;
		updateRoute(NodeU->route, Node_Vec, para);
		updateRoute(NodeV->route, Node_Vec, para);
		the_indiv->cost += delta;
		
		return true;
	}
}
bool LocalSearch::MOVE4(std::vector<Node>& Node_Vec, const GlobalParameter& para)
{//2opt_  feature : orders of the same point  cannot reverse, just move     to remain valid
	if (Uhead->pred->index == Vtail->index || Utail->next->index == Vhead->index)return false;
	if (!NodeX->isDepot && Xtail->next->index == Vhead->index)return false;
	if (!NodeY->isDepot && Ytail->next->index == Uhead->index)return false;
	Node* first = NodeU->position < NodeV->position ? Uhead : Vhead;
	Node* end = NodeU->position < NodeV->position ? Vtail : Utail;
	delta = -para.Dist[first->whichpoint][first->pred->whichpoint] - para.Dist[end->whichpoint][end->next->whichpoint]
		+ para.Dist[first->whichpoint][end->next->whichpoint] + para.Dist[first->pred->whichpoint][end->whichpoint];
	if (delta >= -para.Precision)return false;
	if(valid[NodeU->route->index].size() > 0)
	{
		Node* tmp = NodeU->position < NodeV->position ? Vhead : Uhead;	
		Node* check = tmp;
		double test = NodeU->route->DepotNode->pred->cumulate_D - first->pred->cumulate_D + first->pred->cumulate_P;
		int whichpoint = first->pred->whichpoint;
		Node* mynode = NULL;
		for (mynode = first->pred; mynode->whichpoint == whichpoint; mynode = mynode->pred);
		mynode = mynode->next;
		int index = mynode->index;

		for (; check->index != index; tmp = tmp->next, check = tmp)
		{
			for (; check->whichpoint == tmp->whichpoint; check = check->next)
			{
				test += check->LoadDelta;
				if (!check->PbiggerD)continue;
				if (test > capacity_precision)return false;
			}
			for (tmp = tmp->pred, whichpoint = tmp->whichpoint; tmp->whichpoint == whichpoint; tmp = tmp->pred);
		}
	}	
	//feasible check over
	Node* next = end->next;
	Node* pointtail = first;	//record the tail of this pair
	Node* pointhead = first;
	Node* pointtailnext;		//record the next of the tail of this pair
	Node* firstpred = first->pred;
	Node* endnext = end->next;
	while(pointhead->index!=endnext->index)
	{
		for (; pointtail->next->whichpoint == pointhead->whichpoint; pointtail = pointtail->next);
		pointtailnext = pointtail->next;
		pointtail->next = next;
		next->pred = pointtail;
		next = pointhead;
		pointhead = pointtailnext;
		pointtail = pointhead;
	}
	firstpred->next = next;
	next->pred = firstpred;
	operate_num++;
	updateRoute(NodeU->route, Node_Vec, para);
	the_indiv->cost += delta;
	return true;
}
bool LocalSearch::MOVE5(std::vector<Node>& Node_Vec, const GlobalParameter& para)
{	// (u,x) (v,y) -> (u,y) (v,x)
	delta = -para.Dist[UWhich][XWhich] - para.Dist[VWhich][YWhich]
		+ para.Dist[UWhich][YWhich] + para.Dist[XWhich][VWhich];
	if (delta >= -para.Precision)return false;

	double testu = Utail->cumulate_D + NodeV->route->DepotNode->pred->cumulate_D - Vtail->cumulate_D;//load at beginning 
	double testv = Vtail->cumulate_D + NodeU->route->DepotNode->pred->cumulate_D - Utail->cumulate_D;
	double up = Utail->cumulate_P + NodeV->route->DepotNode->pred->cumulate_P - Vtail->cumulate_P;//load at end
	double vp= Vtail->cumulate_P + NodeU->route->DepotNode->pred->cumulate_P - Utail->cumulate_P;
	Node* my;
	if (testu > capacity_precision || testv > capacity_precision || up > capacity_precision || vp > capacity_precision)return false;
	if (valid[NodeU->route->index].size() > 0 || valid[NodeV->route->index].size() > 0)
	{	
		for (my = NodeU->route->DepotNode->next; my->index != Xhead->index; my = my->next)
		{
			testu += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testu > capacity_precision)return false;
		}
		for (my = Yhead; !my->isDepot; my = my->next)
		{
			testu += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testu > capacity_precision)return false;
		}
		for (my = NodeV->route->DepotNode->next; my->index != Yhead->index; my = my->next)
		{
			testv += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testv > capacity_precision)return false;
		}
		for (my = Xhead; !my->isDepot; my = my->next)
		{
			testv += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testv > capacity_precision)return false;
		}
	}
	Xhead->pred = Vtail;
	Vtail->next = Xhead;
	Yhead->pred = Utail;
	Utail->next = Yhead;
	for (my = Yhead; !my->isDepot; my = my->next)my->route = NodeU->route;
	Node* routeVtail = my->pred;
	for (my = Xhead; !my->isDepot; my = my->next)my->route = NodeV->route;
	Node* routeUtail = my->pred;
	NodeU->route->DepotNode->pred = routeVtail;
	routeVtail->next = NodeU->route->DepotNode;
	NodeV->route->DepotNode->pred = routeUtail;
	routeUtail->next = NodeV->route->DepotNode;
	operate_num++;
	updateRoute(NodeU->route, Node_Vec, para);
	updateRoute(NodeV->route, Node_Vec, para);
	the_indiv->cost += delta;
	return true;
}
bool LocalSearch::MOVE6(std::vector<Node>& Node_Vec, const GlobalParameter& para)
{//(u, x) (v, y) -> (u, v) (x, y)
	delta = -para.Dist[UWhich][XWhich] - para.Dist[VWhich][YWhich]
		+ para.Dist[UWhich][VWhich] + para.Dist[XWhich][YWhich];
	if (delta >= -para.Precision || (NodeX->isDepot && NodeY->isDepot))return false;

	double testu = Utail->cumulate_D + Vtail->cumulate_D;
	double testv = NodeU->route->DepotNode->pred->cumulate_D - Utail->cumulate_D + NodeV->route->DepotNode->pred->cumulate_D - Vtail->cumulate_D;
	double up = Utail->cumulate_P + Vtail->cumulate_P;
	double vp= NodeU->route->DepotNode->pred->cumulate_P - Utail->cumulate_P + NodeV->route->DepotNode->pred->cumulate_P - Vtail->cumulate_P;
	if (testu > capacity_precision || testv > capacity_precision || up > capacity_precision || vp > capacity_precision)return false;
	Node* my;
	if (valid[NodeU->route->index].size() > 0 || valid[NodeV->route->index].size() > 0)
	{
		for (my = NodeU->route->DepotNode->next; my->index != Xhead->index; my = my->next)
		{
			testu += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testu > capacity_precision)return false;
		}
		for (my = Vtail; !my->isDepot; my = my->pred)
		{
			testu += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testu > capacity_precision)return false;
		}
		for (my = NodeU->route->DepotNode->pred; my->index != Utail->index; my = my->pred)
		{
			testv += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testv > capacity_precision)return false;
		}
		for (my = Yhead; !my->isDepot; my = my->next)
		{
			testv += my->LoadDelta;
			if (!my->PbiggerD)continue;
			if (testv > capacity_precision)return false;
		}
	}
	Node* pred = Utail;
	my = Vtail;
	Node* next = my->pred;
	while(!my->isDepot)
	{
		my->route = NodeU->route;
		my->pred = pred;
		pred->next = my;
		pred = my;
		my = next;
		next = next->pred;
	}
	Node* routeVhead = pred;
	pred = Yhead;
	my = Xhead;
	next = my->next;
	while (!my->isDepot)
	{
		my->route = NodeY->route;
		my->next = pred;
		pred->pred = my;
		pred = my;
		my = next;
		next = next->next;
	}
	Node* routeUtail = pred;
	if (Xhead->isDepot)//x maybe depot 
	{
		routeUtail = NodeY;
	}
	NodeU->route->DepotNode->pred = routeVhead;
	routeVhead->next = NodeU->route->DepotNode;
	NodeY->route->DepotNode->next = routeUtail;
	routeUtail->pred = NodeY->route->DepotNode;
	operate_num++;
	updateRoute(NodeU->route, Node_Vec, para);
	updateRoute(NodeY->route, Node_Vec, para);
	the_indiv->cost += delta;
	return true;
}
bool LocalSearch::elimination(std::vector<Node>& Node_Vec, const GlobalParameter& para)
{
	if (!the_indiv->need_elimination)return false;
	count_size = 0;
	for (int i = 0; i < RouteVector.size(); i++)
	{
		count_size += RouteVector[i]->DepotNode->next->isDepot ? 0 : 1;
	}
	if (count_size <= para.Vehicle)
	{
		the_indiv->need_elimination = false;
		return false;
	}
	TwoRoute* myroute = nullptr;
	Node* mynode = nullptr;
	for (int i = 0,count=9999; i < RouteVector.size(); i++)
	{
		if (RouteVector[i]->DepotNode->next->isDepot)continue;
		if (RouteVector[i]->DepotNode->pred->position < count)
		{
			myroute = RouteVector[i];
			count = myroute->DepotNode->position;
		}
	}
	std::vector<double>remainload(RouteVector.size(), 0.);		// store ( capacity - maxload) of each route , to represent the number of load can add, after  simplification
	std::vector<bool>flag(RouteVector.size(), false);
	bool haschanged = false;
	for (int i = 0; i < RouteVector.size(); i++)
	{
		if (RouteVector[i]->DepotNode->next->isDepot || RouteVector[i]->index == myroute->index)continue;
		remainload[i] = para.Capacity - RouteVector[i]->MaxLoad;
	}
	if (myroute != nullptr) {
		for (Node* eliNode = myroute->DepotNode->next; !eliNode->isDepot;)
		{
			Node* nextNode = eliNode->next;
			for (int i = 0; i < RouteVector.size(); i++)
			{
				if (i == myroute->index)continue;
				if (eliNode->demand_D > remainload[i] + para.Precision || eliNode->demand_P > remainload[i] + para.Precision)continue;
				the_indiv->cost +=
					-para.Dist[eliNode->whichpoint][eliNode->pred->whichpoint] - para.Dist[eliNode->whichpoint][eliNode->next->whichpoint]
					+ para.Dist[eliNode->pred->whichpoint][eliNode->next->whichpoint]
					- para.Dist[0][RouteVector[i]->DepotNode->next->whichpoint]
					+ para.Dist[eliNode->whichpoint][RouteVector[i]->DepotNode->next->whichpoint] + para.Dist[0][eliNode->whichpoint];
				Insert(eliNode, RouteVector[i]->DepotNode);
				haschanged = true;
				flag[i] = true;
				flag[myroute->index] = true;
				remainload[i] -= std::max<double>(eliNode->demand_D, eliNode->demand_P);
				break;
			}
			eliNode = nextNode;
		}
	}
	
	if (haschanged)operate_num++;
	for (int i = 0; i < RouteVector.size(); i++)
	{
		if (flag[i])updateRoute(RouteVector[i], Node_Vec, para);
	}
	if (myroute->DepotNode->next->isDepot)
	{	// empty route , means this route is eliminated successfully
		count_size--;
		if (count_size == para.Vehicle)the_indiv->need_elimination = false;
		return true;
	}
	else return false;
}
bool LocalSearch::GetNodeV()
{
	VWhich = NodeV->whichpoint, VIndex = NodeV->index;
	if (NodeV->isDepot)return true;
	SamePoint = UWhich == VWhich;
	if (NodeU->index == NodeV->index)return true;
	SameRoute = NodeU->route->index == NodeV->route->index;

	Uhead = NodeU, Utail = NodeU;
	while (Uhead->pred->whichpoint == UWhich)Uhead = Uhead->pred;
	while (Utail->next->whichpoint == UWhich)Utail = Utail->next;
	NodeX = Utail->next;
	XWhich = NodeX->whichpoint;
	Xhead = NodeX, Xtail = NodeX;
	if (!NodeX->isDepot)while (XWhich == Xtail->next->whichpoint)Xtail = Xtail->next;

	Vhead = NodeV, Vtail = NodeV;
	while (Vhead->pred->whichpoint == VWhich)Vhead = Vhead->pred;
	if (!markpointifchanged && Vhead->index == tmpnode->index)return true;
	tmpnode = Vhead;
	while (Vtail->next->whichpoint == VWhich)Vtail = Vtail->next;
	NodeY = Vtail->next;
	YWhich = NodeY->whichpoint;
	Yhead = NodeY, Ytail = NodeY;
	if (!NodeY->isDepot)while (YWhich == Ytail->next->whichpoint)Ytail = Ytail->next;
	if (SameRoute && Uhead->index == Vhead->index)return true;
	return false;
}
void LocalSearch::updateRoute(TwoRoute* Route, std::vector<Node>& Node_Vec, const GlobalParameter& para)
{
	// update route information and aux information 
	markpointifchanged = true;
	int pos = -1;
	double cumD = 0;
	double cumP = 0;
	Node* thisnode = Route->DepotNode->next;
	Route->Record_Last_Change = operate_num;		
	valid[Route->index].clear();
	while (!thisnode->isDepot)
	{//need update:1.cumulateD 2.cumulateP 3.position 4.valid 5.maxLoad of route
		cumD += thisnode->demand_D;
		cumP += thisnode->demand_P;
		pos++;

		thisnode->cumulate_D = cumD;
		thisnode->cumulate_P = cumP;
		thisnode->position = pos;
		if (thisnode->PbiggerD)valid[Route->index].push_back(thisnode->index);
		thisnode = thisnode->next;
	}
	Route->MaxLoad = cumD;
	if (valid[Route->index].size() > 0)
	{
		for (Node* mynode = Route->DepotNode->next; !mynode->isDepot; mynode = mynode->next)
		{
			if (!mynode->PbiggerD)continue;
			Route->MaxLoad = Route->MaxLoad > (cumD - mynode->cumulate_D + mynode->cumulate_P) ? Route->MaxLoad : cumD - mynode->cumulate_D + mynode->cumulate_P;
		}
	}
}
void LocalSearch::GetRouteInformFromNode(Individual& indiv, std::vector<Node>& Node_Vec, const GlobalParameter& para)
{	// obtain routes of indiv from RouteVector 
	indiv.Route.clear();
	for (int i = 0, count = 0; i < RouteVector.size(); i++)
	{
		Node* thisnode = RouteVector[i]->DepotNode->next;
		if (thisnode->isDepot)continue;//empty route 
		indiv.Route.push_back(std::vector<int>());
		while (!thisnode->isDepot)
		{
			indiv.Route[count].push_back(thisnode->index);
			thisnode = thisnode->next;
		}
		count++;
	}
}


LocalSearch::LocalSearch(std::vector<Node>& Node_Vec, const GlobalParameter& para)
{	//initialize aux information's data structure 
	U_vector = std::vector<int>(para.Num - 1, 0);
	for (int i = 1; i <= para.Num - 1; i++)U_vector[i - 1] = i;
	NodeGroup = std::vector<std::vector<Node*>>(para.Dimension, std::vector<Node*>());
	int point = 0;
	for (int i = 0; i < para.Num;)
	{
		if (Node_Vec[i].whichpoint == point)NodeGroup[point].push_back(&Node_Vec[i++]);
		else point++;
	}
	for (int i = 0; i < para.Dimension; i++)near.push_back(Nearby(i, para));
	capacity_precision = para.Capacity + para.Precision;
}
