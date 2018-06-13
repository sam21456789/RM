#include <iostream>
#include <fstream>
#include <ctime>
#include <cstdlib>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <utility>
#include <iomanip>
#include <fstream>
#include <chrono>
#include "rm.h"

#define near 100

void print(vector<node>& ary)
{
    for(auto& i:ary)
    {
        cout << setw(4) << i.NO;
    }
    cout << endl;
}
//-----------------------------------------------------------------------------------------------------------
rm::rm(const string inputdata,const double maxtime)
{
    cout << inputdata << endl;
    this->Mtime=maxtime*60;
    this->input=inputdata;
    string data="dataset/"+input+".txt";
    fstream file;
    file.open(data,ios::in);
    //--------------------------------------------------------------------
    int tmpno,tmpd;
    double tmpx,tmpy,tmpr,tmpe,tmpt;
    file >> maxVehicle >> Vcapacity;
    while(file >> tmpno >> tmpx >> tmpy >> tmpd >> tmpr >> tmpe >> tmpt)
        custList.push_back(customer(tmpno,tmpx,tmpy,tmpd,tmpr,tmpe,tmpt));
    file.close();
    //--------------------------------------------------------------------
    int Ncust=custList.size();
    this->NumOfCust=Ncust-1;
    this->Dist_table=vector<vector<double>>(Ncust,vector<double>(Ncust));
    this->Nlist.resize(Ncust);
    vector<vector<pair<double,int>>> Nclosest(Ncust);
    for(int i=0; i<Ncust; ++i)
    {
        for(int j=0; j<=i; ++j)
        {
            double dist=sqrt(pow(custList[i].X-custList[j].X,2)+pow(custList[i].Y-custList[j].Y,2));
            Dist_table[i][j]=Dist_table[j][i]=dist;
            if(i!=j)
            {
                Nclosest[i].push_back(make_pair(dist,j));
                Nclosest[j].push_back(make_pair(dist,i));
            }
        }
    }
    for(unsigned int i=0; i<Nclosest.size(); ++i)
    {
        sort(Nclosest[i].begin(),Nclosest[i].end());
        Nlist[i].resize(near);
        for(int j=0; j<near; ++j)
            Nlist[i][j]=Nclosest[i][j].second;
    }
}

//bool rm::check(vector<node>& route)                //Check if this route in this route satisfy the capacity or the time windows
//{
//    for(auto i=route.begin()+1; i!=route.end(); ++i)
//    {
//        double dist=Dist_table[(i-1)->NO][i->NO];
//        double st=custList[(i-1)->NO].Tready;
//        i->Arrival=max((i-1)->Arrival+st+dist,custList[i->NO].Tready);
//        if(i->Arrival >i ->ltArrival)
//            return false;
//    }
//    return true;
//}

bool rm::check(vector<node>& route)                  //Check if this route in this route satisfy the capacity or the time windows
{
    for(auto j=route.begin()+1; j!=route.end(); ++j)
    {
        int src= (j-1)->NO;
        int dest= j->NO;
        double a_j = (j-1)->Arrival + custList[src].Tservice + Dist_table[src][dest];

        if(a_j <= custList[dest].Tdue)      // a_j <= l_j
            j->Arrival= max(a_j,custList[dest].Tready);
        else
            return false;
    }
    return true;
}

void rm::randominsert(node& cust, solution& s, vector<solution>& feasible_set, vector<pair<object,solution>>& infeasible_set)
{
    feasible_set.clear();
    infeasible_set.clear();
    solution tmp=s;  //copy solution
    double minFp=DBL_MAX;
    for(unsigned int i=0; i<s.route_set.size(); ++i)          //calculate all possible position
    {
        for(unsigned int j=1; j<s.route_set[i].size(); ++j)
        {
            vector<node>& arr=tmp.route_set[i];
            arr.insert(arr.begin()+j,cust);
            tmp.setCost(Dist_table,custList,Vcapacity,alpha);
            if(tmp.Fp==0)
                feasible_set.push_back(tmp);
            else
            {
                if(tmp.Fp<minFp)
                    minS=tmp;
                infeasible_set.push_back(make_pair(object(i,j,tmp.Fp),tmp));
            }

            arr.erase(arr.begin()+j);
        }
    }
}

void rm::squeeze(solution& s,int& position)
{
    solution n1=intra_relocation(s,position);
    solution n2=inter_relocation(s,position);
    solution n3=intra_exchange(s,position);
    solution n4=inter_exchange(s,position);
    solution n5=t_opt(s,position);
    vector<solution> neighbor(5);
    neighbor[0]=n1;
    neighbor[1]=n2;
    neighbor[2]=n3;
    neighbor[3]=n4;
    neighbor[4]=n5;
    sort(neighbor.begin(),neighbor.end());
    s=neighbor.front();
}
//------------------------------------------------------------------------------------------------------------
//Local search
//------------------------------------------------------------------------------------------------------------
solution rm::t_opt(solution& s,int& position)   //OK part
{
    solution tmp=s;
    int rsize=tmp.route_set.size();
    vector<node>& ary1=tmp.route_set[position];
//    int r1,r2,p,cust;
//    vector<int>::iterator clost;
//    do
//    {
//        r1=rand()%(ary1.size()-1);
//        cust=ary1[r1].NO;
//        do
//        {
//            p=rand()%rsize;
//        }
//        while(p==position);
//        vector<node>& ary=tmp.route_set[p];
//        r2=rand()%(ary.size()-1);
//        clost=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2+1].NO);
//    }
//    while(clost==Nlist[cust].end());
//
//    vector<node>& ary2=tmp.route_set[p];
//    vector<node> t1(ary1.begin(),ary1.begin()+r1+1);
//    t1.insert(t1.end(),ary2.begin()+r2+1,ary2.end());
//    vector<node> t2(ary2.begin(),ary2.begin()+r2+1);
//    t2.insert(t2.end(),ary1.begin()+r1+1,ary1.end());
//    ary1=t1;
//    ary2=t2;
//    tmp.setCost(Dist_table,custList,Vcapacity,alpha);
//    return tmp;

    solution minNeighbor=s;
    for(unsigned int i=0; i<ary1.size()-1; ++i)
    {
        int cust=ary1[i].NO;
        for(int j=0; j<rsize; ++j)
        {
            if(j!=position)
            {
                vector<node>& ary2=tmp.route_set[j];
                vector<node> c1=ary1;
                vector<node> c2=ary2;
                for(unsigned int k=1; k<ary2.size()-1; ++k)
                {
                    auto clost=find(Nlist[cust].begin(),Nlist[cust].end(),ary2[k+1].NO);
                    if(clost!=Nlist[cust].end())
                    {
                        vector<node> t1(ary1.begin(),ary1.begin()+i+1);
                        t1.insert(t1.end(),ary2.begin()+k+1,ary2.end());
                        vector<node> t2(ary2.begin(),ary2.begin()+k+1);
                        t2.insert(t2.end(),ary1.begin()+i+1,ary1.end());
                        ary1=t1;
                        ary2=t2;
                        tmp.setCost(Dist_table,custList,Vcapacity,alpha);
                        if(tmp.Fp<minNeighbor.Fp)
                            minNeighbor=tmp;
                        ary1=c1;
                        ary2=c2;
                    }
                }
            }
        }
    }
    return minNeighbor;
}

solution rm::intra_relocation(solution& s,int& position) //OK
{
    solution tmp=s;
    vector<node>& ary=tmp.route_set[position];
//    vector<int>::iterator predecessor,succesor;
//    int r,l,cust;
//    do
//    {
//        r=rand()%(ary.size()-2)+1;
//        cust=ary[r].NO;
//        l=rand()%(ary.size()-2)+1;  //insert position
//        if(l<r)
//        {
//            predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[l-1].NO);
//            succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[l].NO);
//        }
//        if(l>r)
//        {
//            predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[l].NO);
//            succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[l+1].NO);
//        }
//    }while(r==l||(predecessor==Nlist[cust].end()&&succesor==Nlist[cust].end()));
//    ary.erase(ary.begin()+r);
//    ary.insert(ary.begin()+l,node(cust));
//    tmp.setCost(Dist_table,custList,Vcapacity,alpha);

//    return tmp;

    solution minNeighbor=s;
    int rsize=ary.size();
    for(int i=1; i<rsize-1; ++i)
    {
        int cust=ary[i].NO;
        for(int j=1; j<rsize-1; ++j)
        {
            if(j<i)
            {
                auto predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[j-1].NO);
                auto succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[j].NO);
                if(predecessor!=Nlist[cust].end()||succesor!=Nlist[cust].end())
                {
                    ary.erase(ary.begin()+i);
                    ary.insert(ary.begin()+j,node(cust));
                    tmp.setCost(Dist_table,custList,Vcapacity,alpha);
                    if(tmp.Fp<minNeighbor.Fp)
                        minNeighbor=tmp;

                    ary.erase(ary.begin()+j);
                    ary.insert(ary.begin()+i,node(cust));
                }
            }
            if(j>i)
            {
                auto predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[j].NO);
                auto succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[j+1].NO);
                if(predecessor!=Nlist[cust].end()||succesor!=Nlist[cust].end())
                {
                    ary.erase(ary.begin()+i);
                    ary.insert(ary.begin()+j,node(cust));
                    tmp.setCost(Dist_table,custList,Vcapacity,alpha);
                    if(tmp.Fp<minNeighbor.Fp)
                        minNeighbor=tmp;

                    ary.erase(ary.begin()+j);
                    ary.insert(ary.begin()+i,node(cust));
                }
            }
        }
    }
    return minNeighbor;

}

solution rm::inter_relocation(solution& s,int& position) //OK
{
    solution tmp=s;
    int rsize=tmp.route_set.size();
    vector<node>& ary1=tmp.route_set[position];  //infeasible route
//    int r1,r2,p,cust;
//    vector<int>::iterator predecessor,succesor;
//    do
//    {
//        r1=rand()%(ary1.size()-2)+1;
//        cust=ary1[r1].NO;
//        do
//        {
//            p=rand()%rsize;
//        }while(p==position);
//        vector<node>& ary=tmp.route_set[p];
//        r2=rand()%(ary.size()-1)+1;
////        predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2-1].NO);
////        succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2].NO);
//    }while(predecessor==Nlist[cust].end()&&succesor==Nlist[cust].end());
//    vector<node>& ary2=tmp.route_set[p];
//    ary1.erase(ary1.begin()+r1);
//    ary2.insert(ary2.begin()+r2,node(cust));
//    tmp.setCost(Dist_table,custList,Vcapacity,alpha);

//    return tmp;

    solution minNeighbor=s;
    for(unsigned int k=1; k<ary1.size()-1; ++k)
    {
        int cust=ary1[k].NO;
        for(int i=0; i<rsize; ++i)
        {
            if(i!=position)
            {
                vector<node>& ary2=tmp.route_set[i];
                for(unsigned int j=1; j<ary2.size(); ++j)
                {
                    auto predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary2[j-1].NO);
                    auto succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary2[j].NO);
                    if(predecessor!=Nlist[cust].end()||succesor!=Nlist[cust].end())
                    {
                        ary1.erase(ary1.begin()+k);
                        ary2.insert(ary2.begin()+j,node(cust));
                        tmp.setCost(Dist_table,custList,Vcapacity,alpha);
                        if(tmp.Fp<minNeighbor.Fp)
                            minNeighbor=tmp;
                        ary2.erase(ary2.begin()+j);
                        ary1.insert(ary1.begin()+k,node(cust));
                    }
                }
            }
        }
    }
    return minNeighbor;
}

solution rm::intra_exchange(solution& s,int& position)   //OK
{
    solution tmp=s;
    vector<node>& ary=tmp.route_set[position];
//    vector<int>::iterator clost;
//    int r,l,cust;
//    do
//    {
//        r=rand()%(ary.size()-2)+1;
//        cust=ary[r].NO;
//        l=rand()%(ary.size()-2)+1;
//        clost=find(Nlist[cust].begin(),Nlist[cust].end(),ary[l].NO);
//    }while(l==r||clost==Nlist[cust].end());
//    swap(ary[r],ary[l]);
//    tmp.setCost(Dist_table,custList,Vcapacity,alpha);
//
//    return tmp;

    solution minNeighbor=s;
    int rsize=ary.size();
    for(int i=1; i<rsize-1; ++i)
    {
        int cust=ary[i].NO;
        for(int j=i+1; j<rsize-1; ++j)
        {
            auto clost=find(Nlist[cust].begin(),Nlist[cust].end(),ary[j].NO);
            if(clost!=Nlist[cust].end())
            {
                swap(ary[i],ary[j]);
                tmp.setCost(Dist_table,custList,Vcapacity,alpha);
                if(tmp.Fp<minNeighbor.Fp)
                    minNeighbor=tmp;
                swap(ary[i],ary[j]);
            }
        }
    }
    return minNeighbor;
}

solution rm::inter_exchange(solution& s,int& position)   //OK
{
    solution tmp=s;
    int rsize=tmp.route_set.size();
    vector<node>& ary1=tmp.route_set[position];
//    int r1,r2,p,cust;
//    vector<int>::iterator clost;
//    do
//    {
//        r1=rand()%(ary1.size()-2)+1;
//        cust=ary1[r1].NO;
//        do
//        {
//            p=rand()%rsize;
//        }while(p==position);
//        vector<node>& ary=tmp.route_set[p];
//        r2=rand()%(ary.size()-2)+1;
//        clost=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2].NO);
//    }while(clost==Nlist[cust].end());
//    vector<node>& ary2=tmp.route_set[p];
//    swap(ary1[r1],ary2[r2]);
//    tmp.setCost(Dist_table,custList,Vcapacity,alpha);
//    return tmp;

    solution minNeighbor=s;
    for(unsigned int k=1; k<ary1.size()-1; ++k)
    {
        int cust=ary1[k].NO;
        for(int i=0; i<rsize; ++i)
        {
            if(i!=position)
            {
                vector<node>& ary2=tmp.route_set[i];
                for(unsigned int j=1; j<ary2.size()-1; ++j)
                {
                    auto clost=find(Nlist[cust].begin(),Nlist[cust].end(),ary2[j].NO);
                    if(clost!=Nlist[cust].end())
                    {
                        swap(ary1[k],ary2[j]);
                        tmp.setCost(Dist_table,custList,Vcapacity,alpha);
                        if(tmp.Fp<minNeighbor.Fp)
                            minNeighbor=tmp;
                        swap(ary1[k],ary2[j]);
                    }
                }
            }
        }
    }
    return minNeighbor;
}
//------------------------------------------------------------------------------------------------------------
solution rm::t_opt(solution& s) //OK part
{
    solution tmp=s;
    int rsize=tmp.route_set.size();
    int r1,r2,p,cust,position;
    vector<int>::iterator closest;
    do
    {
        position=rand()%rsize;
        vector<node> arr=tmp.route_set[position];
        r1=rand()%(arr.size()-1);
        cust=arr[r1].NO;
        do
        {
            p=rand()%rsize;
        }
        while(p==position);
        vector<node>& ary=tmp.route_set[p];
        r2=rand()%(ary.size()-1);
        closest=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2+1].NO);
    }
    while(closest==Nlist[cust].end());

    vector<node>& ary1=tmp.route_set[position];
    vector<node>& ary2=tmp.route_set[p];
    vector<node> t1(ary1.begin(),ary1.begin()+r1+1);
    t1.insert(t1.end(),ary2.begin()+r2+1,ary2.end());
    vector<node> t2(ary2.begin(),ary2.begin()+r2+1);
    t2.insert(t2.end(),ary1.begin()+r1+1,ary1.end());
    ary1=t1;
    ary2=t2;
    tmp.setCost(Dist_table,custList,Vcapacity,alpha);

    return tmp;
}

solution rm::intra_relocation(solution& s) //OK
{
    solution tmp=s;
    int rsize=tmp.route_set.size();
    int r,l,cust,position;
    vector<int>::iterator predecessor,succesor;
    do
    {
        position=rand()%rsize;
        vector<node>& ary=tmp.route_set[position];
        r=rand()%(ary.size()-2)+1;
        cust=ary[r].NO;
        l=rand()%(ary.size()-2)+1;  //insert position
        predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[l].NO);
        succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[l+1].NO);
    }
    while(r==l||(predecessor==Nlist[cust].end()&&succesor==Nlist[cust].end()));

    vector<node>& ary=tmp.route_set[position];
    ary.erase(ary.begin()+r);
    ary.insert(ary.begin()+l,node(cust));
    tmp.setCost(Dist_table,custList,Vcapacity,alpha);

    return tmp;
}

solution rm::inter_relocation(solution& s) //OK
{
    solution tmp=s;
    int rsize=tmp.route_set.size();

    int r1,r2,p,cust,position;
    vector<int>::iterator predecessor,succesor;
    do
    {
        position=rand()%rsize;
        vector<node>& ary1=tmp.route_set[position];
        r1=rand()%(ary1.size()-2)+1;
        cust=ary1[r1].NO;
        do
        {
            p=rand()%rsize;
        }
        while(p==position);
        vector<node>& ary=tmp.route_set[p];
        r2=rand()%(ary.size()-1)+1;
        predecessor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2-1].NO);
        succesor=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2].NO);
    }
    while(predecessor==Nlist[cust].end()&&succesor==Nlist[cust].end());

    vector<node>& ary1=tmp.route_set[position];
    vector<node>& ary2=tmp.route_set[p];
    ary1.erase(ary1.begin()+r1);
    ary2.insert(ary2.begin()+r2,node(cust));
    tmp.setCost(Dist_table,custList,Vcapacity,alpha);

    return tmp;
}

solution rm::intra_exchange(solution& s)   //OK
{
    solution tmp=s;
    int rsize=tmp.route_set.size();
    int r,l,cust,position;
    vector<int>::iterator clost;
    do
    {
        position=rand()%rsize;
        vector<node>& arr=tmp.route_set[position];
        r=rand()%(arr.size()-2)+1;
        cust=arr[r].NO;
        l=rand()%(arr.size()-2)+1;
        clost=find(Nlist[cust].begin(),Nlist[cust].end(),arr[l].NO);
    }
    while(l==r||clost==Nlist[cust].end());

    vector<node>& ary=tmp.route_set[position];
    swap(ary[r],ary[l]);
    tmp.setCost(Dist_table,custList,Vcapacity,alpha);

    return tmp;
}

solution rm::inter_exchange(solution& s)   //OK
{
    solution tmp=s;
    int rsize=tmp.route_set.size();


    int r1,r2,p,cust,position;
    vector<int>::iterator clost;
    do
    {
        position=rand()%rsize;
        vector<node>& ary1=tmp.route_set[position];
        r1=rand()%(ary1.size()-2)+1;
        cust=ary1[r1].NO;
        do
        {
            p=rand()%rsize;
        }
        while(p==position);
        vector<node>& ary=tmp.route_set[p];
        r2=rand()%(ary.size()-2)+1;
        clost=find(Nlist[cust].begin(),Nlist[cust].end(),ary[r2].NO);
    }
    while(clost==Nlist[cust].end());
    vector<node>& ary1=tmp.route_set[position];
    vector<node>& ary2=tmp.route_set[p];
    swap(ary1[r1],ary2[r2]);
    tmp.setCost(Dist_table,custList,Vcapacity,alpha);

    return tmp;
}
//------------------------------------------------------------------------------------------------------------
//lexicographic order
//------------------------------------------------------------------------------------------------------------
void getAllSubsets(vector<vector<int>>& subset,int& rsize,int& k)
{
    subset.clear();

    if(rsize<k)
        k=rsize;
    vector<int> ar(rsize);
    for(int i=1; i<=rsize; ++i)
        ar[i-1]=i;

    vector<int> empty;
    subset.push_back(empty);
    for(int i=0; i<rsize; ++i)
    {
        vector<vector<int>> subsetTemp = subset;

        for (unsigned int j=0; j<subsetTemp.size(); ++j)
            subsetTemp[j].push_back(ar[i]);

        for (unsigned int j=0; j<subsetTemp.size(); ++j)
        {
            if((int)subsetTemp[j].size()<k)
                subset.push_back(subsetTemp[j]);
        }
    }
}

void rm::Allsubset(vector<node>& route, vector<node>& Eject, int& Pbest, pair<int,vector<node>>& best, int routeNO, vector<int>& pcnt)
{
    int size=route.size()-2;
    vector<int> ejectPool(5);
    for(int i=1; i<=size; ++i)
    {
        ejectPool.resize(1);
        ejectPool[0]=i;
        cal_eject(route,ejectPool,Eject,Pbest,best,routeNO,pcnt);
        for(int j=i+1; j<=size; ++j)
        {
            ejectPool.resize(2);
            ejectPool[1]=j;
            cal_eject(route,ejectPool,Eject,Pbest,best,routeNO,pcnt);
            if(size>=3)
            {
                for(int k=j+1; k<=size; ++k)
                {
                    ejectPool.resize(3);
                    ejectPool[2]=k;
                    cal_eject(route,ejectPool,Eject,Pbest,best,routeNO,pcnt);
                    if(size>=4)
                    {
                        for(int m=k+1; m<=size; ++m)
                        {
                            ejectPool.resize(4);
                            ejectPool[3]=m;
                            cal_eject(route,ejectPool,Eject,Pbest,best,routeNO,pcnt);
                            if(size>=5)
                            {
                                for(int n=m+1; n<=size; ++n)
                                {
                                    ejectPool.resize(5);
                                    ejectPool[4]=n;
                                    cal_eject(route,ejectPool,Eject,Pbest,best,routeNO,pcnt);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void rm::cal_eject(vector<node>& route,vector<int>& order,vector<node>& Eject,int& Pbest,pair<int,vector<node>>& best,int routeNO,vector<int>& pcnt)
{
    int origCap=route.back().cap;
    int tail=order.size();
    int Psum=0;
    int Cap=0;

    for(int i=0; i<tail; ++i)
    {
        int p=order[i];
        Psum+=pcnt[route[p].NO];
        Cap+=custList[route[p].NO].Demand;
    }
    if(Psum<Pbest)  //continue to check feasibility
    {
        vector<node> tmp((int)route.size()-tail);
        int cnt=0;
        for(int l=0,m=0; l<(int)route.size(); ++l)
        {
            if(l!=order[cnt])
            {
                tmp[m]=route[l];
                ++m;
            }
            else
                cnt=((cnt+1)==tail)?0:cnt+1;
        }
        int newC=origCap-Cap;
        if(check(tmp)&&(newC<=Vcapacity))
        {
            Pbest=Psum;
            best=make_pair(routeNO,tmp);   //record the position
            vector<node> ejpool(tail);
            for(int l=0; l<tail; ++l)
            {
                int p=order[l];
                ejpool[l]=route[p];
            }
            Eject=ejpool;            //record the ejected customers
        }
    }
}
//------------------------------------------------------------------------------------------------------------
void rm::perturb(solution& s)
{
    int cnt=0;
    while(cnt<1000)
    {
        solution n1=intra_relocation(s);
        solution n2=inter_relocation(s);
        solution n3=intra_exchange(s);
        solution n4=inter_exchange(s);
        solution n5=t_opt(s);
        vector<solution> neighbor(5);
        neighbor[0]=n1;
        neighbor[1]=n2;
        neighbor[2]=n3;
        neighbor[3]=n4;
        neighbor[4]=n5;
        int r=rand()%5;
        if(neighbor[r].Fp==0)
            s=neighbor[r];
        ++cnt;
    }
}

void rm::Initialize(solution& s)
{
    s.route_set.resize(NumOfCust);
    vector<vector<node>>& Route=s.route_set;
    for(int i=1; i<=NumOfCust; ++i)
    {
        vector<node> tr(3,node(0));
        tr[1].NO=i;
        tr[2].ltArrival=custList[0].Tdue;
        Route[i-1]=tr;
    }
    s.setCost(Dist_table,custList,Vcapacity,alpha);
}
//------------------------------------------------------------------------------------------------------------
void rm::exe()
{
    srand((unsigned)time(NULL));
    //--------------------------------------------------------------------
    //Lower bound
    //--------------------------------------------------------------------
    double q=0;    // total load
    for(auto i=custList.begin()+1; i!=custList.end(); ++i)
        q+=i->Demand;
    unsigned int LB= q/Vcapacity;    //Lower bound of vehicles
    //--------------------------------------------------------------------
    solution sol;
    Initialize(sol);
    //-----------------------------------------------------------------------------------------------------------
    double time=0;
    chrono::steady_clock::time_point  t1, t2;
    t1 = chrono::steady_clock::now();
    while(time<Mtime && sol.route_set.size()>LB)
    {
        vector<node> EP;     // Ejection pool
        //-------------------------------------------------------------------------------------------------------
        solution original=sol;
        vector<vector<node>>& R=sol.route_set;
        int r=rand()%R.size();                                //randomly select a route
        for(auto i=R[r].begin()+1; i!=R[r].end()-1; ++i)      //put the removed customers into EP
            EP.push_back(*i);
        R.erase(R.begin()+r);
        sol.setCost(Dist_table,custList,Vcapacity,alpha);
        vector<int> pcnt(NumOfCust+1,1);                      // initialize penalty counters
        //-------------------------------------------------------------------------------------------------------
        double ntime=0;
        chrono::steady_clock::time_point  start, end;
        start = chrono::steady_clock::now();
        while(!EP.empty() && ntime<Mtime)
        {
            bool sign = false;
            node v = EP.back();
            EP.pop_back();
            //---------------------------------------------------------------------------------
            //Random insert
            vector<solution> feasible;
            vector<pair<object,solution>> infeasible;
            randominsert(v,sol,feasible,infeasible);
            if(!feasible.empty())
            {
                int r=rand()%feasible.size();
                sol=feasible[r];
                sign=true;
            }
            else        //Squeeze
            {
                double penalty = minS.Fp;
                while(penalty!=0)
                {
                    solution orig = minS;
                    //random select an infeasible route
                    vector<vector<node>>& arr = minS.route_set;
                    vector<int> infeasible_route;
                    for(unsigned int i=0; i<arr.size(); ++i) //record all infeasible routes
                    {
                        double ptw=arr[i].back().twf;
                        double pc=(arr[i].back().cap>Vcapacity)?(arr[i].back().cap-Vcapacity):0;
                        if((ptw+pc)!=0)
                            infeasible_route.push_back(i);
                    }
                    int r=rand()%infeasible_route.size();   //select an infeasible route
                    int selected=infeasible_route[r];
                    squeeze(minS,selected);
                    if(minS.Fp < penalty)
                        penalty=minS.Fp;
                    else
                    {
                        minS=orig;
                        break;
                    }
                }
                if(minS.Fp!=0)
                {
                    sign=false;
                    if(minS.Pc<minS.Ptw)
                        alpha/=0.99;
                    if(minS.Pc>minS.Ptw)
                        alpha*=0.99;
                }
                else
                {
                    sol=minS;
                    sign=true;
                }
            }
            //---------------------------------------------------------------------------------
            if(!sign)            //if customer is not included in the solution
            {
                int Pbest=INT_MAX;
                pair<int,vector<node>> best;
                vector<node> eject;
                ++pcnt[v.NO];       //increase the penalty counter
                int cnt=0;
                for(unsigned int i=0; i<sol.route_set.size(); ++i)          //calculate all possible position
                {
//                    int rs=sol.route_set[i].size()-1;
//                    vector<vector<int>> ej;
//                    getAllSubsets(ej,rs,kmax);
                    for(unsigned int j=1; j<sol.route_set[i].size(); ++j)
                    {
                        vector<node> &arr=infeasible[cnt].second.route_set[i];  //infeasible route
                        //-----------------------------------------------------------------------------------
                        Allsubset(arr,eject,Pbest,best,i,pcnt);
                        //-----------------------------------------------------------------------------------
//                          int origCap=arr.back().cap;
//                        for(unsigned int k=1; k<ej.size(); ++k)
//                        {
//                            int tail=ej[k].size();
//                            int Psum=0;
//                            int Cap=0;
//                            for(int l=0; l<tail; ++l)
//                            {
//                                int p=ej[k][l];
//                                Psum+=pcnt[arr[p].NO];
//                                Cap+=custList[arr[p].NO].Demand;
//                            }
//                            //---------------------------------------------------------------------
//                            if(Psum<Pbest)  //continue to check feasibility
//                            {
//                                vector<node> tmp((int)arr.size()-tail);
//                                int cnt2=0;
//                                for(int l=0,m=0; l<(int)arr.size(); ++l)
//                                {
//                                    if(l!=ej[k][cnt2])
//                                    {
//                                        tmp[m]=arr[l];
//                                        ++m;
//                                    }
//                                    else
//                                        cnt2=((cnt2+1)==tail)?0:cnt2+1;
//                                }
//                                int newC=origCap-Cap;
//                                if(check(tmp)&&(newC<=Vcapacity))
//                                {
//                                    Pbest=Psum;
//                                    best=make_pair(i,tmp);   //record the position
//                                    vector<node> ejpool(tail);
//                                    for(int l=0; l<tail; ++l)
//                                    {
//                                        int p=ej[k][l];
//                                        ejpool[l]=arr[p];
//                                    }
//                                    eject=ejpool;            //record the ejected customers
//                                }
//                            }
//                        }
                        ++cnt;
                    }
                }
                EP.insert(EP.end(),eject.begin(),eject.end());
                int p=best.first;
                sol.route_set[p]=best.second;
                sol.setCost(Dist_table,custList,Vcapacity,alpha);
                perturb(sol);
            }
            end = chrono::steady_clock::now();
            ntime=chrono::duration_cast<chrono::seconds>(end - start).count();
        }
        if(!EP.empty())
            sol=original;
        t2 = chrono::steady_clock::now();
        time=chrono::duration_cast<chrono::seconds>(t2 - t1).count();
    }
    cout << "---------- GB: " << sol.Nvehicle << "----------" << endl;
    //------------------------------------------------------------------------------------
    ofstream f_best_No;
    string c="result/"+input+"/bestNvehicle.txt";
    f_best_No.open(c);
    f_best_No << sol.route_set.size() <<  endl;
    f_best_No.close();

    ofstream f_best_R;
    string d="result/"+input+"/bestRoute.txt";
    f_best_R.open(d);
    for(unsigned int i=0; i<sol.route_set.size(); ++i)
    {
        for(unsigned int j=0; j<sol.route_set[i].size(); ++j)
            f_best_R << sol.route_set[i][j].NO << "\t";
        f_best_R << "\n";
    }
    f_best_R.close();
}


