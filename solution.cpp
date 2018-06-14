#include <cmath>
#include <utility>
#include <algorithm>
#include <iostream>
#include <iomanip>
#include "solution.h"

#define rsize 200

using namespace std;

bool solution::operator<(const solution& S) const
{
    return (this->Fp < S.Fp);
}

bool solution::operator>(const solution& S) const
{
    return (this->Fp > S.Fp);
}

bool solution::operator==(const solution& S) const
{
    return (this->Fp ==S.Fp);
}

bool solution::operator!=(const solution& S) const
{
    return (this->Fp != S.Fp);
}

void solution::setCost(vector<vector<double>>& Dist_table,vector<customer>& data,int Mcapacity,double alpha)
{
    double _Pc=0;    //capacity penalty
    double _Ptw=0;   //time window penalty

    for(auto i=route_set.begin(); i!=route_set.end();)
    {
        if(i->size()==2)
            i=route_set.erase(i);
        else
        {
            int c=0;
            double Ptwf=0;  //forward penalty
            for(auto j=i->begin()+1; j!=i->end(); ++j)
            {
                int src= (j-1)->NO;
                int dest= j->NO;
                c+= data[dest].Demand;
                double a_j = (j-1)->Arrival + data[src].Tservice + Dist_table[src][dest];
                if(a_j <= data[dest].Tdue)      // a_j <= l_j
                    j->Arrival= max(a_j,data[dest].Tready);
                else
                {
                    j->Arrival= data[dest].Tdue;
                    Ptwf+= (a_j - data[dest].Tdue);
                }
                j->twf=Ptwf;     // forward penalty at customer j
                j->cap=c;
            }
            double Ptwb=0;  //backward penalty
            for(auto j=i->rbegin()+1; j!=i->rend()-1; ++j)
            {
                int src=(j-1)->NO;
                int dest=j->NO;
                double z_j= (j-1)->ltArrival-Dist_table[src][dest]-data[dest].Tservice;
                if(z_j >= data[dest].Tready)      // z_j >= e_j
                    j->ltArrival= min(z_j,data[dest].Tdue);
                else
                {
                    j->ltArrival= data[dest].Tready;
                    Ptwb+= (data[dest].Tready-z_j);
                }
                j->twb=Ptwb;
            }
            _Ptw+=Ptwf;
            _Pc+=(c>Mcapacity)?(c-Mcapacity):0;     //if greater than max capacity
            ++i;
        }
    }
    this->Pc=_Pc;
    this->Ptw=_Ptw;
    this->Fp=this->Pc+alpha*this->Ptw;
    this->Nvehicle=route_set.size();
}

void solution::setCost(vector<vector<double>>& Dist_table,vector<customer>& data,int Mcapacity,double alpha,int routeNum)
{
    double oriPc=(route_set[routeNum].back().cap>Mcapacity)?route_set[routeNum].back().cap-Mcapacity:0;
    double oriPtw=route_set[routeNum].back().twf;

    int c=0;
    double Ptwf=0;  //forward penalty
    for(auto i=route_set[routeNum].begin()+1; i!=route_set[routeNum].end(); ++i)
    {
        int src= (i-1)->NO;
        int dest= i->NO;
        c+= data[dest].Demand;
        double a_i = (i-1)->Arrival + data[src].Tservice + Dist_table[src][dest];
        if(a_i <= data[dest].Tdue)      // a_j <= l_j
            i->Arrival= max(a_i,data[dest].Tready);
        else
        {
            i->Arrival= data[dest].Tdue;
            Ptwf+= (a_i - data[dest].Tdue);
        }
        i->twf=Ptwf;     // forward penalty at customer j
        i->cap=c;
    }
    double Ptwb=0;  //backward penalty
    for(auto i=route_set[routeNum].rbegin()+1; i!=route_set[routeNum].rend(); ++i)
    {
        int src=(i-1)->NO;
        int dest=i->NO;
        double z_i= (i-1)->ltArrival-Dist_table[src][dest]-data[dest].Tservice;
        if(z_i >= data[dest].Tready)      // z_j >= e_j
            i->ltArrival= min(z_i,data[dest].Tdue);
        else
        {
            i->ltArrival= data[dest].Tready;
            Ptwb+= (data[dest].Tready-z_i);
        }
        i->twb=Ptwb;
    }
    double newPc=(c>Mcapacity)?c-Mcapacity:0;
    double newPtw=Ptwf;


    this->Pc-=(newPc-oriPc);
    this->Ptw-=(newPtw-oriPtw);
    this->Fp=this->Pc+alpha*this->Ptw;
    this->Nvehicle=route_set.size();
}

void solution::print()
{
    vector<int> exist(rsize,0);
    int c=0;
    for(auto i=route_set.begin(); i!=route_set.end(); ++i)
    {
        for(auto j=i->begin(); j!=i->end(); ++j)
        {
            cout << setw(4) << j->NO;
            if(j->NO!=0)
            {
                ++exist[(j->NO)-1];
                ++c;
            }
        }
        cout << endl;
    }
    cout << c << endl;
    cout << "Not in: ";
    for(unsigned int i=0; i<rsize; ++i)
    {
        if(exist[i]==0)
            cout << setw(4)<< i+1;
    }
    cout <<  endl;
    cout << "Duplicated: ";
    for(unsigned int i=0; i<rsize; ++i)
    {
        if(exist[i]>=2)
            cout << setw(4)<< i+1;
    }
    cout << endl;
    cout << endl;
}
