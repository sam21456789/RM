#ifndef SOLUTION_H_INCLUDED
#define SOLUTION_H_INCLUDED

#include <cfloat>
#include <climits>
#include <vector>
#include "node.h"
#include "customer.h"

using std::vector;

class solution
{
    public:

        int Nvehicle=INT_MAX;
        double Fp=DBL_MAX;
        vector<vector<node>> route_set;
        double Pc=0;
        double Ptw=0;

    public:

        bool operator<(const solution& S) const;
        bool operator>(const solution& S) const;
        bool operator==(const solution& S) const;
        bool operator!=(const solution& S) const;

    public:

        void setCost(vector<vector<double>>& Dist_table,vector<customer>& data,int Mcapacity,double alpha);
        void setCost(vector<vector<double>>& Dist_table,vector<customer>& data,int Mcapacity,double alpha,int route);
        void print();
};

#endif // SOLUTION_H_INCLUDED
