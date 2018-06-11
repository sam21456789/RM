#ifndef RM_H_INCLUDED
#define RM_H_INCLUDED

#include <vector>
#include <string>
#include "customer.h"
#include "node.h"
#include "solution.h"
#include "object.h"

using namespace std;

class rm
{
    private:
        int maxVehicle;
        int Vcapacity;
        string input;
        vector<customer> custList;              //customer list (about data)
        vector<vector<double>> Dist_table;      //Distance table
        int NumOfCust=0;
        double Mtime;
        double Irand=1000;
        double alpha=1.0;
        int kmax=5;
        vector<vector<int>> Nlist;          //Neighbor list

    public:

        rm(const string inputdata,const double maxtime);

        bool check(vector<node>& route);

        void randominsert(node& cust, solution& s, vector<solution>& feasible_set, vector<pair<object,solution>>& infeasible_set);

        void squeeze(solution& s,int& position);

        solution t_opt(solution& s,int& position);

        solution intra_relocation(solution& s,int& position);

        solution inter_relocation(solution& s,int& position);

        solution intra_exchange(solution& s,int& position);

        solution inter_exchange(solution& s,int& position);

        solution t_opt(solution& s);

        solution intra_relocation(solution& s);

        solution inter_relocation(solution& s);

        solution intra_exchange(solution& s);

        solution inter_exchange(solution& s);

        void perturb(solution& s);

        void exe();

        void Initialize(solution& s);
};

#endif // RM_H_INCLUDED
