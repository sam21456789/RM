#include "customer.h"
#include <iostream>

using std::cout;
using std::endl;

customer::customer(const int no,const double x,const double y,const int demand,const double startT,const double endT,const double serviceT)
{
    this->NO = no;
    this->X = x;
    this->Y = y;
    this->Demand = demand;
    this->Tready = startT;
    this->Tdue = endT;
    this->Tservice = serviceT;
}

void customer::print()
{
    cout << " NO." << this->NO << endl;
    cout << " Coordinate: ( " << this->X << " , " << Y << " )" << endl;
    cout << " Demand: " << this->Demand << endl;
    cout << " Time Window: [ " << this->Tready << " , " << Tdue << " ] " << endl;
    cout << " Service Time: " << this->Tservice << endl;
    cout << endl;
}
