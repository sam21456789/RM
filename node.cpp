#include "node.h"

node::node(){}

node::node(const int no)
{
    this->NO=no;
}

node::node(const int no, const double z)
{
    this->NO=no;
    this->ltArrival=z;
}
