#include "edge.h"

edge::edge(){}

edge::edge(int src,int dest)
{
    this->source=src;
    this->destination=dest;
}
edge::edge(const edge& E)
{
    this->source=E.source;
    this->destination=E.destination;
}
edge& edge::operator=(const edge& E)
{
    this->source=E.source;
    this->destination=E.destination;
    return *this;
}

edge::~edge(){};
