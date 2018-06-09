#ifndef NODE_H_INCLUDED
#define NODE_H_INCLUDED

class node
{
    public:
        int NO=0;               //customer number
        double Arrival=0;       //earliest arrival time
        double ltArrival=0;     //latest start time
        double twf=0;           //forward stimulated violation
        double twb=0;           //backward stimulated violation
        double cap=0;

    public:
        node();
        node(const int no);
        node(const int no,const double z);
};

#endif // NODE_H_INCLUDED
