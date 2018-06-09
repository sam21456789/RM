#ifndef CUSTOMER_H_INCLUDED
#define CUSTOMER_H_INCLUDED

class customer
{
    public:

        int NO=0;
        double X=0;
        double Y=0;
        int Demand=0;
        double Tready=0;
        double Tdue=0;
        double Tservice=0;

    public:

        customer(const int no,const double x,const double y,const int demand,const double startT,const double endT,const double serviceT);

        void print();
};

#endif // CUSTOMER_H_INCLUDED
