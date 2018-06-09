#ifndef OBJECT_H_INCLUDED
#define OBJECT_H_INCLUDED

class object
{
    public:

        double Fp=0;
        int position1=0;
        int position2=0;

    public:

        object(int p1,int p2,double fp);

    public:

        bool operator<(const object& O) const;
        bool operator>(const object& O) const;
        bool operator==(const object& O) const;
        bool operator!=(const object& O) const;

};

#endif // object_H_INCLUDED
