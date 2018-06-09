#include "object.h"

object::object(int p1,int p2,double fp)
{
    this->Fp=fp;
    this->position1=p1;
    this->position2=p2;
}

bool object::operator<(const object& O) const
{
    return (this->Fp < O.Fp);
}

bool object::operator>(const object& O) const
{
    return (this->Fp < O.Fp);
}

bool object::operator==(const object& O) const
{
    return (this->Fp == O.Fp);
}

bool object::operator!=(const object& O) const
{
    return (this->Fp != O.Fp);
}

