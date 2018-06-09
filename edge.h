#ifndef EDGE_H_INCLUDED
#define EDGE_H_INCLUDED

class edge
{
    public:

        int source=0;
		int destination=0;

		edge();
        edge(int src,int dest);
        edge(const edge& E);
        edge& operator=(const edge& E);
        ~edge();
};

#endif // EDGE_H_INCLUDED
