CXX := g++
CXXFLAGS := -g -Wall -std=c++11 -O3
SRCS := customer.cpp node.cpp object.cpp solution.cpp rm.cpp main.cpp
OBJS := $(SRCS:.cpp=.o)

all: main
	
main: $(OBJS)
	$(CXX) $(CXXFLAGS) -o rmh $(OBJS)
	
$(OBJS) : $(SRCS)

clean:
	rm -rf rmh *.o 

help:
	@echo "      =========================================="
	@echo "        If you want to execute this program."
	@echo "        You must follow certain form."
	@echo "        i.e. ./rmh [dataset] [runs] [time]"
	@echo "      =========================================="