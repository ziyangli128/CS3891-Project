#include <boost/python.hpp> 
#include <rtt_context.cpp>
using namespace boost::python; 
 
BOOST_PYTHON_MODULE(python_c) 
{ 

    def("rrt", rrt); 
    def("greet", greet); 
} 