#ifndef TRIPLE_HPP
#define TRIPLE_HPP

#include <string>
#include <tuple>
#include <math.h>
#include <iostream>

// To define the 3x1 vector and the 3x3 matrix in our code first
using triple = std::tuple<double, double, double>;
using triple_matrix = std::tuple<triple,triple,triple>;

// For the subtraction of vectors
triple operator-(const triple& p_end, const triple& p_start);

// For the addition of vectors
triple operator+(const triple& p_end, const triple& p_start);

// |V| = sqrt(x*x + y*y + normal_vector_hat*normal_vector_hat)
// To calculate the magnitude of a vector 
double triple_norm(triple p);

// To calculate the cross product of 2 vectors
triple triple_cross_product(triple vector_one, triple vector_two);

// For the division of a vector by a number
triple operator/(const triple& tr, const double num);

// For the multiplication of a vector by a number or matrix by vector
triple operator*(const triple& tr, const double num);
triple operator*(const triple_matrix& rotational_matrix, const triple& p_start);


// To decide if the numbers of one vector are all bigger than the corresponding numbers of another vector 
bool operator>(const triple& p_start, const triple& p_end);


void print_triple(triple tr, std::string str = "");
#endif // TRIPLE_HTP


