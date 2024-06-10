#include "helpers/triple.hpp"

// To define the 3x1 vector and the 3x3 matrix in our code first
using triple = std::tuple<double, double, double>;
using triple_matrix = std::tuple<triple,triple,triple>;

// For the subtraction of vectors
triple operator-(const triple& p_end, const triple& p_start) {
    return std::make_tuple(
        std::get<0>(p_end) - std::get<0>(p_start),
        std::get<1>(p_end) - std::get<1>(p_start),
        std::get<2>(p_end) - std::get<2>(p_start)
    );
};

// For the addition of vectors
triple operator+(const triple& p_end, const triple& p_start) {
    return std::make_tuple(
        std::get<0>(p_end) + std::get<0>(p_start),
        std::get<1>(p_end) + std::get<1>(p_start),
        std::get<2>(p_end) + std::get<2>(p_start)
    );
};

// |V| = sqrt(x*x + y*y + normal_vector_hat*normal_vector_hat)
// To calculate the magnitude of a vector 
double triple_norm(triple p) {
    return std::sqrt(
        std::get<0>(p) * std::get<0>(p) + 
        std::get<1>(p) * std::get<1>(p) + 
        std::get<2>(p) * std::get<2>(p)
    );
}


// To calculate the cross product of 2 vectors
triple triple_cross_product(triple vector_one, triple vector_two){

        return std::make_tuple(
        std::get<1>(vector_one) * std::get<2>(vector_two) - std::get<1>(vector_two) * std::get<2>(vector_one),
        std::get<2>(vector_one) * std::get<0>(vector_two) - std::get<0>(vector_one) * std::get<2>(vector_two),
        std::get<0>(vector_one) * std::get<1>(vector_two) - std::get<0>(vector_two) *  std::get<1>(vector_one)
    );
        
}

// For the division of a vector by a number
triple operator/(const triple& tr, const double num) {
    return std::make_tuple(
        std::get<0>(tr) / num,
        std::get<1>(tr) / num,
        std::get<2>(tr) / num
    );
}

// For the multiplication of a vector by a number
triple operator*(const triple& tr, const double num) {
    return std::make_tuple(
        std::get<0>(tr) * num,
        std::get<1>(tr) * num,
        std::get<2>(tr) * num
    );
}

    // To decide if the numbers of one vector are all bigger than the corresponding numbers of another vector 
bool operator>(const triple& p_start, const triple& p_end) {
    return std::get<0>(p_end) <= std::get<0>(p_start) &&
    std::get<1>(p_end) <= std::get<1>(p_start) &&
    std::get<2>(p_end) <= std::get<2>(p_start);
}

triple operator*(const triple_matrix& rotational_matrix, const triple& p_start){
    return
    (std::get<0>(rotational_matrix) * std::get<0>(p_start)) +
    (std::get<1>(rotational_matrix) * std::get<1>(p_start)) +
    (std::get<2>(rotational_matrix) * std::get<2>(p_start));
}

void print_triple(triple tr, std::string str /*=""*/) {
    std::cout << str << "Triple: x =" << std::get<0>(tr)
        << ", y = " << std::get<1>(tr) 
        << ", z = " << std::get<2>(tr) << "\n";
}