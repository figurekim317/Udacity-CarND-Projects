#include <iostream>
#include <Eigen/Dense>


using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;

int main()
{
    MatrixXd m;

    cout << &m << endl;
    cout << m << endl;
    
    // cout << &MatrixXd(4, 4) <<endl;

    m = MatrixXd(4, 4);
    m << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;

    cout << &m << endl;
    
    
    MatrixXd c = MatrixXd(1, 1);
    cout << &c << endl;

    c = m;

    cout << &c << endl;
}
