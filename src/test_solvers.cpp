#include <trig_solvers.h>
#include <iostream>
#include <algorithm>

double randQ(double qmin, double qmax)
{
  return qmin + ((qmax-qmin)*rand())/RAND_MAX;
}

double randV()
{
  return -3 + (6.*rand())/RAND_MAX;
}



using namespace ecn;

std::vector<int> tests = {2,3,4,5,6,7,8};

void print_result(int i, std::vector<double> sol, double qi)
{
  if(std::count(tests.begin(), tests.end(), i))
  {
    std::cout << "   Type " << i << ": " << sol.size() << " solution";
    for(auto qs: sol)
      if(isNull(qs-qi))
        std::cout << " / ok";
    std::cout << '\n';
  }

}

void print_result(int i, std::vector<JointSolution> sol, double qi, double qj)
{
  if(std::count(tests.begin(), tests.end(), i))
  {
    std::cout << "   Type " << i << ": " << sol.size() << " solution";
    for(auto qs: sol)
      if(isNull(qs.qi-qi) && isNull(qs.qj-qj))
        std::cout << " / ok";
    std::cout << '\n';
  }

}

int main()
{
  srand (clock());
  double x1, y1, x2, y2, z1, z2, w1, w2;

  for(int i = 0; i < 100; ++i)
  {
    std::cout << "------ Iter " << i+1 << " ------" << std::endl;
    // random joint limits
    const double qimin = randQ(-M_PI, 0);
    const double qimax = qimin + randQ(M_PI/4, M_PI);
    const double qjmin = randQ(-M_PI, 0);
    const double qjmax = qjmin + randQ(M_PI/4, M_PI);
    const double qi = randQ(qimin, qimax);
    const double qj = randQ(qjmin, qjmax);
    const double ci = cos(qi);
    const double si = sin(qi);
    const double cj = cos(qj);
    const double sj = sin(qj);

    // type2
    x1 = randV();
    y1 = randV();
    z1 = x1*si + y1*ci;
    print_result(2, solveType2(x1, y1, z1), qi);

    // type3

    x1 = randV();
    y1 = 0*randV();
    x2 = 0*randV();
    y2 = randV();
    z1 = x1*si + y1*ci;
    z2 = x2*si + y2*ci;
    print_result(3, solveType3(x1, y1, z1, x2, y2, z2), qi);

    // type 4
    x1 = randV();
    y1 = x1*qj*si;
    x2 = randV();
    y2 = x2*qj*ci;
    print_result(4, solveType4(x1, y1, x2, y2), qi, qj);


    // type 5
    x1 = randV();
    z1 = randV();
    x2 = randV();
    z2 = randV();
    y1 = x1*si-z1*qj;
    y2 = x2*ci-z2*qj;
    print_result(5, solveType5(x1, y1, z1, x2, y2, z2), qi, qj);


    // type 6
    x1 = randV();
    y1 = randV();
    w1 = randV();
    z1 = w1*sj-x1*ci-y1*si;
    z2 = w1*cj-x1*si+y1*ci;
    print_result(6, solveType6(x1, y1, z1, z2, w1), qi, qj);

    // type 7
    x1 = randV();
    y1 = randV();
    w1 = randV();
    w2 = randV();
    z1 = w1*cj+w2*sj-x1*ci-y1*si;
    z2 = w1*sj-w2*cj-x1*si+y1*ci;
    print_result(7, solveType7(x1, y1, z1, z2, w1, w2), qi, qj);

    // type 8
    x1 = randV();
    y1 = randV();
    z1 = x1*ci + y1*cos(qi+qj);
    z2 = x1*si + y1*sin(qi+qj);
    print_result(8, solveType8(x1, y1, z1, z2), qi, qj);
  }

  x1 = 0.3;
  y1 = 0.2;
  z1 = -0.1;
  z2 = -0.4;
  for(auto q: solveType8(x1, y1, z1, z2))
    std::cout << "q1: " << q.qi << ", q2 = " << q.qj << std::endl;









}
