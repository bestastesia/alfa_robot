#include <alfa_robot_moveit_config/comfort_height.hpp>
#include <cassert>
#include <cmath>
#include <limits>
#include <random>

using alfa_robot::motion::chooseComfortHeight;
int main()
{
  auto close = [](double a, double b) { assert(std::abs(a-b) < 1e-8); };
  auto q = chooseComfortHeight(1, .4, .6, 1, 0, -1, 0, .8, .8, .8);
  close(q.position, -.6 + std::sqrt(.28));  // exact above root; below root projects onto the lower limit
  close(q.ratio, .8);
  q = chooseComfortHeight(1, 1.5, .6, 1, -1, -1, 0, .7, .7, .7);
  close(q.position, -.5 + std::sqrt(.13)); // both roots legal; above beats shorter below travel
  q = chooseComfortHeight(1, .4, .6, 1, 0, -1, 0, .8, .8, .8, "below");
  close(q.position, -1); assert(q.projected);
  q = chooseComfortHeight(1, .4, 1.2, 1, 0, -1, 0, .7, .8, .9);
  close(q.position, -.6); close(q.ratio, 1.2);
  q = chooseComfortHeight(2, 0, 0, 1, 0, -1, 0, .7, .8, .9);
  close(q.position, -1); close(q.ratio, 1);
  q = chooseComfortHeight(1, .5, 0, 1, -.2, -1, 0, .3, .3, .3);
  close(q.position, -.4);  // nonzero initial lift
  q = chooseComfortHeight(1, .5, .4, 1, 0, -.5, -.5 + .5, .4, .4, .4);
  close(q.position, -.5);
  // x=.9, second-row boxes: constrain to home-connected lift clearance, not
  // just the mechanical -1m stop. Box6 stays unchanged; box7 uses nearest safe Z.
  q = chooseComfortHeight(1.744633752, .610, 1.031151421, .978999993,
                          0, -.990, 0, 1.10, 1.15, 1.15);
  close(q.position, -.682676949);
  q = chooseComfortHeight(1.744633752, .610, 1.13400935, .978999993,
                          0, -.990, 0, 1.10, 1.15, 1.15);
  close(q.position, -.990); assert(q.projected); assert(q.ratio > 1.15);
  q = chooseComfortHeight(1.744633752, .610, 1.13400935, .978999993,
                          0, 0, 0, 1.10, 1.15, 1.15);
  close(q.position, 0);  // no lift travel available: still try at home
  for (int bad=0; bad<5; ++bad) {
    bool rejected = false;
    try {
      chooseComfortHeight(1, 0, .5, bad==0 ? 0 : 1, 0, -1, 0,
        bad==1 ? .9 : .7, bad==2 ? std::numeric_limits<double>::quiet_NaN() : .8,
        bad==3 ? .6 : .9, bad==4 ? "invalid" : "auto");
    } catch (const std::invalid_argument&) { rejected=true; }
    assert(rejected);
  }
  // Independently check the primary geometric optimum against a dense legal grid.
  std::mt19937 rng(42);
  std::uniform_real_distribution<double> u(0, 1);
  for (int i=0; i<200; ++i) {
    const double s=2*u(rng), t=2*u(rng), xy=1.5*u(rng), l=.5+u(rng);
    q=chooseComfortHeight(s,t,xy,l,0,-1,0,.7,.8,.9);
    assert(q.position>=-1 && q.position<=0);
    for (int j=0; j<=10000; ++j) {
      double r=std::hypot(xy,s-double(j)/10000-t)/l;
      assert(std::abs(q.ratio-.8)<=std::abs(r-.8)+1e-8);
    }
  }
}
