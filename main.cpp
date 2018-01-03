#ifdef _WIN32
#undef __STRICT_ANSI__
#endif

///Kalman filter example
///Adapted from merge from www.adrianboeing.com and http://greg.czerniak.info/guides/kalman1
///following
/// * Simon, Dan. Kalman Filtering. Embedded Systems Programming. June 2001.

#include <iostream>
#include <vector>
#include <boost/shared_ptr.hpp>
#include <boost/numeric/ublas/io.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include "kalmanfilter.h"
#include "matrix.h"
#include "whitenoisesystem.h"

///Context:
///A car that has a constant acceleration that has its position determined by GPS
///The car its speedometer is not used (as observation(1,1) is equal to 0.0),
///  and gives junk values (as x_real_noise(1) is equal to 10000000.0)
int main()
{
  Matrix::Test();

  using boost::numeric::ublas::matrix;
  using boost::numeric::ublas::vector;

  const double t = 0.1;
  const double acceleration = 1.0;

  //The real state vector
  //[ position ]
  //[ velocity ]
  const vector<double> x_real = Matrix::CreateVector( { 0.0, 0.0 } );

  //Real measurement noise
  //[ standard deviation of noise in position ]   [ standard deviation of noise in GPS                       ]
  //[ standard deviation of noise in velocity ] = [ standard deviation of noise in defect/unused speedometer ]
  const vector<double> x_real_measurement_noise = Matrix::CreateVector( { 10.0, 10000000.0 } );

  //Guess of the state matrix
  //Position and velocity guess is way off on purpose
  //[ position ]
  //[ velocity ]
  const vector<double> x_first_guess = Matrix::CreateVector( { 100.0, 10.0 } );

  //Guess of the covariances
  //[ 1.0   0.0 ]
  //[ 0.0   1.0 ]
  const matrix<double> p_first_guess = Matrix::CreateMatrix(2,2, { 1.0, 0.0, 0.0, 1.0 } );

  //Effect of inputs on state
  //Input = gas pedal, which gives acceleration 'a'
  //[ 1.0   0.5 * t * t ]   [teleportation (not used)                 x = 0.5 * a * t * t ]
  //[ 0.0   t           ] = [no effect of teleportation on velocity   v = a * t           ]
  const matrix<double> control = Matrix::CreateMatrix(2,2, { 1.0, 0.0, 0.5 * t * t, t } );

  //Estimated measurement noise
  //[ 10.0          0.0 ]   [ Estimated noise in GPS   ?                                                     ]
  //[  0.0   10000000.0 ] = [ ?                        Estimated noise in speedometer (absent in this setup) ]
  const matrix<double> measurement_noise = Matrix::CreateMatrix(2,2, { 10.0, 0.0, 0.0, 10000000.0 } );

  //Observational matrix
  //[ 1.0   0.0 ]   [GPS measurement   ?                                         ]
  //[ 0.0   0.0 ] = [?                 Speedometer (absent/unused in this setup) ]
  const matrix<double> observation = Matrix::CreateMatrix(2,2, { 1.0, 0.0, 0.0, 0.0 } );

  //Real process noise
  //[ 0.001 ]   [ noise in position ]
  //[ 0.001 ] = [ noise in velocity ]
  const vector<double> real_process_noise = Matrix::CreateVector( {0.01,0.01} );

  //Estimated process noise covariance
  //[ 0.01   0.01 ]
  //[ 0.01   0.01 ]
  const matrix<double> process_noise = Matrix::CreateMatrix(2,2,{0.01,0.01,0.01,0.01});

  //State transition matrix, the effect of the current state on the next
  //[ 1.0     t ]   [ position keeps its value             a velocity changes the position ]
  //[ 0.0   1.0 ] = [ position has no effect on velocity   a velocity keeps its value      ]
  const matrix<double> state_transition = Matrix::CreateMatrix(2,2,{1.0,0.0,t,1.0});

  WhiteNoiseSystem s(control,x_real,x_real_measurement_noise,real_process_noise,state_transition);

  KalmanFilter k(control,x_first_guess,p_first_guess,measurement_noise,observation,process_noise,state_transition);

  std::cout << "x_real,x_measured,x_Kalman,v_real,v_measured,v_Kalman\n";
  for (int i=0;i!=250;++i)
  {
    //A constant push the gas pedal, which results in a constant acceleration
    const vector<double> input = Matrix::CreateVector( { 0.0, acceleration } );
    //Update reality, that is, let the real system (i.e. reality) go to its next state
    s.GoToNextState(input);
    //Perform a noisy measurement
    const vector<double> z_measured = s.Measure();
    //Pass this measurement to the filter
    k.SupplyMeasurementAndInput(z_measured,input);
    //Display what the filter predicts
    const vector<double> x_est_last = k.Predict();

    std::cout << s.PeekAtRealState()(0) << "," << z_measured(0) << "," << x_est_last(0) << "," << s.PeekAtRealState()(1) << "," << z_measured(1) << "," << x_est_last(1) << '\n';
  }
}
