/* 
    One dimensional Kalman filter inspired by the intuitive explanations provided here: https://www.kalmanfilter.net/default.aspx
    and implementation inspired by: https://github.com/bachagas/Kalman
*/

class KalmanFilter {

    public:
        KalmanFilter(double initial_prediction, double measurement_uncertainty, double process_noise_covariance, double estimated_error_coveriance){
            this->x = initial_prediction; 
            this->r = measurement_uncertainty;
            this->q = process_noise_covariance;
            this->p = estimated_error_coveriance;
        }
        KalmanFilter(){

        }

        double kalmanIteration(double measurement){

            this->p = covarianceExtrapolation(this->p, this->q);

            this->k = kalmanGain(this->p, this->r);

            this->x = stateUpdate(this->x, this->k, measurement);

            this->p = coverianceUpdate(this-> k, this->p);

            return this->x;
        }

        double covarianceExtrapolation(double process_noise_covariance, double estimated_error_coveriance){
            // Update the extrapolated estimate uncertainty -- p = p + q 
            return estimated_error_coveriance + process_noise_covariance; 
        }

        double kalmanGain(double estimated_error_coveriance, double measurement_uncertainty){
            // Kalman Gain calculation -- k = p / p + r
            return estimated_error_coveriance / (estimated_error_coveriance + measurement_uncertainty);
        }

        double stateUpdate(double current_state, double kalman_gain, double measurment){
            // Estimate the state -- x = x + k (z - x)
            return current_state + (kalman_gain * ( measurment - current_state));
        }

        double coverianceUpdate(double kalmain_gain, double estimated_error_coveriance ){
            // Update estimate uncertainty -- p = (1 - k) p
            return (1 - kalmain_gain) * estimated_error_coveriance;
        }

        double getKalmanGain(){
            return this->k;
        }

        double getEstimationErrorCovariance(){
            return this->p;
        }

        double getProcessNoiseCoveriance(){
            return this->q;
        }

        double getMeasurementNoiseCoveriance(){
            return this->r;
        }




    private:
        /* Kalman filter variables */
        double q; // Process noise covariance
        double r; // measurement noise covariance
        double x; // value
        double p; // estimation error covariance
        double k; // Kalman gain
};