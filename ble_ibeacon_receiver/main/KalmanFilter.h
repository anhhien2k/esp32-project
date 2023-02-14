#ifndef SimpleKalmanFilter_h
#define SimpleKalmanFilter_h

//void SimpleKalmanFilter(float mea_e, float est_e, float q);
//float updateEstimate(float mea);
//void setMeasurementError(float mea_e);
//void setEstimateError(float est_e);
//void setProcessNoise(float q);
//float getKalmanGain();
//float getEstimateError();

typedef struct{
	float _err_measure;
	float _err_estimate;
	float _q;
	float _current_estimate;
	float _last_estimate;
	float _kalman_gain;
}KalmanVariable_t;

typedef struct{
	KalmanVariable_t KalmanVariable;
    void  (*SimpleKalmanFilter_func) (KalmanVariable_t *temp, float mea_e, float est_e, float q);
    float (*updateEstimate_func) (KalmanVariable_t *temp, float mea);
    void  (*setMeasurementError_func) (KalmanVariable_t *temp, float mea_e);
    void  (*setEstimateError_func)(KalmanVariable_t *temp, float est_e);
    void  (*setProcessNoise_func)(KalmanVariable_t *temp, float q);
    float (*getKalmanGain_func)(KalmanVariable_t *temp);
    float (*getEstimateError_func)(KalmanVariable_t *temp);
    void  (*displayParameter_func)(KalmanVariable_t *temp);
}KalmanFunc_t;

void SimpleKalmanFilter(KalmanVariable_t *temp, float mea_e, float est_e, float q);
float updateEstimate(KalmanVariable_t *temp, float mea);
void setMeasurementError(KalmanVariable_t *temp, float mea_e);
void setEstimateError(KalmanVariable_t *temp, float est_e);
void setProcessNoise(KalmanVariable_t *temp, float q) ;
float getKalmanGain(KalmanVariable_t *temp) ;
float getEstimateError(KalmanVariable_t *temp);
void displayParameter(KalmanVariable_t *temp);

char *hex2str(uint8_t *data, uint8_t len);
#endif
