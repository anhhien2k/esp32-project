#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "math.h"
#include "KalmanFilter.h"

/*
//9, 1, 0.065
float _err_measure;
float _err_estimate;
float _q;
float _current_estimate = 0;
float _last_estimate = 0;
float _kalman_gain = 0;

void SimpleKalmanFilter(float mea_e, float est_e, float q)
{
  _err_measure=mea_e;
  _err_estimate=est_e;
  _q = q;
}

float updateEstimate(float mea)
{
  _kalman_gain = _err_estimate/(_err_estimate + _err_measure);//0.5
  _current_estimate = _last_estimate + _kalman_gain * (mea - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain)*_err_estimate + fabs(_last_estimate-_current_estimate)*_q;
  _last_estimate=_current_estimate;

  return _current_estimate;
}

void setMeasurementError(float mea_e)
{
  _err_measure=mea_e;
}

void setEstimateError(float est_e)
{
  _err_estimate=est_e;
}

void setProcessNoise(float q)
{
  _q=q;
}

float getKalmanGain() {
  return _kalman_gain;
}

float getEstimateError() {
  return _err_estimate;
}
*/

void SimpleKalmanFilter(KalmanVariable_t *temp, float mea_e, float est_e, float q){
  temp->_err_measure = mea_e;
  temp->_err_estimate = est_e;
  temp->_q = q;
  temp->_current_estimate = 0;
  temp->_last_estimate = 0;
  temp->_kalman_gain = 0;
}

float updateEstimate(KalmanVariable_t *temp, float mea){
  temp->_kalman_gain = temp->_err_estimate/(temp->_err_estimate + temp->_err_measure);
  temp->_current_estimate = temp->_last_estimate + temp->_kalman_gain * (mea - temp->_last_estimate);
  temp->_err_estimate =  (1.0 - temp->_kalman_gain)*temp->_err_estimate + fabs(temp->_last_estimate - temp->_current_estimate)*temp->_q;
  temp->_last_estimate = temp->_current_estimate;

  return temp->_current_estimate;
}

void setMeasurementError(KalmanVariable_t *temp, float mea_e){
	temp->_err_measure = mea_e;
}

void setEstimateError(KalmanVariable_t *temp, float est_e) {
	temp->_err_estimate = est_e;
}

void setProcessNoise(KalmanVariable_t *temp, float q) {
	temp->_q = q;
}

float getKalmanGain(KalmanVariable_t *temp) {
  return temp->_kalman_gain;
}

float getEstimateError(KalmanVariable_t *temp) {
  return temp->_err_estimate;
}

void displayParameter(KalmanVariable_t *temp){
	printf("\n Parameter: _err_measure: %.03f, _err_estimate: %.03f, _q: %0.3f\n", temp->_err_measure, temp->_err_estimate, temp->_q);
}

char *hex2str(uint8_t *data, uint8_t len){
    static char str[101];
    const char hex[] = "0123456789abcdef";
    if (len > 50) {
        len = 50;
    }
    if (data  == NULL) {
//    	ESP_LOGE(__FUNCTION__, "NULL pointer");
    	printf("\nhex2str: NULL pointer");
        return str;
    }
    for (uint8_t i = 0; i < len; i++)
    {
        str[i * 2] = hex[data[i] >> 4];
        str[i * 2 + 1] = hex[data[i] & 0x0F];
    }
    str[len * 2] = 0;
    return str;
}
