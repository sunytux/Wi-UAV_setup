/* System Headers */
#include <boost/algorithm/string.hpp>
#include <boost/format.hpp>
#include <boost/program_options.hpp>
#include <boost/thread.hpp>
#include <cmath>
#include <complex>
#include <csignal>
#include <fstream>
#include <iostream>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <mutex>

/* UHD Library Headers */
#include <uhd/exception.hpp>
#include <uhd/types/tune_request.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread_priority.hpp>

/* mraa headers */
#include "mraa/common.hpp"
#include "mraa/gpio.hpp"

#include "radio.hpp"
#include "switch.hpp"
#include "utils.hpp"
#include "config.hpp"
static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}
float avg;
extern bool endFlag;
float measureFromRadioF1;
float measureFromRadioF2;
Radio_data_s radioMeasure;
extern int switchState;
bool current_freq = 0;
bool usrpInitializedFlag = false;

float FREQ_USERS[2] = {FREQ_USER1, FREQ_USER2};

std::mutex radioMutex;

int RadioRXStream1()
{
    double seconds_in_future = 1.5;

    /* USRP Setup */
    uhd::usrp::multi_usrp::sptr usrp = initUsrp(" ");

    /* Stream initialisation */
    uhd::stream_args_t stream_args("fc32"); // complex floats
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = MAX_RECORDED_SAMPLES;
    stream_cmd.stream_now = false;

    std::vector<std::complex<float>> buff(rx_stream->get_max_num_samps());

    /* Switch Setup */
    initSwitch();

    while (!endFlag) {
        for (int userIdx = 0; userIdx < N_USERS; ++userIdx) {
            /* Adapt user frequency */
            uhd::tune_request_t tune_request(FREQ_USERS[userIdx]);
            usrp->set_rx_freq(tune_request);

            for (int antIdx = 0; antIdx < N_ANTENNAS; ++antIdx) {
               double  now = usrp->get_time_now().get_real_secs();
                if (now > seconds_in_future)
                {
                    printf("MEASURE_PERIOD is too small! Difference is %f\n",
                           now - seconds_in_future);
                    seconds_in_future = now + MEASURE_PERIOD;
                }
                stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
                rx_stream->issue_stream_cmd(stream_cmd);

                float avg = 0.0f;
                size_t accumulatedSamples = 0u;

                while (accumulatedSamples < MAX_RECORDED_SAMPLES) {
                    uhd::rx_metadata_t md;
                    size_t receivedSamples = rx_stream->recv(
                        &buff.front(), buff.size(), md, MEASURE_PERIOD, true);

                    if (receivedSamples > 0u) {
                        for (int j = 0u; j < buff.size(); j++) {
                            avg = avg + abs(buff[j]);
                        }
                        avg = avg / buff.size();
                        
                        radioMutex.lock();
                        radioMeasure.average = avg;
                        radioMeasure.switchState = switchState;
                        radioMeasure.user = userIdx;
                        radioMutex.unlock();

                        /* Error handling */
                        if (md.error_code
                            != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                            throw std::runtime_error(
                                str(boost::format("Receiver error %s")
                                    % md.strerror()));
                        }

                        accumulatedSamples += receivedSamples;
                    }
                }

                seconds_in_future = seconds_in_future + MEASURE_PERIOD;

                switchNextAntenna();
            }
        }
    }
}

Radio_data_s getCurrentRadioMeasure(){
    Radio_data_s radioMeasure_cpy;

    radioMutex.lock();

    radioMeasure_cpy.average = radioMeasure.average;
    radioMeasure_cpy.switchState = radioMeasure.switchState;
    radioMeasure_cpy.user = radioMeasure.user;

    radioMutex.unlock();
    
    return radioMeasure_cpy;
}

void scanAllUsers(std::vector<User_rss_s>& usersRss)
{
    for (int i = 0; i < N_USERS * N_ANTENNAS; i++) {
        Radio_data_s rMeasure = getCurrentRadioMeasure();

        usersRss[rMeasure.user].user = rMeasure.user;
        usersRss[rMeasure.user].rss[rMeasure.switchState] =
            rMeasure.average;

        usleep(MEASURE_PERIOD * S);
    }
}

uhd::usrp::multi_usrp::sptr initUsrp(std::string addr)
{
    /* Set the scheduling priority on the current thread */
    uhd::set_thread_priority_safe();

    /* Creating a USRP Object */
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(addr);
    usrp->set_rx_rate(RX_DAC_RATE);
    usrp->set_time_now(uhd::time_spec_t(0.0));
    uhd::tune_request_t tune_request(FREQ_USER1);
    usrp->set_rx_freq(tune_request);
    usrp->set_rx_bandwidth(BANDWIDTH);
    usrp->set_rx_antenna(SDR_ANTENNA);
    usrp->set_clock_source(SDR_CLOCK_SOURCE);
    usrp->set_rx_gain(RX_GAIN);

    usrpInitializedFlag = true;

    return usrp;
}
