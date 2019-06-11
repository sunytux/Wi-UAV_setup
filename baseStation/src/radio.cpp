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

static bool stop_signal_called = false;
void sig_int_handler(int)
{
    stop_signal_called = true;
}
float avg;
extern bool endFlag;
float measureFromRadioF1;
float measureFromRadioF2;
float measureFromRadio = 0;
extern int switchState;
bool current_freq = 0;
bool usrpInitializedFlag = false;

int RadioRXStream1()
{
    double seconds_in_future = 1.5;
    size_t MEASURE_SIZE = 1000u;
    float MEASURE_PERIOD = 0.05f; // @TODO put that in a define

    /* USRP Setup */
    uhd::usrp::multi_usrp::sptr usrp = initUsrp(" ");

    /* Stream initialisation */
    uhd::stream_args_t stream_args("fc32"); // complex floats
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    uhd::stream_cmd_t stream_cmd(
        uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
    stream_cmd.num_samps = MEASURE_SIZE;
    stream_cmd.stream_now = false;

    std::vector<std::complex<float>> buff(rx_stream->get_max_num_samps());

    /* Switch Setup */
    initSwitch();

    while (!endFlag) {
        stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
        rx_stream->issue_stream_cmd(stream_cmd);

        float avg = 0.0f;
        size_t accumulatedSamples = 0u;

        while (accumulatedSamples < MEASURE_SIZE) {
            uhd::rx_metadata_t md;
            size_t receivedSamples = rx_stream->recv(
                &buff.front(), buff.size(), md, MEASURE_PERIOD, true);

            if (receivedSamples > 0u) {
                for (int j = 0u; j < buff.size(); j++) {
                    avg = avg + abs(buff[j]);
                }
                avg = avg / buff.size();

                measureFromRadio = avg;

                /* Error handling */
                if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE) {
                    throw std::runtime_error(str(
                        boost::format("Receiver error %s") % md.strerror()));
                }

                accumulatedSamples += receivedSamples;
            }
        }

        seconds_in_future = seconds_in_future + MEASURE_PERIOD;

        switchNextAntenna();
    }
}

uhd::usrp::multi_usrp::sptr initUsrp(std::string addr)
{
    // Set the scheduling priority on the current thread
    uhd::set_thread_priority_safe();

    // Creating a USRP Object
    // Present in MP
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(addr);
    usrp->set_rx_rate(100e6 / 16);
    usrp->set_time_now(uhd::time_spec_t(0.0));
    uhd::tune_request_t tune_request(2e9);
    usrp->set_rx_freq(tune_request);
    usrp->set_rx_bandwidth(100e6);

    // Added compared to MP
    usrp->set_rx_antenna("TX/RX");
    usrp->set_clock_source("internal");
    /* @TODO Define a gain*/
    // usrp->set_rx_gain(RX_GAIN);

    usrpInitializedFlag = true;

    return usrp;
}
