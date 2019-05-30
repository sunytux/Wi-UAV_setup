/*
 * Command the USRP using UHD's libraries
 */ 

#include "radio.hpp"
#include <fstream>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>
#include <cmath>

/* mraa headers */
#include "mraa/common.hpp"
#include "mraa/gpio.hpp"

//UHD Library Headers
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/algorithm/string.hpp>
#include <complex>
#include <uhd/types/tune_request.hpp>
#include <uhd/utils/thread_priority.hpp>
#include <uhd/utils/safe_main.hpp>
#include <uhd/usrp/multi_usrp.hpp>
#include <uhd/exception.hpp>
#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>
#include <iostream>
#include <fstream>
#include <csignal>
#include <complex>

// defin GPIOs
#define V1_PIN 7    // Corresponds to R-pi GPIO4
#define V2_PIN 15   // Corresponds to R-pi GPIO22
#define V3_PIN 16   // Corresponds to R-pi PWM3

namespace po = boost::program_options;

static bool stop_signal_called = false;
void sig_int_handler(int){stop_signal_called = true;}
float avg;
extern bool endFlag;
mraa::Gpio gpio_1(V1_PIN); 
mraa::Gpio gpio_2(V2_PIN); 
mraa::Gpio gpio_3(V3_PIN); 
extern float measureFromRadioF1;
extern float measureFromRadioF2;
extern float measureFromRadio;
int switchState = 1;
bool current_freq = 0;

void testSwitch()
{
	initSwitchGPIOControl();
    toggleGPIOs(0,1);
    
    while(1)
    {
		switchNextAntenna(&switchState);
        std::cout << "New switchstate is " << switchState << ", waiting for input to switch" << std::endl;
        char whatever;
        std::cin >> whatever;
	}
}

int initSwitchGPIOControl()
{   
    // Initial state: listen to RF1
    mraa::Result status;
   
    // init V1 pin to LOW
    status = gpio_1.dir(mraa::DIR_OUT);     // Set as output
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }
    status = gpio_1.write(0);               // Set to low
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    // init V2 pin to LOW
    status = gpio_2.dir(mraa::DIR_OUT);     // Set as output
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }
    status = gpio_2.write(0);               // Set to low
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    // init V3 pin to HIGH
    status = gpio_3.dir(mraa::DIR_OUT);     // Set as output
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }
    status = gpio_3.write(1);               // Set to high
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int toggleGPIOs(bool V2_status, bool V3_status)
{
    mraa::Result status;
	
    status = gpio_2.write(V2_status);
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    status = gpio_3.write(V3_status);
    if (status != mraa::SUCCESS) {
        printError(status);
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

void switchNextAntenna(int * currentState)
{
    /*
    Truth table of the RF switch:
    | V1 | V2 | V3 | Active Input 
    |____|____|____|_____________
    |    |    |    |  
    | Lo | Lo | Lo |   RF4
    | Lo | Lo | Hi |   RF1
    | Lo | Hi | Lo |   RF2
    | Lo | Hi | Hi |   RF3
    | Hi | Lo | Lo |   RF4
    | Hi | Lo | Hi |   All Off
    | Hi | Hi | Hi |   All Off
    | Hi | Hi | Hi |   Unsupported

    All control have 100k internal pull down !

    With V1 always grounded, two pin control is enough -> 2-bit state variable.
    0b01 means V2 low, V3 high -> RF1 active
    0b11 means V2 high, V3 high -> RF3 active, etc.
    */

    switch(*currentState)
    {
        // 0b01 = 1; RF1 active, go to RF2
        case 1: 
            //std::cout << "Switching from RF1 to RF2" << std::endl;
            toggleGPIOs(1,0);
            *currentState = 2;
            break;

        // 0b10 = 2; RF2 active, go to RF3
        case 2: 
            //std::cout << "Switching from RF2 to RF3" << std::endl;
            toggleGPIOs(1,1);
            *currentState = 3;
            break;            

        // 0b11 = 3; RF3 active, go to RF4
        case 3: 
            //std::cout << "Switching from RF3 to RF4" << std::endl;
            toggleGPIOs(0,0);
            *currentState = 0;
            break;

        // 0b00 = 0; RF4 active, go to RF1
        case 0: 
            //std::cout << "Switching from RF4 to RF1" << std::endl;
            toggleGPIOs(0,1);
            *currentState = 1;
            break;
    }
}

void dataToFile(std::string filename, Flight* flight)
{
    std::ofstream outfile;
    outfile.open(filename, std::ios::out);
    // Change precision of floats to print
    outfile.precision(10);
    outfile.setf(std::ios::fixed);
    outfile.setf(std::ios::showpoint);   
    outfile << "Latitude, Longitude, Altitude, Height, Yaw, SwitchState, RadioValue" << std::endl;

    PositionData pos;
    Angle yaw;
    int data_amount = 0;
    int data_threshold = 5000;

    while(!endFlag && data_amount < data_threshold)
    {
	data_amount++;
        usleep(25000);
        pos = flight -> getPosition();
        yaw = flight -> getYaw();     // IN RAD !

        outfile <<  pos.latitude    << ","
                <<  pos.longitude   << ","
                <<  pos.altitude    << ","
                <<  pos.height      << ","
                <<  yaw             << ","
                <<  switchState     << ","
                <<  measureFromRadio  << std::endl;
    }
	
    if(data_amount > data_threshold)
    {
	   std::cout << "Threshold exceeded, expect incomplete data." << std::endl;
    }

    outfile.close();
}

int uhd_init_usrp(int argc, char *argv[]){
    uhd::set_thread_priority_safe();

    //variables to be set by po
    std::string args;
    
    std::cout << "Using Boost "     
          << BOOST_VERSION / 100000     << "."  // major version
          << BOOST_VERSION / 100 % 1000 << "."  // minor version
          << BOOST_VERSION % 100                // patch level
          << std::endl;

    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "multi uhd device address args")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("Mini-example to initialize a USRP (args==%s).") % args << std::endl;
        return ~0;
    }

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);

    return EXIT_SUCCESS;
}

int RadioRXStream1(int argc, char *argv[]){
    // Four antennas, one frequency
    uhd::set_thread_priority_safe();
    measureFromRadio = 0;

    //variables to be set by po
    std::string args;
    std::string wire;
    double seconds_in_future;
    size_t total_num_samps;
    size_t spb;
    double rate;
    std::string channel_list;
    double freq;
    double gain;
    double norm;
    
    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("programmatic", "test")
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("wire", po::value<std::string>(&wire)->default_value(""), "the over the wire type, sc16, sc8, etc")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(1000), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming samples")
        ("dilv", "specify to disable inner-loop verbose")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("freq", po::value<double>(&freq)->default_value(2e9), "RF center frequency in Hz")
        ("spb", po::value<size_t>(&spb)->default_value(1000), "samples per buffer")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX Timed Samples %s") % desc << std::endl;
    }

    bool verbose = vm.count("dilv") == 0;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

   //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()){
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }

    //set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));
    
    //set the center frequency
    if (vm.count("freq")) { //with default of 0.0 this will always be true
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(tune_request);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;
    }
    
    //set the RF gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain() << std::endl << std::endl;
    }

    // set the IF filter bandwidth
    double bw(100e6);
    std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw / 1e6) << std::endl;
    usrp->set_rx_bandwidth(bw);
    std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % (usrp->get_rx_bandwidth() / 1e6) << std::endl << std::endl;

    //create a receive streamer
    uhd::stream_args_t stream_args("fc32", wire); //complex floats
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    
    // Init GPIO
    initSwitchGPIOControl();
    toggleGPIOs(0,1);

    while(!endFlag) 
    { 
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps = total_num_samps;
        stream_cmd.stream_now = false;
        stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
        rx_stream->issue_stream_cmd(stream_cmd);

        // meta-data will be filled in by recv()
        uhd::rx_metadata_t md;

        // allocate buffer to receive with samples
        std::vector<std::complex<float> > buff(rx_stream->get_max_num_samps());
        std::vector<void *> buffs;
        for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
            buffs.push_back(&buff.front()); //same buffer for each channel

        // the first call to recv() will block this many seconds before receiving
        double timeout = seconds_in_future + 0.05; //timeout (delay before receive + padding)
        float avg = 0;
        size_t num_acc_samps = 0; //number of accumulated samples
        while(num_acc_samps < total_num_samps)
        {
            //receive a single packet
            size_t num_rx_samps = rx_stream->recv(
                buffs, buff.size(), md, timeout, true
            );
            for(int j=0; j<1000;j++){
                norm=abs(buff[j]);
                avg=avg+norm;
            }
            avg = avg/1000;
            
            // use a small timeout for subsequent packets
            timeout = 0.1;
            measureFromRadio = avg;
            //std::cout << "Radio value: " << measureFromRadio << std::endl;

            //handle the error code
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                throw std::runtime_error(str(boost::format(
                    "Receiver error %s"
                ) % md.strerror()));
            }

            num_acc_samps += num_rx_samps;
        }

        if (num_acc_samps < total_num_samps) std::cerr << "Receive timeout before all samples received..." << std::endl;

        //finished
        //std::cout << std::endl << "Done!" << std::endl << std::endl;
        seconds_in_future = seconds_in_future +0.05;
        
        // Go to next antenna
        switchNextAntenna(&switchState);
    }
}

int RadioRXStream2(int argc, char *argv[]){
    // Two antennas
    uhd::set_thread_priority_safe();
    measureFromRadio = 0;

    // Two freq variables
    int N_freq = 2;
    current_freq = 0;
    const std::vector<double> freqs = {2000e6, 2100e6};
    double span = 0.05; // Scan time in sec

    //variables to be set by po
    std::string args;
    std::string wire;
    double seconds_in_future;
    size_t total_num_samps;
    size_t spb;
    double rate;
    std::string channel_list;
    double freq;
    double gain;
    double norm;
    
    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("programmatic", "test")
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("wire", po::value<std::string>(&wire)->default_value(""), "the over the wire type, sc16, sc8, etc")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(1000), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming samples")
        ("dilv", "specify to disable inner-loop verbose")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("freq", po::value<double>(&freq)->default_value(2000e6), "RF center frequency in Hz")
        ("spb", po::value<size_t>(&spb)->default_value(1000), "samples per buffer")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX Timed Samples %s") % desc << std::endl;
    }

    bool verbose = vm.count("dilv") == 0;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

   //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()){
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }

    //set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));
    
    //set the center frequency
    if (vm.count("freq")) { //with default of 0.0 this will always be true
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(tune_request);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;
    }

    // // Set the parameters to change frequency
    // uhd::tune_request_t tune_requests[N_freq - 1];
    // uhd::tune_result_t t_result[N_freq - 1];

    // // Create tune requests
    // for(int i = 0; i < N_freq; i++)
    // {
    //     tune_requests[i].target_freq = freqs[i];
    //     //tune_request[i].args = uhd::device_addr_t("mode_n=integer");
    //     //tune_request[i].dsp_freq_policy = uhd::tune_request_t::policy_t(int('A'));
    //     //tune_request[i].rf_freq_policy = uhd::tune_request_t::policy_t(int('A'));
    //     std::cout << "Target " << i << ": " << freqs[i]/1e6 << " MHz" << std::endl;
    //     t_result[i] = usrp->set_rx_freq(tune_requests[i]);
    // }
    
    //set the RF gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain() << std::endl << std::endl;
    }

    // set the IF filter bandwidth
    double bw(25 * 1e6);
    std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw / 1e6) << std::endl;
    usrp->set_rx_bandwidth(bw);
    std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % (usrp->get_rx_bandwidth() / 1e6) << std::endl << std::endl;

    //create a receive streamer
    uhd::stream_args_t stream_args("fc32", wire); //complex floats
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);

    // Init GPIO
    initSwitchGPIOControl();
    toggleGPIOs(0,1);
    
 //    // Set timing of commands
 //    double cmd_time[N_freq - 1];
 //    usrp->set_time_now(uhd::time_spec_t(0.0));
	
 //    cmd_time[0]= usrp->get_time_now().get_real_secs() + (0.001);
	// cmd_time[1]= cmd_time[0] + 4*span;
	
	// //set the command in time
	// usrp->set_command_time(cmd_time[0]);
	// usrp->set_rx_freq(tune_requests[0]);
	// usrp->set_command_time(cmd_time[1]);
	// usrp->set_rx_freq(tune_requests[1]);
		
    bool i = 0;
    while(!endFlag) 
    { 		

        if (switchState == 0 && i != 0)
        // After looping on all 4 antennas, switch to the other frequency and start again
        {   

            current_freq = (current_freq == 1) ? 0:1;
			//std::cout << "Changing USRP freq" << std::endl;
            
            // cmd_time[0]= usrp->get_time_now().get_real_secs() + (0.001);
			// cmd_time[1]= cmd_time[0] + 4*span;
			
			//set the command in time
			//usrp->set_command_time(cmd_time[0]);
            uhd::tune_request_t tune_request(freqs[current_freq]);
			usrp->set_rx_freq(tune_request);
			//usrp->set_command_time(cmd_time[1]);
			// usrp->set_rx_freq(tune_requests[1]);
        }
        i = 1;
        
		//std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;
        
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps = total_num_samps;
        stream_cmd.stream_now = false;
        stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
        rx_stream->issue_stream_cmd(stream_cmd);
        // meta-data will be filled in by recv()
        uhd::rx_metadata_t md;

        // allocate buffer to receive with samples
        std::vector<std::complex<float> > buff(rx_stream->get_max_num_samps());
        std::vector<void *> buffs;
        for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
            buffs.push_back(&buff.front()); //same buffer for each channel

        // the first call to recv() will block this many seconds before receiving
        double timeout = seconds_in_future + 0.05; //timeout (delay before receive + padding)
        float avg = 0;
        size_t num_acc_samps = 0; //number of accumulated samples
        
        while(num_acc_samps < total_num_samps)
        {
            //receive a single packet
            size_t num_rx_samps = rx_stream->recv(
                buffs, buff.size(), md, timeout, true
            );
            for(int j=0; j<1000;j++){
                norm=abs(buff[j]);
                avg=avg+norm;
            }
            avg = avg/1000;
            
            // use a small timeout for subsequent packets
            timeout = 0.1;
            if (current_freq == 1)
            {
                measureFromRadioF2 = avg;
            }
            else
            {
                measureFromRadioF1 = avg;   
            }

            //handle the error code
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                throw std::runtime_error(str(boost::format(
                    "Receiver error %s"
                ) % md.strerror()));
            }

            num_acc_samps += num_rx_samps;
        }

        if (num_acc_samps < total_num_samps) std::cerr << "Receive timeout before all samples received..." << std::endl;

        //finished
        //std::cout << std::endl << "Done!" << std::endl << std::endl;
        seconds_in_future = seconds_in_future +0.05;
        

        // Toggle the antenna connected to the switch and update state
        switchNextAntenna(&switchState);
    }
}

std::vector<float> RadioRXBurst(int argc, char *argv[], float frequency){
    
    std::vector<float> measurements(4,0);

    // Four antennas, one frequency
    uhd::set_thread_priority_safe();
    measureFromRadio = 0;

    //variables to be set by po
    std::string args;
    std::string wire;
    double seconds_in_future;
    size_t total_num_samps;
    size_t spb;
    double rate;
    std::string channel_list;
    double freq;
    double gain;
    double norm;
    
    //setup the program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("programmatic", "test")
        ("help", "help message")
        ("args", po::value<std::string>(&args)->default_value(""), "single uhd device address args")
        ("wire", po::value<std::string>(&wire)->default_value(""), "the over the wire type, sc16, sc8, etc")
        ("secs", po::value<double>(&seconds_in_future)->default_value(1.5), "number of seconds in the future to receive")
        ("nsamps", po::value<size_t>(&total_num_samps)->default_value(1000), "total number of samples to receive")
        ("rate", po::value<double>(&rate)->default_value(100e6/16), "rate of incoming samples")
        ("dilv", "specify to disable inner-loop verbose")
        ("channels", po::value<std::string>(&channel_list)->default_value("0"), "which channel(s) to use (specify \"0\", \"1\", \"0,1\", etc)")
        ("freq", po::value<double>(&freq)->default_value(frequency), "RF center frequency in Hz")
        ("spb", po::value<size_t>(&spb)->default_value(1000), "samples per buffer")
        ("gain", po::value<double>(&gain), "gain for the RF chain")
    ;
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    //print the help message
    if (vm.count("help")){
        std::cout << boost::format("UHD RX Timed Samples %s") % desc << std::endl;
    }

    bool verbose = vm.count("dilv") == 0;

    //create a usrp device
    std::cout << std::endl;
    std::cout << boost::format("Creating the usrp device with: %s...") % args << std::endl;
    uhd::usrp::multi_usrp::sptr usrp = uhd::usrp::multi_usrp::make(args);
    std::cout << boost::format("Using Device: %s") % usrp->get_pp_string() << std::endl;

   //detect which channels to use
    std::vector<std::string> channel_strings;
    std::vector<size_t> channel_nums;
    boost::split(channel_strings, channel_list, boost::is_any_of("\"',"));
    for(size_t ch = 0; ch < channel_strings.size(); ch++){
        size_t chan = boost::lexical_cast<int>(channel_strings[ch]);
        if(chan >= usrp->get_tx_num_channels() or chan >= usrp->get_rx_num_channels()){
            throw std::runtime_error("Invalid channel(s) specified.");
        }
        else channel_nums.push_back(boost::lexical_cast<int>(channel_strings[ch]));
    }

    //set the rx sample rate
    std::cout << boost::format("Setting RX Rate: %f Msps...") % (rate/1e6) << std::endl;
    usrp->set_rx_rate(rate);
    std::cout << boost::format("Actual RX Rate: %f Msps...") % (usrp->get_rx_rate()/1e6) << std::endl << std::endl;

    std::cout << boost::format("Setting device timestamp to 0...") << std::endl;
    usrp->set_time_now(uhd::time_spec_t(0.0));
    
    //set the center frequency
    if (vm.count("freq")) { //with default of 0.0 this will always be true
        std::cout << boost::format("Setting RX Freq: %f MHz...") % (freq/1e6) << std::endl;
        uhd::tune_request_t tune_request(freq);
        if(vm.count("int-n")) tune_request.args = uhd::device_addr_t("mode_n=integer");
        usrp->set_rx_freq(tune_request);
        std::cout << boost::format("Actual RX Freq: %f MHz...") % (usrp->get_rx_freq()/1e6) << std::endl << std::endl;
    }
    
    //set the RF gain
    if (vm.count("gain")) {
        std::cout << boost::format("Setting RX Gain: %f dB...") % gain << std::endl;
        usrp->set_rx_gain(gain);
        std::cout << boost::format("Actual RX Gain: %f dB...") % usrp->get_rx_gain() << std::endl << std::endl;
    }

    // set the IF filter bandwidth
    double bw(25 * 1e6);
    std::cout << boost::format("Setting RX Bandwidth: %f MHz...") % (bw / 1e6) << std::endl;
    usrp->set_rx_bandwidth(bw);
    std::cout << boost::format("Actual RX Bandwidth: %f MHz...") % (usrp->get_rx_bandwidth() / 1e6) << std::endl << std::endl;

    //create a receive streamer
    uhd::stream_args_t stream_args("fc32", wire); //complex floats
    stream_args.channels = channel_nums;
    uhd::rx_streamer::sptr rx_stream = usrp->get_rx_stream(stream_args);
    
    // Init GPIO
    initSwitchGPIOControl();
    toggleGPIOs(0,1);

    for (int i = 0; i < 16; ++i) 
    { 
        uhd::stream_cmd_t stream_cmd(uhd::stream_cmd_t::STREAM_MODE_NUM_SAMPS_AND_DONE);
        stream_cmd.num_samps = total_num_samps;
        stream_cmd.stream_now = false;
        stream_cmd.time_spec = uhd::time_spec_t(seconds_in_future);
        rx_stream->issue_stream_cmd(stream_cmd);

        // meta-data will be filled in by recv()
        uhd::rx_metadata_t md;

        // allocate buffer to receive with samples
        std::vector<std::complex<float> > buff(rx_stream->get_max_num_samps());
        std::vector<void *> buffs;
        for (size_t ch = 0; ch < rx_stream->get_num_channels(); ch++)
            buffs.push_back(&buff.front()); //same buffer for each channel

        // the first call to recv() will block this many seconds before receiving
        double timeout = seconds_in_future + 0.05; //timeout (delay before receive + padding)
        float avg = 0;
        size_t num_acc_samps = 0; //number of accumulated samples
        while(num_acc_samps < total_num_samps)
        {
            //receive a single packet
            size_t num_rx_samps = rx_stream->recv(
                buffs, buff.size(), md, timeout, true
            );
            for(int j=0; j<1000;j++){
                norm=abs(buff[j]);
                avg=avg+norm;
            }
            avg = avg/1000;
            
            // use a small timeout for subsequent packets
            timeout = 0.1;
            if(avg > measurements[switchState])
            {
                measurements[switchState] = avg;    
            }
            
            //std::cout << "Radio value: " << measureFromRadio << std::endl;

            //handle the error code
            if (md.error_code == uhd::rx_metadata_t::ERROR_CODE_TIMEOUT) break;
            if (md.error_code != uhd::rx_metadata_t::ERROR_CODE_NONE){
                throw std::runtime_error(str(boost::format(
                    "Receiver error %s"
                ) % md.strerror()));
            }

            num_acc_samps += num_rx_samps;
        }

        if (num_acc_samps < total_num_samps) std::cerr << "Receive timeout before all samples received..." << std::endl;

        //finished
        //std::cout << std::endl << "Done!" << std::endl << std::endl;
        seconds_in_future = seconds_in_future +0.05;
        
        // Go to next antenna
        switchNextAntenna(&switchState);
    }

    return measurements;
}

int radioMain(int argc, char *argv[], int config)
{    
    switch (config)
    {
        case 1:
	    {
            int status1 = RadioRXStream1(argc, argv);
            break;
	    }
        case 2:
        {
			std::cout << "Going for radio2" << std::endl;
            int status2 = RadioRXStream2(argc, argv);
            break;            
	    }
    }

    
    return 0;
}
