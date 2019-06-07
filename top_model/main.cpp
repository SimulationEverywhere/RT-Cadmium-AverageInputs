/**
* By: Ben Earle and Kyle Bjornson
* ARSLab - Carleton University
*
* Analog Input:
* Model to interface with a analog Input pin for Embedded Cadmium.
*/
#include <iostream>
#include <chrono>
#include <algorithm>
#include <string>

#include <cadmium/modeling/coupled_model.hpp>
#include <cadmium/modeling/ports.hpp>
#include <cadmium/modeling/dynamic_model_translator.hpp>
#include <cadmium/concept/coupled_model_assert.hpp>
#include <cadmium/modeling/dynamic_coupled.hpp>
#include <cadmium/modeling/dynamic_atomic.hpp>
#include <cadmium/engine/pdevs_dynamic_runner.hpp>
#include <cadmium/logger/tuple_to_ostream.hpp>
#include <cadmium/logger/common_loggers.hpp>

#include <NDTime.hpp>
#include <cadmium/io/iestream.hpp>


#include <cadmium/embedded/fusion/averageInput.hpp>
#include <cadmium/embedded/io/analogInput.hpp>
#include <cadmium/embedded/io/pwmOutput.hpp>

#ifdef ECADMIUM
  #include "../mbed.h"
#else
  // When simulating the model it will use these files as IO in place of the pins specified.
  const char* A4   = "./inputs/A4_In.txt";
  const char* A5   = "./inputs/A5_In.txt";
  const char* D11 = "./outputs/D11_Out.txt";
#endif

using namespace std;

using hclock=chrono::high_resolution_clock;
using TIME = NDTime;


int main(int argc, char ** argv) {

  #ifdef ECADMIUM
      //Logging is done over cout in ECADMIUM
      struct oss_sink_provider{
        static std::ostream& sink(){
          return cout;
        }
      };
  #else
    // all simulation timing and I/O streams are ommited when running embedded
    auto start = hclock::now(); //to measure simulation execution time

    static std::ofstream out_data("avg_in_output.txt");
    struct oss_sink_provider{
      static std::ostream& sink(){
        return out_data;
      }
    };
  #endif

  /*************** Loggers *******************/
  using info=cadmium::logger::logger<cadmium::logger::logger_info, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using debug=cadmium::logger::logger<cadmium::logger::logger_debug, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using state=cadmium::logger::logger<cadmium::logger::logger_state, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_messages=cadmium::logger::logger<cadmium::logger::logger_messages, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using routing=cadmium::logger::logger<cadmium::logger::logger_message_routing, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using global_time=cadmium::logger::logger<cadmium::logger::logger_global_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using local_time=cadmium::logger::logger<cadmium::logger::logger_local_time, cadmium::dynamic::logger::formatter<TIME>, oss_sink_provider>;
  using log_all=cadmium::logger::multilogger<info, debug, state, log_messages, routing, global_time, local_time>;

  using logger_top=cadmium::logger::multilogger<log_messages, global_time>;


  /*******************************************/

  using AtomicModelPtr=std::shared_ptr<cadmium::dynamic::modeling::model>;
  using CoupledModelPtr=std::shared_ptr<cadmium::dynamic::modeling::coupled<TIME>>;

  /********************************************/
  /*********** MajorityVote *******************/
  /********************************************/

  AtomicModelPtr avg = cadmium::dynamic::translate::make_dynamic_atomic_model<AverageInput, TIME>("avg", 2);

  /********************************************/
  /********** DigitalInput1 *******************/
  /********************************************/
  AtomicModelPtr A4_in = cadmium::dynamic::translate::make_dynamic_atomic_model<AnalogInput, TIME>("A4", A4);
  AtomicModelPtr A5_in = cadmium::dynamic::translate::make_dynamic_atomic_model<AnalogInput, TIME>("A5", A5);

  /********************************************/
  /********* DigitalOutput1 *******************/
  /********************************************/
  AtomicModelPtr pwmOut = cadmium::dynamic::translate::make_dynamic_atomic_model<PwmOutput, TIME>("pwmOut", D11);


  /************************/
  /*******TOP MODEL********/
  /************************/
  //No external ports since this model is designed for the embedded platform
  cadmium::dynamic::modeling::Ports iports_TOP = {};
  cadmium::dynamic::modeling::Ports oports_TOP = {};
  cadmium::dynamic::modeling::EICs eics_TOP = {};
  cadmium::dynamic::modeling::EOCs eocs_TOP = {};
  cadmium::dynamic::modeling::Models submodels_TOP =  {avg, A4_in, A5_in, pwmOut};
  cadmium::dynamic::modeling::ICs ics_TOP = {
    // Majority vote's output will controll the output
    cadmium::dynamic::translate::make_IC<averageInput_defs::out, pwmOutput_defs::in>("avg","pwmOut"),
    // All the analog inputs will be fed into the averager
    cadmium::dynamic::translate::make_IC<analogInput_defs::out, averageInput_defs::in1>("A4", "avg"),
    cadmium::dynamic::translate::make_IC<analogInput_defs::out, averageInput_defs::in2>("A5", "avg")
  };
  CoupledModelPtr TOP = std::make_shared<cadmium::dynamic::modeling::coupled<TIME>>(
    "TOP",
    submodels_TOP,
    iports_TOP,
    oports_TOP,
    eics_TOP,
    eocs_TOP,
    ics_TOP
  );

  ///****************////
  #ifdef ECADMIUM
    DigitalOut rightMotorEn(D9);
    DigitalOut rightMotor1(D8);
    rightMotorEn = 1;
    rightMotor1 = 0;
    //cadmium::dynamic::engine::runner<NDTime, logger_top> r(TOP, {0});
    cadmium::dynamic::engine::runner<NDTime, cadmium::logger::not_logger> r(TOP, {0});
    r.run_until(NDTime("00:10:00:000"));
  #else
    cadmium::dynamic::engine::runner<NDTime, logger_top> r(TOP, {0});
    r.run_until(NDTime("00:10:00:000"));
    return 0;
  #endif
}
