
#include <cstdlib>
#include <unistd.h>

// Boost
#include <boost/bind.hpp>
#include <boost/thread/locks.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/TaskBrowser.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>


#include <rtt/os/startstop.h>
#include <rtt/scripting/Scripting.hpp>
#include <rtt/transports/corba/corba.h>
#include <rtt/transports/corba/TaskContextServer.hpp>

// RTT/ROS Simulation Clock Activity
#include <rtt_rosclock/rtt_rosclock.h>
#ifdef RTT_GAZEBO_DEBUG
#include <rtt_rosclock/prof.h>
#include <rtt_rosclock/throttle.h>
#endif

#include "rtt_system_plugin.h"

using namespace rtt_gazebo_system;
using namespace RTT;

void RTTSystemPlugin::Load(int argc, char **argv)
{
  // Initialize RTT
  __os_init(argc, argv);

  sim_clock_period_ = 0.0;
  for (int i = 1; i < argc; ++i) {
    if (strcmp("--sim_clock_period", argv[i]) == 0) {
      sim_clock_period_ = std::stod( std::string(argv[i+1]) );
      break;
    }
  }

  RTT::Logger::log().setStdStream(std::cerr);
  RTT::Logger::log().mayLogStdOut(true);

  Logger::log() << Logger::Info << "RTTSystemPlugin sim_clock_period: " << sim_clock_period_ << Logger::endl;

/*
  // Setup TaskContext server if necessary
  if(CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    // Initialize orb
    RTT::corba::TaskContextServer::InitOrb(argc, argv);
    // Propcess orb requests in a thread
    RTT::corba::TaskContextServer::ThreadOrb();
  }
*/
}

void RTTSystemPlugin::Init()
{
  // Initialize and enable the simulation clock
  rtt_rosclock::use_manual_clock();
  rtt_rosclock::enable_sim();

  update_connection_ =
    gazebo::event::Events::ConnectWorldUpdateEnd(
        boost::bind(&RTTSystemPlugin::updateClock, this));

  // TODO: Create a worldupdateend connection
  
  simulate_clock_ = true;
}

RTTSystemPlugin::~RTTSystemPlugin()
{
/*  // Stop the Orb thread
  if(!CORBA::is_nil(RTT::corba::TaskContextServer::orb)) {
    RTT::corba::TaskContextServer::ShutdownOrb();
    RTT::corba::TaskContextServer::DestroyOrb();
    RTT::corba::TaskContextServer::CleanupServers();
  }
*/
}

void RTTSystemPlugin::updateClock()
{
  // Wait for previous update thread
//  if(update_thread_.joinable()) {
//    update_thread_stop_ = true;
//    update_thread_.join();
//  }
//  update_thread_stop_ = false;

  updateClockLoop();
  // Start update thread
//  update_thread_ = boost::thread(
//      boost::bind(&RTTSystemPlugin::updateClockLoop, this));
}

void RTTSystemPlugin::updateClockLoop()
{
  {

    // Get the simulation time
    gazebo::common::Time gz_time = gazebo::physics::get_world()->SimTime();

    // Update the clock from the simulation time and execute the SimClockActivities
    // NOTE: all orocos TaskContexts which use a SimClockActivity are updated within this call
#ifdef RTT_GAZEBO_DEBUG
    static rtt_rosclock::WallProf prof(5.0);
    static rtt_rosclock::WallThrottle throttle(ros::Duration(1.0));

    prof.tic();
#endif

//    if (sim_clock_period_ == 0.0) {
        rtt_rosclock::update_sim_clock(ros::Time(gz_time.sec, gz_time.nsec));
/*    }
    else {
        ros::Time first_update_time = rtt_rosclock::rtt_wall_now();
        ros::Time time_start = ros::Time(gz_time.sec, gz_time.nsec);
        ros::Time time_update = time_start;
        rtt_rosclock::update_sim_clock(time_start);

        double max_step = gazebo::physics::get_world()->GetPhysicsEngine()->GetMaxStepSize();

        while (true) {
            usleep( int(sim_clock_period_*1000000.0) );
            ros::Time time_now = rtt_rosclock::rtt_wall_now();
            time_update = time_update + ros::Duration(sim_clock_period_);
            if (time_now - first_update_time >= ros::Duration(max_step) || time_update - time_start >= ros::Duration(max_step)) {
                break;
            }
            rtt_rosclock::update_sim_clock(time_update);
        }
    }
*/

#ifdef RTT_GAZEBO_DEBUG
    prof.toc();
    if(throttle.ready()) {
      prof.analyze();
      Logger::log() << Logger::Debug << prof.mean() << " +/- " << prof.stddev() <<" [s] ("<<prof.n()<<") for update_sim_clock()" << Logger::endl;
    }
    static ros::Time last_update_time = rtt_rosclock::rtt_wall_now();
#endif
  }
}

GZ_REGISTER_SYSTEM_PLUGIN(rtt_gazebo_system::RTTSystemPlugin)
