/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Jonathan Bohren
   Desc:   Gazebo plugin for running OROCOS RTT components */

// Boost
#include <boost/bind.hpp>

// Gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <rtt/internal/GlobalService.hpp>

// Orocos
#include <rtt/deployment/ComponentLoader.hpp>
#include <ocl/DeploymentComponent.hpp>
#include <ocl/LoggingService.hpp>
#include <rtt/Logger.hpp>

#include <rtt/scripting/Scripting.hpp>
#include <rtt/transports/corba/corba.h>
#include <rtt/transports/corba/TaskContextServer.hpp>
#include <rtt/plugin/PluginLoader.hpp>

#include <rtt_rosclock/rtt_rosclock.h>

#include <ros/ros.h>
#include "std_srvs/Empty.h"

#include <unistd.h>

// RTT/ROS Simulation Clock Activity

#include "gazebo_deployer_model_plugin.h"

using namespace rtt_gazebo_deployer;
using namespace RTT;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(GazeboDeployerModelPlugin);

// Static definitions
RTT::corba::TaskContextServer * GazeboDeployerModelPlugin::taskcontext_server;
OCL::DeploymentComponent * GazeboDeployerModelPlugin::default_deployer = NULL;
std::map<std::string,SubsystemDeployer*> GazeboDeployerModelPlugin::deployers;
boost::mutex GazeboDeployerModelPlugin::deferred_load_mutex;
ros::NodeHandle GazeboDeployerModelPlugin::nh("~");
ros::ServiceServer GazeboDeployerModelPlugin::ss_enable_sim_;

static bool enable_sim_flag = false;

bool enable_sim(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    enable_sim_flag = true;
    return true;
}

static boost::thread single_step_thread_;

void singleStep() {
    if (!enable_sim_flag) {
        return;
    }

    single_step_thread_.join();
    single_step_thread_ = boost::thread(boost::bind(&gazebo::physics::World::Step, gazebo::physics::get_world(), 1));
}

void waitForPreviousStep() {
    if (!enable_sim_flag) {
        return;
    }
}

GazeboDeployerModelPlugin::GazeboDeployerModelPlugin() :
  gazebo::ModelPlugin()
{
}

GazeboDeployerModelPlugin::~GazeboDeployerModelPlugin()
{
}

// Overloaded Gazebo entry point
void GazeboDeployerModelPlugin::Load(
    gazebo::physics::ModelPtr parent,
    sdf::ElementPtr sdf)
{
  RTT::Logger::Instance()->in("GazeboDeployerModelPlugin::load");

  // Save pointer to the model
  parent_model_ = parent;

  // Save the SDF source
  sdf_ = sdf;
/*
  // Perform the rest of the asynchronous loading
  // This is important since RTT scripts need the gazebo clock to tick
  // forward. Make the model weightless while this happens, the actual
  // gravity modes will be restored at the end of the load thread.
  model_links_ = parent_model_->GetLinks();
  for(gazebo::physics::Link_V::iterator it = model_links_.begin();
      it != model_links_.end();
      ++it)
  {
    actual_gravity_modes_.push_back(
        std::pair<gazebo::physics::LinkPtr, bool>(*it, (*it)->GetGravityMode()));
    (*it)->SetGravityMode(false);
  }
*/
  deferred_load_thread_ = boost::thread(boost::bind(&GazeboDeployerModelPlugin::loadThread, this));
}

// Load in seperate thread from Gazebo in case something blocks
void GazeboDeployerModelPlugin::loadThread()
{
  boost::mutex::scoped_lock load_lock(deferred_load_mutex);

  RTT::Logger::Instance()->in("GazeboDeployerModelPlugin::loadThread");

  Logger::log() << Logger::Info << "Loading RTT Model Plugin..." << Logger::endl;

  std::string master_service_name;
  std::string master_service_subname;

  if(sdf_->HasElement("master_service"))
  {
    Logger::log() << Logger::Info << "Loading Gazebo RTT components..." << Logger::endl;

    sdf::ElementPtr master_service_elem = sdf_->GetElement("master_service");

    if(!master_service_elem->HasElement("name"))
    {
      Logger::log() << Logger::Error << "SDF rtt_gazebo plugin <master_service> tag is missing a required field <name>" << Logger::endl;
      _exit(13);
      return;
    }
    sdf::ElementPtr master_service_name_elem = master_service_elem->GetElement("name");
    master_service_name = master_service_name_elem->Get<std::string>();

    if(master_service_elem->HasElement("subname")) {
      sdf::ElementPtr master_service_subname_elem = master_service_elem->GetElement("subname");
      master_service_subname = master_service_subname_elem->Get<std::string>();
    }
  }
  else {
    Logger::log() << Logger::Error << "SDF rtt_gazebo plugin <master_service> tag is missing" << Logger::endl;
    _exit(12);
    return;
  }


  // Create main gazebo deployer if necessary
  if (default_deployer == NULL) {

    Logger::log() << Logger::Info << "Creating new default deployer named \"gazebo\"" << Logger::endl;
    // Create the gazebo deployer
    default_deployer = new OCL::DeploymentComponent("gazebo");
    default_deployer->import("rtt_rosnode");
    default_deployer->import("rtt_rosdeployment");
    static_cast<RTT::TaskContext*>(default_deployer)->loadService("rosdeployment");

    // Attach the taskcontext server to this component
    taskcontext_server = RTT::corba::TaskContextServer::Create(default_deployer);
  }

  // Check if this deployer should have a custom name
  if(sdf_->HasElement("isolated")) {
    deployer_name_ = master_service_name + master_service_subname;//parent_model_->GetName()+std::string("__deployer__");
  } else {
    deployer_name_ = "gazebo";
  }

  Logger::log() << Logger::Info << "Deployer name: " << deployer_name_ << Logger::endl;


  if (!RTT::internal::GlobalService::Instance()->hasService("gazebo_rtt_service")) {
    RTT::Service::shared_ptr gazebo_rtt_service = RTT::internal::GlobalService::Instance()->provides("gazebo_rtt_service");

    gazebo_rtt_service->doc("RTT service for realtime and non-realtime clock measurement.");

    gazebo_rtt_service->addOperation("singleStep", &singleStep).doc(
        "Execute single step of gazebo simulation.");

    gazebo_rtt_service->addOperation("waitForPreviousStep", &waitForPreviousStep).doc(
        "Execute single step of gazebo simulation.");

    ss_enable_sim_ = nh.advertiseService("enable_sim", &enable_sim);
  }



  // Create component deployer if necessary
  if(deployer_name_ != "gazebo" && deployers.find(deployer_name_) == deployers.end()) {
//  if(deployers.find(deployer_name_) == deployers.end()) {
    Logger::log() << Logger::Info << "Creating new deployer named \"" << deployer_name_ << "\"" << Logger::endl;
    deployers[deployer_name_] = new SubsystemDeployer(deployer_name_);

// TODO: add master service name
    deployers[deployer_name_]->initializeSubsystem(master_service_name, master_service_subname);
    deployers[deployer_name_]->getDc()->connectPeers(default_deployer);
    deployers[deployer_name_]->getDc()->import("rtt_rosnode");
    deployers[deployer_name_]->getDc()->import("rtt_rosdeployment");
    static_cast<RTT::TaskContext*>(deployers[deployer_name_]->getDc().get())->loadService("rosdeployment");
    RTT::corba::TaskContextServer::Create(deployers[deployer_name_]->getDc().get());
  }


  // Error message if the model couldn't be found
  if (!parent_model_) {
    _exit(11);
    return;
  }

  // Get a pointer to this model's deployer
  boost::shared_ptr<OCL::DeploymentComponent > deployer = deployers[deployer_name_]->getDc();

  // Check if there is a special gazebo component that should be connected to the world
  if(sdf_->HasElement("component"))
  {
    Logger::log() << Logger::Info << "Loading Gazebo RTT components..." << Logger::endl;

    sdf::ElementPtr component_elem = sdf_->GetElement("component");

    while(component_elem && component_elem->GetName() == "component")
    {
      // Initialize gazebo component
      RTT::TaskContext* new_model_component = NULL;

      if(!component_elem->HasElement("package") ||
         !component_elem->HasElement("type") ||
         !component_elem->HasElement("name"))
      {
        Logger::log() << Logger::Error << "SDF rtt_gazebo plugin <component> tag is missing a required field!" << Logger::endl;
        gzerr << "SDF rtt_gazebo plugin <component> tag is missing a required field!" << std::endl;
        _exit(10);
        return;
      }
      // Get the component name
      Logger::log() << Logger::Info << "Getting gazebo RTT component information..." << Logger::endl;
      std::string model_component_package = component_elem->GetElement("package")->Get<std::string>();
      std::string model_component_type = component_elem->GetElement("type")->Get<std::string>();
      std::string model_component_name = component_elem->GetElement("name")->Get<std::string>();

      Logger::log() << Logger::Info << "Loading gazebo RTT component package \"" << model_component_package <<"\""<<Logger::endl;

      // Import the package
      if(!rtt_ros::import(model_component_package)) {
        Logger::log() << Logger::Error << "Could not import rtt_gazebo model component package: \"" << model_component_package << "\"" << Logger::endl;
        gzerr << "Could not import rtt_gazebo model component package: \"" << model_component_package << "\"" <<std::endl;
        _exit(9);
        return;
      }

      // Load the component
      if(!deployer->loadComponent(model_component_name, model_component_type)) {
        Logger::log() << Logger::Error << "Could not load rtt_gazebo model component: \"" << model_component_type << "\"" << Logger::endl;
        gzerr << "Could not load rtt_gazebo model component: \"" << model_component_type << "\"" <<std::endl;
        _exit(8);
        return;
      }

      // Get the model component from the deployer by name
      if(deployer->hasPeer(model_component_name)) {
        new_model_component = deployer->getPeer(model_component_name);
      } else {
        Logger::log() << Logger::Error << "SDF model plugin specified a special gazebo component to connect to the gazebo update, named \""<<model_component_name<<"\", but there is no peer by that name." << Logger::endl;
        gzerr << "SDF model plugin specified a special gazebo component to connect to the gazebo update, named \""<<model_component_name<<"\", but there is no peer by that name." <<std::endl;
        _exit(7);
        return;
      }

      // Make sure the component has the required interfaces
      if( new_model_component == NULL ) {
        Logger::log() << Logger::Error << "RTT model component was not properly created." << Logger::endl;
        gzerr << "RTT model component was not properly created." << std::endl; return; }
      if( !new_model_component->provides()->hasService("gazebo") ) {
        Logger::log() << Logger::Error << "RTT model component does not have required \"gazebo\" service." << Logger::endl;
        gzerr << "RTT model component does not have required \"gazebo\" service." << std::endl; return; }
      if( !new_model_component->provides("gazebo")->hasOperation("configure") ) {
        Logger::log() << Logger::Error << "RTT model component does not have required \"gazebo.configure\" operation." << Logger::endl;
        gzerr << "RTT model component does not have required \"gazebo.configure\" operation." << std::endl; return; }
      if( !new_model_component->provides("gazebo")->hasOperation("update") ) {
        Logger::log() << Logger::Error << "RTT model component does not have required \"gazebo.update\" operation." << Logger::endl;
        gzerr << "RTT model component does not have required \"gazebo.update\" operation." << std::endl; return; }

      // Configure the component with the parent model
      RTT::OperationCaller<bool(gazebo::physics::ModelPtr)> gazebo_configure =
        new_model_component->provides("gazebo")->getOperation("configure");

      // Make sure the operation is ready
      if(!gazebo_configure.ready()) {
        Logger::log() << Logger::Error << "RTT model component's \"gazebo.configure\" operation could not be connected. Check its signature." << Logger::endl;
        gzerr <<"RTT model component's \"gazebo.configure\" operation could not be connected. Check its signature." << std::endl;
        _exit(6);
        return;
      }

      if(!gazebo_configure(parent_model_)){
        Logger::log() << Logger::Error << "RTT model component's \"gazebo.configure\" operation returned false." << Logger::endl;
        gzerr <<"RTT model component's \"gazebo.configure\" operation returned false." << std::endl;
        _exit(5);
        return;
      }

      // Get gazebo update function
      GazeboUpdateCaller gazebo_update_caller = new_model_component->provides("gazebo")->getOperation("update");

      if(!gazebo_update_caller.ready()) {
        Logger::log() << Logger::Error << "RTT model component's \"gazebo.update\" operation could not be connected. Check its signature." << Logger::endl;
        gzerr <<"RTT model component's \"gazebo.update\" operation could not be connected. Check its signature." << std::endl;
        _exit(4);
        return;
      }

      model_components_.push_back(new_model_component);
      gazebo_update_callers_.push_back(gazebo_update_caller);

      // Get the next element
      component_elem = component_elem->GetNextElement("component");
    }

    if(model_components_.empty()) {
      Logger::log() << Logger::Error << "Could not load any RTT components!" << Logger::endl;
      gzerr << "Could not load any RTT components!" << std::endl;
      _exit(3);
      return;
    }

  } else {
    RTT::log(RTT::Warning) << "No RTT component defined for Gazebo hooks." << Logger::endl;
    // return;
  }

  // Load initialization scripts
  this->loadScripts();

  deployers[deployer_name_]->configure();
//  deployers[deployer_name_]->runTaskBrowser();


  // Listen to the update event. This event is broadcast every simulation iteration.
  update_connections_.push_back(gazebo::event::Events::ConnectWorldUpdateEnd(
          boost::bind(&GazeboDeployerModelPlugin::gazeboUpdate, this)));

  RTT::log(RTT::Info) << "Gazebo rtt plugin loaded." << Logger::endl;
}

void GazeboDeployerModelPlugin::loadScripts()
{

  // Get a pointer to this model's deployer
  //boost::shared_ptr<OCL::DeploymentComponent > deployer = deployers[deployer_name_]->getDc();
  SubsystemDeployer* deployer = deployers[deployer_name_];

  std::vector<std::string > scripts;

  // Get the orocos ops script(s) to run in the deployer
  if(sdf_->HasElement("orocosScript"))
  {
    sdf::ElementPtr script_elem = sdf_->GetElement("orocosScript");

    while(script_elem && script_elem->GetName() == "orocosScript")
    {
      if(script_elem->HasElement("filename")) {
        std::string ops_script_file = script_elem->GetElement("filename")->Get<std::string>();
        RTT::log(RTT::Info) << "Running orocos ops script file " << ops_script_file << "..." << Logger::endl;

//        gzlog << "Running orocos ops script file "<<ops_script_file<<"..." << std::endl;
        scripts.push_back(ops_script_file);
//        if(!deployer->runScript(ops_script_file)) {
//          gzerr << "Could not run ops script file "<<ops_script_file<<"!" << std::endl;
//          return;
//        }
      }
//TODO: inline scripts are not supported
//      else if(script_elem->HasElement("inline")) {
//        std::string ops_script = script_elem->GetElement("inline")->Get<std::string>();
//        gzlog << "Running inline orocos ops script:"<< std::endl << ops_script << std::endl;
//        if(!deployer->getProvider<RTT::Scripting>("scripting")->eval(ops_script)) {
//          gzerr << "Could not run inline ops script!" << std::endl;
//          return;
//        }
//      }

      script_elem = script_elem->GetNextElement("orocosScript");
    }
  }

  std::vector<std::string > subsystem_xmls;
  if(sdf_->HasElement("subsystem_xml"))
  {
    sdf::ElementPtr script_elem = sdf_->GetElement("subsystem_xml");

    while(script_elem && script_elem->GetName() == "subsystem_xml")
    {
      if(script_elem->HasElement("filename")) {
        std::string ops_script_file = script_elem->GetElement("filename")->Get<std::string>();
        RTT::log(RTT::Info) << "Running orocos subsystem xml file " << ops_script_file << "..." << Logger::endl;

        subsystem_xmls.push_back(ops_script_file);
      }


      script_elem = script_elem->GetNextElement("subsystem_xml");
    }
  }

  if (!deployer->runXmls(subsystem_xmls)) {
      Logger::log() << Logger::Error << "Could not load subsystem xml files." << Logger::endl;
      _exit(2);
      return;
  }

  if (!deployer->runScripts(scripts)) {
      Logger::log() << Logger::Error << "Could not load script files." << Logger::endl;
      _exit(1);
      return;
  }

/*
  // Load lua scripting service
  if(!RTT::plugin::PluginLoader::Instance()->loadService("Lua", deployer)) {
    gzerr << "Could not load lua service." << std::endl;
    return;
  }

  RTT::OperationCaller<bool(std::string)> exec_file =
    deployer->provides("Lua")->getOperation("exec_file");
  RTT::OperationCaller<bool(std::string)> exec_str =
    deployer->provides("Lua")->getOperation("exec_str");

  if(!exec_file.ready() || !exec_str.ready()) {
    gzerr << "Could not get lua operations." << std::endl;
    return;
  }

  // Load rttlib for first-class operation support
  exec_str("require(\"rttlib\")");

  // Get lua scripts to run in the deployer
  if(sdf_->HasElement("luaScript"))
  {

    sdf::ElementPtr script_elem = sdf_->GetElement("luaScript");

    while(script_elem && script_elem->GetName() == "luaScript")
    {
      if(script_elem->HasElement("filename")) {
        std::string lua_script_file = script_elem->GetElement("filename")->Get<std::string>();
        gzlog << "Running orocos lua script file "<<lua_script_file<<"..." << std::endl;
        if(!exec_file(lua_script_file)) {
          gzerr << "Could not run lua script file "<<lua_script_file<<"!" << std::endl;
          return;
        }
      } else if(script_elem->HasElement("inline")) {
        std::string lua_script = script_elem->GetElement("inline")->Get<std::string>();
        gzlog << "Running inline orocos lua script:" << std::endl << lua_script << std::endl;
        if(!exec_str(lua_script)) {
          gzerr << "Could not run inline lua script!" << std::endl;
          return;
        }
      }

      script_elem = script_elem->GetNextElement("luaScript");
    }
  }
*/
  Logger::log() << Logger::Info << "Done executing Orocos scripts for gazebo model plugin." << Logger::endl;

  // Restore gravity modes
  for(std::vector<std::pair<gazebo::physics::LinkPtr, bool> >::iterator it = actual_gravity_modes_.begin();
      it != actual_gravity_modes_.end();
      ++it)
  {
    it->first->SetGravityMode(it->second);
  }
}

// Called by the world update start event
void GazeboDeployerModelPlugin::gazeboUpdate()
{
  RTT::Logger::Instance()->in("GazeboDeployerModelPlugin::gazeboUpdate");

  boost::mutex::scoped_try_lock lock(deferred_load_mutex);

  if(lock) {
    // Call orocos RTT model component gazebo.update() operations
    for(std::vector<GazeboUpdateCaller>::iterator caller = gazebo_update_callers_.begin();
        caller != gazebo_update_callers_.end();
        ++caller)
    {
      (*caller)(parent_model_);
    }
  }
}
