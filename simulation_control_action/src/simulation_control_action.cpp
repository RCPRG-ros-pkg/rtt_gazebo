/*
 Copyright (c) 2014, Robot Control and Pattern Recognition Group, Warsaw University of Technology
 All rights reserved.
 
 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.
     * Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.
     * Neither the name of the Warsaw University of Technology nor the
       names of its contributors may be used to endorse or promote products
       derived from this software without specific prior written permission.
 
 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "simulation_control_msgs/SimControlAction.h"
#include "simulation_control_msgs/SimControlGoal.h"

#include <rtt/RTT.hpp>
#include <rtt/Component.hpp>
#include <rtt/Logger.hpp>

#include <ros/ros.h>
#include "rtt_actionlib/rtt_actionlib.h"
#include "rtt_actionlib/rtt_action_server.h"

#include "rtt_rosclock/rtt_rosclock.h"

#include <iostream>
#include <string>
#include <map>
#include <math.h>

#include <fcntl.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>

#include "Eigen/Dense"

using namespace RTT;

class SimulationControlActionServer : public RTT::TaskContext {
private:
	typedef actionlib::ServerGoalHandle<simulation_control_msgs::SimControlAction> GoalHandle;
	typedef boost::shared_ptr<const simulation_control_msgs::SimControlGoal> Goal;

	rtt_actionlib::RTTActionServer<simulation_control_msgs::SimControlAction> as_;
	GoalHandle active_goal_;
	simulation_control_msgs::SimControlFeedback feedback_;
	/*
	RTT::InputPort<uint8_t> port_homing_required_in_;
	RTT::InputPort<uint8_t> port_homing_in_progress_in_;
	RTT::InputPort<uint8_t> port_enabled_in_;
    uint8_t homing_required_in_;
    uint8_t homing_in_progress_in_;
    uint8_t enabled_in_;
    bool communication_ok_;

	RTT::OutputPort<uint8_t> port_homing_start_out_;
    RTT::OutputPort<uint8_t>  port_enable_out_;

    bool enable_action_active_;

    ros::Time action_start_time_;
	*/
public:
	explicit SimulationControlActionServer(const std::string& name)
		: TaskContext(name)
        //, enable_action_active_(false)
	{
		/*
		this->ports()->addPort("homing_required_INPORT", port_homing_required_in_);
		this->ports()->addPort("homing_in_progress_INPORT", port_homing_in_progress_in_);
		this->ports()->addPort("enabled_INPORT", port_enabled_in_);
		this->ports()->addPort("homing_start_OUTPORT", port_homing_start_out_);
		this->ports()->addPort("enable_OUTPORT", port_enable_out_);
		*/
		as_.addPorts(this->provides());
		as_.registerGoalCallback(boost::bind(&SimulationControlActionServer::goalCB, this, _1));
		as_.registerCancelCallback(boost::bind(&SimulationControlActionServer::cancelCB, this, _1));
	}

	~SimulationControlActionServer() {
	}

	bool startHook() {
		if (as_.ready()) {
			as_.start();
		} else {
			return false;
		}
		return true;
	}

    void updateHook() {
    	/*
        uint8_t prev_homing_required_in = homing_required_in_;
        uint8_t prev_homing_in_progress_in = homing_in_progress_in_;
        uint8_t prev_enabled_in = enabled_in_;

        if (port_homing_required_in_.read(homing_required_in_) == RTT::NewData &&
                port_homing_in_progress_in_.read(homing_in_progress_in_) == RTT::NewData && port_enabled_in_.read(enabled_in_) == RTT::NewData) {
            communication_ok_ = true;

            if (prev_homing_required_in != homing_required_in_ ||
                    prev_homing_in_progress_in != homing_in_progress_in_ ||
                    prev_enabled_in != enabled_in_) {
                Logger::log() << Logger::Info << getName() << " en: " << (enabled_in_?"t":"f") << ", h.req.: " << (homing_required_in_?"t":"f") << ", h.in progr.: " << (homing_in_progress_in_?"t":"f") << Logger::endl;
            }

            if ((rtt_rosclock::host_now()-action_start_time_).toSec() > 0.5 &&
                    active_goal_.isValid() &&
                    active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE) {
                if (enable_action_active_) {
                    enable_action_active_ = false;
			        motor_action_msgs::MotorResult res;
                    if (enabled_in_) {
    			        res.error_code = motor_action_msgs::MotorResult::SUCCESSFUL;
	    		        active_goal_.setSucceeded(res);
                    }
                    else {
    			        res.error_code = motor_action_msgs::MotorResult::ERROR_UNKNOWN;
	    		        active_goal_.setAborted(res);
                    }
                }
                else {
        			active_goal_.publishFeedback(feedback_);
                    if (!homing_in_progress_in_ && !homing_required_in_) {
				        motor_action_msgs::MotorResult res;
				        res.error_code = motor_action_msgs::MotorResult::SUCCESSFUL;
				        active_goal_.setSucceeded(res);
                    }
                    else if (!homing_in_progress_in_ && homing_required_in_) {
			            motor_action_msgs::MotorResult res;
			            res.error_code = motor_action_msgs::MotorResult::ERROR_UNKNOWN;
			            active_goal_.setAborted(res);
                    }
                }
            }
        }
        else {
            communication_ok_ = false;
        }
        */
	}

private:
	void goalCB(GoalHandle gh) {
		Goal g = gh.getGoal();
		std::cout << "SimulationControlActionServer: received a new goal: " << g->run_steps << std::endl;
		/*
		// cancel active goal
		if (active_goal_.isValid() && (active_goal_.getGoalStatus().status == actionlib_msgs::GoalStatus::ACTIVE)) {
			motor_action_msgs::MotorResult res;
			res.error_code = motor_action_msgs::MotorResult::ERROR_HOMING_IN_PROGRESS;
			gh.setRejected(res);
			return;
		}

        if (!communication_ok_) {
			motor_action_msgs::MotorResult res;
			res.error_code = motor_action_msgs::MotorResult::ERROR_NO_COMMUNICATION;
			gh.setRejected(res);
			return;
        }

		Goal g = gh.getGoal();

        if (g->action == motor_action_msgs::MotorGoal::ACTION_START_HOMING && !homing_required_in_) {
			motor_action_msgs::MotorResult res;
			res.error_code = motor_action_msgs::MotorResult::ERROR_HOMING_DONE;
			gh.setRejected(res);
			return;
        }

        if (homing_in_progress_in_) {
			motor_action_msgs::MotorResult res;
			res.error_code = motor_action_msgs::MotorResult::ERROR_HOMING_IN_PROGRESS;
			gh.setRejected(res);
			return;
        }

        if (g->action == motor_action_msgs::MotorGoal::ACTION_START_HOMING) {
            port_homing_start_out_.write(1);
        }
        else if (g->action == motor_action_msgs::MotorGoal::ACTION_ENABLE) {
            if (enabled_in_) {
			    motor_action_msgs::MotorResult res;
			    res.error_code = motor_action_msgs::MotorResult::ERROR_ALREADY_ENABLED;
			    gh.setRejected(res);
			    return;
            }
            port_enable_out_.write(1);
            enable_action_active_ = true;
        }

        action_start_time_ = rtt_rosclock::host_now();
		gh.setAccepted();
		active_goal_ = gh;
        return;
        */
	}

	void cancelCB(GoalHandle gh) {
		/*
		if (active_goal_ == gh) {
			active_goal_.setCanceled();
		}
		*/
	}
};
ORO_CREATE_COMPONENT(SimulationControlActionServer)

