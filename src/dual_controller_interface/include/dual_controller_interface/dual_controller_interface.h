/*
 * Copyright (c) 2015, Scott K Logan
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _dual_controller_interface_h
#define _dual_controller_interface_h

#include <controller_interface/controller.h>
#include <controller_interface/controller_base.h>

namespace dual_controller_interface
{
	template <class T, class U>
	class DualController : public controller_interface::ControllerBase
	{
	public:
		DualController()
		{
		}

		virtual ~DualController()
		{
		}

		virtual bool init(T *hw_a, U *hw_b, ros::NodeHandle &controller_nh)
		{
			return true;
		};

		virtual bool init(T *hw_a, U *hw_b, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh)
		{
			return true;
		};

	protected:
		virtual bool initRequest(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh, std::set<std::string> &claimed_resources)
		{
			if (state_ != CONSTRUCTED)
			{
				ROS_ERROR("Cannot initialize this controller because it failed to be constructed");
				return false;
			}

			T *hw_a = robot_hw->get<T>();
			if (!hw_a)
			{
				ROS_ERROR("This controller requires a hardware interface of type '%s'. Make sure this is registered in the hardware_interface::RobotHW class.", getHardwareInterfaceTypeA().c_str());
				return false;
			}

			U *hw_b = robot_hw->get<U>();
			if (!hw_a)
			{
				ROS_ERROR("This controller requires a hardware interface of type '%s'. Make sure this is registered in the hardware_interface::RobotHW class.", getHardwareInterfaceTypeB().c_str());
				return false;
			}

			hw_a->clearClaims();
			hw_b->clearClaims();
			if (!init(hw_a, hw_b, controller_nh) || !init(hw_a, hw_b, root_nh, controller_nh))
			{
				ROS_ERROR("Failed to initialize the controller");
				return false;
			}
			claimed_resources = hw_a->getClaims();
			std::set<std::string> claimed_resources_b = hw_b->getClaims();
			claimed_resources.insert(claimed_resources_b.begin(), claimed_resources_b.end());
			hw_a->clearClaims();
			hw_b->clearClaims();

			state_ = INITIALIZED;
			return true;
		}

		virtual std::string getHardwareInterfaceType() const
		{
			return std::string();
		}

		virtual std::string getHardwareInterfaceTypeA() const
		{
			return hardware_interface::internal::demangledTypeName<T>();
		}

		virtual std::string getHardwareInterfaceTypeB() const
		{
			return hardware_interface::internal::demangledTypeName<U>();
		}

	private:
		DualController<T, U>(const DualController<T, U> &c);
		DualController<T, U> & operator =(const DualController<T, U> &c);
	};

}

#endif /* _dual_controller_interface_h */
