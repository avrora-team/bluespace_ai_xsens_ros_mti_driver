// BSD 3-Clause License
//
// Copyright (c) 2021, BlueSpace.ai, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from
//    this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

//  Copyright (c) 2003-2021 Xsens Technologies B.V. or subsidiaries worldwide.
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without modification,
//  are permitted provided that the following conditions are met:
//  
//  1.	Redistributions of source code must retain the above copyright notice,
//  	this list of conditions, and the following disclaimer.
//  
//  2.	Redistributions in binary form must reproduce the above copyright notice,
//  	this list of conditions, and the following disclaimer in the documentation
//  	and/or other materials provided with the distribution.
//  
//  3.	Neither the names of the copyright holders nor the names of their contributors
//  	may be used to endorse or promote products derived from this software without
//  	specific prior written permission.
//  
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
//  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
//  MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL
//  THE COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//  SPECIAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT 
//  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
//  HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY OR
//  TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
//  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.THE LAWS OF THE NETHERLANDS 
//  SHALL BE EXCLUSIVELY APPLICABLE AND ANY DISPUTES SHALL BE FINALLY SETTLED UNDER THE RULES 
//  OF ARBITRATION OF THE INTERNATIONAL CHAMBER OF COMMERCE IN THE HAGUE BY ONE OR MORE 
//  ARBITRATORS APPOINTED IN ACCORDANCE WITH SAID RULES.
//  

#include "xdacallback.h"

#include <xscontroller/xsdevice_def.h>
#include <xstypes/xsdatapacket.h>

XdaCallback::XdaCallback(rclcpp::Node& node, size_t maxBufferSize)
	: m_maxBufferSize(maxBufferSize)
	, parent_node(node)
{
    parent_node.declare_parameter<bool>("use_utc_time");
    parent_node.declare_parameter<bool>("require_utc_valid");

    use_utc_time = false;
    require_utc_valid = true;
    parent_node.get_parameter("use_utc_time", use_utc_time);
    parent_node.get_parameter("require_utc_valid", require_utc_valid);

    if (use_utc_time) {
        RCLCPP_INFO(parent_node.get_logger(), "Using UTC time.");
    }
    if (!require_utc_valid) {
      RCLCPP_INFO(parent_node.get_logger(), "UTC valid flag is not required.");
    }
}

XdaCallback::~XdaCallback() throw()
{
}

// Returns empty packet on timeout
RosXsDataPacket XdaCallback::next(const std::chrono::milliseconds &timeout)
{
	RosXsDataPacket packet;

	std::unique_lock<std::mutex> lock(m_mutex);

	if (m_condition.wait_for(lock, timeout, [&] { return !m_buffer.empty(); }))
	{
		assert(!m_buffer.empty());

		packet = m_buffer.front();
		m_buffer.pop_front();
	}

	return packet;
}

void XdaCallback::onLiveDataAvailable(XsDevice *, const XsDataPacket *packet)
{
    rclcpp::Time packet_time;

    if (use_utc_time) {
        if (!packet->containsUtcTime()) {
            RCLCPP_INFO_THROTTLE(parent_node.get_logger(), *parent_node.get_clock(), 1000, "Packet doesn't contains UTC time. Check device or driver configuration. Skipping data");
            return;
        }

        auto utc_time = packet->utcTime();

        if (!(utc_time.m_valid & 0b100) && require_utc_valid) {
            RCLCPP_INFO_THROTTLE(parent_node.get_logger(), *parent_node.get_clock(), 1000, "UTC time is not valid. Skipping data");
            return;
        }

        using namespace std::chrono;
        year_month_day ymd{year(utc_time.m_year),
                           month(utc_time.m_month),
                           day(utc_time.m_day)};

        hh_mm_ss hms{hours(utc_time.m_hour) + minutes(utc_time.m_minute) + seconds(utc_time.m_second)};

        auto seconds_since_epoch = (sys_days(ymd) + seconds(hms)).time_since_epoch().count();

        packet_time = rclcpp::Time(seconds_since_epoch, utc_time.m_nano);
    } else {
        packet_time = parent_node.now();
    }

	std::unique_lock<std::mutex> lock(m_mutex);

	assert(packet != 0);

	// Discard oldest packet if buffer full
	if (m_buffer.size() == m_maxBufferSize)
	{
		m_buffer.pop_front();
	}

	// Push new packet
	m_buffer.emplace_back(packet_time, *packet);

	// Manual unlocking is done before notifying, to avoid waking up
	// the waiting thread only to block again
	lock.unlock();
	m_condition.notify_one();
}
