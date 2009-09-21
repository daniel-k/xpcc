// coding: utf-8
// ----------------------------------------------------------------------------
/* Copyright (c) 2009, Roboterclub Aachen e.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Roboterclub Aachen e.V. nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ROBOTERCLUB AACHEN E.V. ''AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ROBOTERCLUB AACHEN E.V. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 */
// ----------------------------------------------------------------------------
#ifndef XPCC_LOG_SINGLETON__HPP
#define XPCC_LOG_SINGLETON__HPP


#include "log_device.hpp"
#include "log_level.hpp"
#include "../../hal/io/iodevice.hpp"

namespace xpcc {
	/**
	 * @class 	LogSingleton
	 * @brief 	This singleton, connect the Logger to the LogDevice.
	 *
	 * @ingroup logger
	 * @version	$Id$
	 * @since 	01 December 2006
	 * @author	Christofer Hedbrand,
	 * 			Carsten Schmidt,
	 * 			Martin Rosekeit <martin.rosekeit@rwth-aachen.de>
	 */
	class LogSingleton : public IODevice {
		public:
			static LogSingleton&
			instance();

			//! Set a new log device in the singleton.
			//!
			//! The singleton takes ownership of the device (the
			//! singleton will delete the object)
			//!
			//! \code
			//! 	xpcc::LogSingleton::instance().setLogDevice(new MyLogDevice);
			//! \endcode
			void
			setLogDevice(LogDevice* device);

			//! Set a new log device writer in the singleton.
			//!
			//! The singleton takes \b not ownership of the device (the
			//! singleton will \b not delete the object)
			//!
			//! \code
			//!		MyLogDevice device;
			//! 	xpcc::LogSingleton::instance().setLogDevice(device);
			//! \endcode
			void
			setLogDevice(LogDevice& device);

			void
			setLevel(Log::Level level);

			inline const Log::Level&
			getLevel() const
			{
				return this->level;
			}

			virtual void
			put(char c);

			virtual void
			put(const char* s);

			virtual void
			flush();

			virtual bool
			get(char& value);

		private:
			LogSingleton();								// Private constructor because of singleton
			LogSingleton(const LogSingleton & logger);	// Private copy constructor because of singleton
			~LogSingleton();

			LogDevice*		device;
			bool			ownDevice;
			Log::Level 		level;
	};

	//! Set a new log device in the LogSingleton.
	//!
	//! The singleton takes ownership of the device (the
	//! singleton will delete the object)
	//!
	//! \code
	//! 	xpcc::setLogDevice(new MyLogDevice);
	//! \endcode
	//!
	//! This is a shortcut to call
	//! \code
	//! 	xpcc::LogSingleton::instance().setLogDevice(new MyLogDevice);
	//! \endcode
	inline void
	setLogDevice(LogDevice* device)
	{
		LogSingleton::instance().setLogDevice(device);
	}

	//! Set a new log device writer in the LogSingleton.
	//!
	//! The singleton takes \b not ownership of the device (the
	//! singleton will \b not delete the object)
	//!
	//! \code
	//!		MyLogDevice device;
	//! 	xpcc::setLogDevice(device);
	//! \endcode
	//!
	//! This is a shortcut to call
	//! \code
	//!		MyLogDevice device;
	//! 	LogSingleton::instance().setLogDevice(device);
	//! \endcode
	inline void
	setLogDevice(LogDevice& device)
	{
		LogSingleton::instance().setLogDevice(device);
	}
};


#endif /* XPCC_LOG_SINGLETON__HPP */
