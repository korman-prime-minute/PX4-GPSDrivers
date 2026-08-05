/****************************************************************************
 *
 *   Copyright (c) 2012-2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ubx.cpp
 *
 * U-Blox protocol implementation. Following u-blox 6/7/8/9 Receiver Description
 * including Prototol Specification.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 * @author Beat Kueng <beat-kueng@gmx.net>
 *
 * @author Hannes Delago
 *   (rework, add ubx7+ compatibility)
 *
 * @see https://www.u-blox.com/sites/default/files/products/documents/u-blox6-GPS-GLONASS-QZSS-V14_ReceiverDescrProtSpec_%28GPS.G6-SW-12013%29_Public.pdf
 * @see https://www.u-blox.com/sites/default/files/products/documents/u-blox8-M8_ReceiverDescrProtSpec_%28UBX-13003221%29_Public.pdf
 * @see https://www.u-blox.com/sites/default/files/ZED-F9P_InterfaceDescription_%28UBX-18010854%29.pdf
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>

#include "rtcm.h"
#include "ubx.h"
#include "crc.h"

// CPU time
#include <drivers/drv_hrt.h>

/* prime_console_log is provided only by the safety_controller parent repo.
 * Guard the include so this "devices" submodule still compiles under a parent
 * that lacks the header; PRIME_LOG then falls back to UBX_INFO below. */
#if defined(__has_include)
#  if __has_include(<prime_console_log/prime_console_log.h>)
#    include <prime_console_log/prime_console_log.h>
#  endif
#endif

/* RTCM/RTK diagnostic logging throttle intervals [microseconds] */
static constexpr hrt_abstime UBX_RTCM_LOG_SUMMARY_INTERVAL = 5 * 1000 * 1000;
static constexpr hrt_abstime UBX_RTCM_LOG_ABSENT_INTERVAL  = 10 * 1000 * 1000;

/* CSV prefixes for logging */
static constexpr char UBX_NAV_DOP_PREFIX[] = "DOP";
static constexpr char UBX_NAV_PVT_PREFIX[] = "PVT";
static constexpr char UBX_RXM_RTCM_PREFIX[] = "RTM";

#define MIN(X,Y)              ((X) < (Y) ? (X) : (Y))
#define SWAP16(X)             ((((X) >>  8) & 0x00ff) | (((X) << 8) & 0xff00))

/**** Trace macros, disable for production builds */
#define UBX_TRACE_PARSER(...) {/*GPS_INFO(__VA_ARGS__);*/}    // decoding progress in parse_char()
#define UBX_TRACE_RXMSG(...)  {/*GPS_INFO(__VA_ARGS__);*/}    // Rx msgs in payload_rx_done()
#define UBX_TRACE_SVINFO(...) {/*GPS_INFO(__VA_ARGS__);*/}    // NAV-SVINFO processing (debug use only, will cause rx buffer overflows)

/**** Warning macros, disable to save memory */
#define UBX_WARN(...)         {GPS_WARN(__VA_ARGS__);}
#define UBX_DEBUG(...)        {GPS_WARN(__VA_ARGS__);}
#define UBX_ERR(...)          {GPS_ERR(__VA_ARGS__);}
#define UBX_INFO(...)         {GPS_INFO(__VA_ARGS__);}

/* Non-blocking [UBXCFG]-tagged process log for the SD config-file / RAM-dump feature.
 * Mirrored to console + UART5 for GUI-side sharing; safe in the RX path. */
/* PRIME_LOG is a parent-repo-provided non-blocking logger (console + UART mirror).
 * When this "devices" submodule is built by a parent that lacks it, fall back to
 * UBX_INFO so the direct PRIME_LOG(...) call sites below still compile. */
#if !defined(PRIME_LOG)
#define PRIME_LOG(fmt, ...)   UBX_INFO(fmt, ##__VA_ARGS__)
#endif

#define UBXCFG_LOG(fmt, ...)  PRIME_LOG("[UBXCFG] " fmt "\r\n", ##__VA_ARGS__)

GPSDriverUBX::GPSDriverUBX(Interface gpsInterface, GPSCallbackPtr callback, void *callback_user,
			   sensor_gps_s *gps_position, satellite_info_s *satellite_info, uint8_t dynamic_model,
			   float heading_offset, int32_t uart2_baudrate, UBXMode mode, float pvt_warn_rate_hz,
			   bool cfg_file_enabled) :
	GPSBaseStationSupport(callback, callback_user),
	_interface(gpsInterface),
	_gps_position(gps_position),
	_satellite_info(satellite_info),
	_dyn_model(dynamic_model),
	_mode(mode),
	_heading_offset(heading_offset),
	_uart2_baudrate(uart2_baudrate),
	_nav_pvt_warn_period_us(pvt_warn_rate_hz > 0.f ? (uint32_t)(1e6f / pvt_warn_rate_hz) : 0),
	_cfg_file_enabled(cfg_file_enabled)
{
	/* Emit CSV headers for UBX messages (PVT, DOP) once at driver construction */
	// PX4_INFO_RAW("PVT,now_us,iTOW,year,month,day,hour,min,sec,valid,tAcc,nano,fixType,flags,numSV,lon,lat,height,hMSL,hAcc,vAcc,velN,velE,velD,gSpeed,headMot,sAcc,headAcc,pDOP,headVeh\r\n");
	// PX4_INFO_RAW("DOP,now_us,iTOW,gDOP,pDOP,tDOP,vDOP,hDOP,nDOP,eDOP\r\n");
	// PX4_INFO_RAW("RTM,now_us,version,flags,subType,refStationID,msgType\r\n");
	decodeInit();
}

GPSDriverUBX::~GPSDriverUBX()
{
	delete _rtcm_parsing;
}

int
GPSDriverUBX::configure(unsigned &baudrate, const GPSConfig &config)
{
	_configured = false;
	_output_mode = config.output_mode;

	ubx_payload_tx_cfg_prt_t cfg_prt[2];

	uint16_t out_proto_mask = _output_mode == OutputMode::GPS ?
				  UBX_TX_CFG_PRT_PROTO_UBX :
				  (UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM);

	uint16_t in_proto_mask = (_output_mode == OutputMode::GPS || _output_mode == OutputMode::GPSAndRTCM) ?
				 (UBX_TX_CFG_PRT_PROTO_UBX | UBX_TX_CFG_PRT_PROTO_RTCM) :
				 UBX_TX_CFG_PRT_PROTO_UBX;

	const bool auto_baudrate = baudrate == 0;
	const uint32_t DEFAULT_BAUDRATE = 115200;

	if (_interface == Interface::UART) {

		/* try different baudrates */
		const unsigned baudrates[] = {DEFAULT_BAUDRATE};

		unsigned baud_i;
		unsigned desired_baudrate = auto_baudrate ? UBX_BAUDRATE_M8_AND_NEWER : baudrate;

		if ((_mode == UBXMode::RoverWithMovingBaseUART1) || (_mode == UBXMode::MovingBaseUART1)) {
			desired_baudrate = UART1_BAUDRATE_HEADING;
		}

		for (baud_i = 0; baud_i < sizeof(baudrates) / sizeof(baudrates[0]); baud_i++) {
			unsigned test_baudrate = baudrates[baud_i];

			if (!auto_baudrate && baudrate != test_baudrate) {
				continue; // skip to next baudrate
			}

			UBX_DEBUG("baudrate set to %i", test_baudrate);

			setBaudrate(test_baudrate);

			/* flush input and wait for at least 20 ms silence */
			decodeInit();
			receive(20);
			decodeInit();

			// === DISABLED PERIPHERAL CONFIG WRITE ===
			// Originally: probe CFG-VALSET (proto v27+) by configuring UART1 framing
			// (1 stop bit, 8 data bits, no parity) + enable UBX in/out and disable NMEA.
			// Useful when host wants the receiver's UART1 to speak pure UBX so the
			// parser is not polluted by NMEA sentences. Disabled to leave UART1
			// framing/protocols at whatever the receiver was provisioned with.
			// try CFG-VALSET: if we get an ACK we know we can use protocol version 27+
			int cfg_valset_msg_size = initCfgValset();
			(void)cfg_valset_msg_size; // unused once writes commented out
			// cfg_valset_msg_size = initCfgValset();
			// // UART1
			// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_STOPBITS, 1, cfg_valset_msg_size);
			// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_DATABITS, 0, cfg_valset_msg_size);
			// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1_PARITY, 0, cfg_valset_msg_size);
			// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1, cfg_valset_msg_size);
			// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0, cfg_valset_msg_size);
			// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);
			// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_NMEA, 0, cfg_valset_msg_size);
			// // TODO: are we ever connected to UART2?

			// Note: USB protocol settings are handled later in the configureDevice function.

			bool cfg_valset_success = true;

			// if (sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {

			// 	// Note: The M10 comes up sending NMEA sentences at 9600. It can't
			// 	// respond with an ACK until the current sentence has completed transmission.
			// 	// This can take over a second so need a large timeout on this particular wait.
			// 	// Once it has acked this it will turn off the NMEA sentences and all is good
			// 	// for future transactions.
			// 	if (waitForAck(UBX_MSG_CFG_VALSET, 2000, true) == 0) {
			// 		cfg_valset_success = true;
			// 	}
			// }

			if (cfg_valset_success) {
				_proto_ver_27_or_higher = true;
				// === DISABLED PERIPHERAL CONFIG WRITE ===
				// Originally: CFG-VALSET UBX_CFG_KEY_CFG_UART1_BAUDRATE = 57600 — set
				// the receiver's UART1 baud rate so host and receiver agree on link
				// speed. Useful when the receiver is currently at a different baud
				// than the host expects. Disabled: assume receiver is already at
				// the link baud (host-side setBaudrate() below still runs).
				// Now we only have to change the baudrate
				// cfg_valset_msg_size = initCfgValset();
				desired_baudrate = DEFAULT_BAUDRATE;
				// cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART1_BAUDRATE, desired_baudrate, cfg_valset_msg_size);
				//
				// if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
				// 	continue;
				// }
				//
				// /* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
				// waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

			} else {
				_proto_ver_27_or_higher = false;

				UBX_DEBUG("trying old protocol");

				// === DISABLED PERIPHERAL CONFIG WRITE ===
				// Originally (pre-v27 fallback): two CFG-PRT messages.
				// 1st - tell receiver's UART1 + USB ports to accept UBX (and RTCM
				//       when running as RTCM in/out) and emit only UBX/RTCM, at
				//       the currently-probed baud rate.
				// 2nd - change the receiver's UART1/USB baud rate to desired_baudrate.
				// Useful on legacy u-blox 5/6/7/8 modules that lack the
				// configuration database. Disabled to leave port settings as
				// provisioned; we still suppress this code path by suppressing
				// re-probing, and (void) avoids unused-var warnings.
				(void)cfg_prt; (void)in_proto_mask; (void)out_proto_mask; (void)test_baudrate;
				// memset(cfg_prt, 0, 2 * sizeof(ubx_payload_tx_cfg_prt_t));
				// cfg_prt[0].portID       = UBX_TX_CFG_PRT_PORTID;
				// cfg_prt[0].mode         = UBX_TX_CFG_PRT_MODE;
				// cfg_prt[0].baudRate     = test_baudrate;
				// cfg_prt[0].inProtoMask  = in_proto_mask;
				// cfg_prt[0].outProtoMask = out_proto_mask;
				// cfg_prt[1].portID       = UBX_TX_CFG_PRT_PORTID_USB;
				// cfg_prt[1].mode         = UBX_TX_CFG_PRT_MODE;
				// cfg_prt[1].baudRate     = test_baudrate;
				// cfg_prt[1].inProtoMask  = in_proto_mask;
				// cfg_prt[1].outProtoMask = out_proto_mask;
				//
				// if (!sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, 2 * sizeof(ubx_payload_tx_cfg_prt_t))) {
				// 	continue;
				// }
				//
				// if (waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false) < 0) {
				// 	/* try next baudrate */
				// 	continue;
				// }

				if (auto_baudrate) {
					desired_baudrate = UBX_TX_CFG_PRT_BAUDRATE;
				}

				// /* Send a CFG-PRT message again, this time change the baudrate */
				// cfg_prt[0].baudRate = desired_baudrate;
				// cfg_prt[1].baudRate = desired_baudrate;
				//
				// if (!sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, 2 * sizeof(ubx_payload_tx_cfg_prt_t))) {
				// 	continue;
				// }
				//
				// /* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
				// waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);
			}

			if (desired_baudrate != test_baudrate) {
				setBaudrate(desired_baudrate);

				decodeInit();
				receive(20);
				decodeInit();
			}

			/* at this point we have correct baudrate on both ends */
			baudrate = desired_baudrate;
			break;
		}

		if (baud_i >= sizeof(baudrates) / sizeof(baudrates[0])) {
			return -1;	// connection and/or baudrate detection failed
		}

	} else if (_interface == Interface::SPI) {

		// === DISABLED PERIPHERAL CONFIG WRITE ===
		// Originally: CFG-VALSET probe over SPI to enable the SPI peripheral on
		// the receiver, set the SPI MAXFF (max number of trailing 0xFF before
		// the receiver stops sending filler), and enable UBX (+RTCM in/out per
		// _output_mode) while disabling NMEA. Useful to make the receiver's SPI
		// port speak only the protocols this driver parses. Disabled: leaves
		// SPI port protocol set as provisioned. Assume v27+ so the rest of the
		// driver follows the modern code path (matches UART branch behavior).
		// int cfg_valset_msg_size = initCfgValset();
		// cfgValset<uint8_t>(UBX_CFG_KEY_SPI_ENABLED, 1, cfg_valset_msg_size);
		// cfgValset<uint8_t>(UBX_CFG_KEY_SPI_MAXFF, 1, cfg_valset_msg_size);
		// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIINPROT_UBX, 1, cfg_valset_msg_size);
		// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIINPROT_RTCM3X, _output_mode == OutputMode::RTCM ? 0 : 1, cfg_valset_msg_size);
		// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIINPROT_NMEA, 0, cfg_valset_msg_size);
		// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIOUTPROT_UBX, 1, cfg_valset_msg_size);
		// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIOUTPROT_RTCM3X, _output_mode == OutputMode::GPS ? 0 : 1, cfg_valset_msg_size);
		// cfgValset<uint8_t>(UBX_CFG_KEY_CFG_SPIOUTPROT_NMEA, 0, cfg_valset_msg_size);
		//
		// bool cfg_valset_success = false;
		//
		// if (sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		// 	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) == 0) {
		// 		cfg_valset_success = true;
		// 	}
		// }
		//
		// if (cfg_valset_success) {
		// 	_proto_ver_27_or_higher = true;
		// } else {
		// === DISABLED PERIPHERAL CONFIG WRITE (pre-v27 SPI fallback) ===
		// Originally: CFG-PRT on the SPI port to set SPI mode bits and the
		// in/out protocol masks (UBX, optionally RTCM3X). Same purpose as the
		// CFG-VALSET above, for receivers without the v27 config DB.
		// 	_proto_ver_27_or_higher = false;
		// 	memset(cfg_prt, 0, sizeof(ubx_payload_tx_cfg_prt_t));
		// 	cfg_prt[0].portID       = UBX_TX_CFG_PRT_PORTID_SPI;
		// 	cfg_prt[0].mode         = UBX_TX_CFG_PRT_MODE_SPI;
		// 	cfg_prt[0].inProtoMask  = in_proto_mask;
		// 	cfg_prt[0].outProtoMask = out_proto_mask;
		//
		// 	if (!sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, sizeof(ubx_payload_tx_cfg_prt_t))) {
		// 		return -1;
		// 	}
		//
		// 	waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);
		// }
		_proto_ver_27_or_higher = true; // assume modern receiver; no probe sent

	} else {
		return -1;
	}

	// UBX_DEBUG("Protocol version 27+: %i", static_cast<int>(_proto_ver_27_or_higher));

	/* Request module version information by sending an empty MON-VER message */
	// if (!sendMessage(UBX_MSG_MON_VER, nullptr, 0)) {
	// 	return -1;
	// }

	// /* Wait for the reply so that we know to which device we're connected (_board will be set).
	//  * Note: we won't actually get an ACK-ACK, but UBX_MSG_MON_VER will also set the ack state.
	//  */
	// if (waitForAck(UBX_MSG_MON_VER, UBX_CONFIG_TIMEOUT, true) < 0) {
	// 	return -1;
	// }


	// === DISABLED PERIPHERAL CONFIG WRITE ===
	// Originally: on u-blox 8 modules in auto-baud mode, push the receiver's
	// UART1 + USB baud rate up to the M8+ default (CFG-PRT). Useful to step up
	// from the initial slow probe baud to a higher link rate now that the
	// board is identified. Disabled - we keep whatever baud the link probe
	// settled on and do not touch the receiver's port config.
	// /* Now that we know the board, update the baudrate on M8 boards (on F9+ we already used the
	//  * higher baudrate with CFG-VALSET) */
	// if (_interface == Interface::UART && auto_baudrate && _board == Board::u_blox8) {
	//
	// 	cfg_prt[0].baudRate = UBX_BAUDRATE_M8_AND_NEWER;
	// 	cfg_prt[1].baudRate = UBX_BAUDRATE_M8_AND_NEWER;
	//
	// 	if (sendMessage(UBX_MSG_CFG_PRT, (uint8_t *)cfg_prt, 2 * sizeof(ubx_payload_tx_cfg_prt_t))) {
	// 		/* no ACK is expected here, but read the buffer anyway in case we actually get an ACK */
	// 		waitForAck(UBX_MSG_CFG_PRT, UBX_CONFIG_TIMEOUT, false);
	//
	// 		setBaudrate(UBX_BAUDRATE_M8_AND_NEWER);
	// 		baudrate = UBX_BAUDRATE_M8_AND_NEWER;
	// 	}
	// }

	/* Provision the receiver from a config file on the SD card, if enabled (GPS_UBX_CFGFILE).
	 * Runs once here, after the link baud is locked and the device is responsive, but before
	 * output-mode/RTCM setup. Failures never abort bring-up (see loadConfigFromFile). UART only:
	 * the config file targets the F9P's own port config which is set over the host UART link. */
	if (_cfg_file_enabled && _interface == Interface::UART) {
		loadConfigFromFile(UBX_CFGFILE_PATH);
	}

	if (_output_mode == OutputMode::GPSAndRTCM || _output_mode == OutputMode::RTCM || _mode == UBXMode::MovingBaseUART1) {
		if (!_rtcm_parsing) {
			_rtcm_parsing = new RTCMParsing();
		}

		_rtcm_parsing->reset();
	}

	if (_output_mode == OutputMode::RTCM) {
		// RTCM mode force stationary dynamic model
		_dyn_model = 2;
	}

	int ret;

	if (_proto_ver_27_or_higher) {
		UBX_DEBUG("configureDevice disabled!");
		//ret = configureDevice(config, _uart2_baudrate);
		_use_nav_pvt = true;
		ret = 0;

	} else {
		ret = configureDevicePreV27(config.gnss_systems);
	}

	if (ret != 0) {
		return ret;
	}

	if (_output_mode == OutputMode::RTCM) {
		if (restartSurveyIn() < 0) {
			return -1;
		}

	} else if (_output_mode == OutputMode::GPSAndRTCM) {
		if (activateRTCMOutput(false) < 0) {
			return -1;
		}
	}

	_configured = true;
	return 0;
}


int GPSDriverUBX::configureDevicePreV27(const GNSSSystemsMask &gnssSystems)
{
	// === DISABLED PERIPHERAL CONFIG WRITES (entire pre-v27 init) ===
	// Originally configures legacy u-blox 6/7/8 receivers via the old
	// per-message UBX-CFG-* commands:
	//   * CFG-RATE   : measurement + navigation solution rate, and time reference
	//                  (GPS/GLONASS/UTC). Sets how often the receiver computes a fix.
	//   * CFG-NAV5   : navigation engine settings - dynamic platform model
	//                  (airborne/automotive/etc.), 2D/3D fix mode, masks. Tunes
	//                  Kalman filter behavior for the expected motion profile.
	//   * CFG-GNSS   : which constellations + signals to track (GPS+QZSS, SBAS,
	//                  Galileo, BeiDou, GLONASS, IMES) and channel allocations.
	//                  Useful to enable/disable specific GNSS systems.
	//   * CFG-MSG    : per-message output-rate selectors for NAV-PVT/POSLLH/SOL/
	//                  VELNED/TIMEUTC/STATUS/DOP/SVINFO and MON-HW. Tells the
	//                  receiver which solution/status messages to stream and how
	//                  often (divisor of measurement rate).
	// All of these are useful when you want PX4 to dictate fix rate, motion
	// model, constellation mix, and message stream. Disabled here so the
	// receiver keeps whatever rate/model/constellations/messages were
	// provisioned externally (e.g. via u-center).
	(void)gnssSystems;
	return 0;
#if 0
	/* Send a CFG-RATE message to define update rate */
	memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
	_buf.payload_tx_cfg_rate.measRate	= UBX_TX_CFG_RATE_MEASINTERVAL;
	_buf.payload_tx_cfg_rate.navRate	= UBX_TX_CFG_RATE_NAVRATE;
	_buf.payload_tx_cfg_rate.timeRef	= UBX_TX_CFG_RATE_TIMEREF;

	if (!sendMessage(UBX_MSG_CFG_RATE, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rate))) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_RATE, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	/* send a NAV5 message to set the options for the internal filter */
	memset(&_buf.payload_tx_cfg_nav5, 0, sizeof(_buf.payload_tx_cfg_nav5));
	_buf.payload_tx_cfg_nav5.mask		= UBX_TX_CFG_NAV5_MASK;
	_buf.payload_tx_cfg_nav5.dynModel	= _dyn_model;
	_buf.payload_tx_cfg_nav5.fixMode	= UBX_TX_CFG_NAV5_FIXMODE;

	if (!sendMessage(UBX_MSG_CFG_NAV5, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_nav5))) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_NAV5, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	/* configure active GNSS systems (number of channels and used signals taken from U-Center default) */
	if (static_cast<int32_t>(gnssSystems) != 0) {
		memset(&_buf.payload_tx_cfg_gnss, 0, sizeof(_buf.payload_tx_cfg_gnss));
		_buf.payload_tx_cfg_gnss.msgVer = 0x00;
		_buf.payload_tx_cfg_gnss.numTrkChHw = 0x00;  // read only
		_buf.payload_tx_cfg_gnss.numTrkChUse = 0xFF;  // use max number of HW channels
		_buf.payload_tx_cfg_gnss.numConfigBlocks = 7;  // always configure all systems

		// GPS and QZSS should always be enabled and disabled together, according to uBlox
		_buf.payload_tx_cfg_gnss.block[0].gnssId = UBX_TX_CFG_GNSS_GNSSID_GPS;
		_buf.payload_tx_cfg_gnss.block[1].gnssId = UBX_TX_CFG_GNSS_GNSSID_QZSS;

		if (gnssSystems & GNSSSystemsMask::ENABLE_GPS) {
			UBX_DEBUG("GNSS Systems: Use GPS + QZSS");
			_buf.payload_tx_cfg_gnss.block[0].resTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[0].maxTrkCh = 16;
			_buf.payload_tx_cfg_gnss.block[0].flags = UBX_TX_CFG_GNSS_FLAGS_GPS_L1CA | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
			_buf.payload_tx_cfg_gnss.block[1].resTrkCh = 0;
			_buf.payload_tx_cfg_gnss.block[1].maxTrkCh = 3;
			_buf.payload_tx_cfg_gnss.block[1].flags = UBX_TX_CFG_GNSS_FLAGS_QZSS_L1CA | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[2].gnssId = UBX_TX_CFG_GNSS_GNSSID_SBAS;

		if (gnssSystems & GNSSSystemsMask::ENABLE_SBAS) {
			UBX_DEBUG("GNSS Systems: Use SBAS");
			_buf.payload_tx_cfg_gnss.block[2].resTrkCh = 1;
			_buf.payload_tx_cfg_gnss.block[2].maxTrkCh = 3;
			_buf.payload_tx_cfg_gnss.block[2].flags = UBX_TX_CFG_GNSS_FLAGS_SBAS_L1CA | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[3].gnssId = UBX_TX_CFG_GNSS_GNSSID_GALILEO;

		if (gnssSystems & GNSSSystemsMask::ENABLE_GALILEO) {
			UBX_DEBUG("GNSS Systems: Use Galileo");
			_buf.payload_tx_cfg_gnss.block[3].resTrkCh = 4;
			_buf.payload_tx_cfg_gnss.block[3].maxTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[3].flags = UBX_TX_CFG_GNSS_FLAGS_GALILEO_E1 | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[4].gnssId = UBX_TX_CFG_GNSS_GNSSID_BEIDOU;

		if (gnssSystems & GNSSSystemsMask::ENABLE_BEIDOU) {
			UBX_DEBUG("GNSS Systems: Use BeiDou");
			_buf.payload_tx_cfg_gnss.block[4].resTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[4].maxTrkCh = 16;
			_buf.payload_tx_cfg_gnss.block[4].flags = UBX_TX_CFG_GNSS_FLAGS_BEIDOU_B1I | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		_buf.payload_tx_cfg_gnss.block[5].gnssId = UBX_TX_CFG_GNSS_GNSSID_GLONASS;

		if (gnssSystems & GNSSSystemsMask::ENABLE_GLONASS) {
			UBX_DEBUG("GNSS Systems: Use GLONASS");
			_buf.payload_tx_cfg_gnss.block[5].resTrkCh = 8;
			_buf.payload_tx_cfg_gnss.block[5].maxTrkCh = 14;
			_buf.payload_tx_cfg_gnss.block[5].flags = UBX_TX_CFG_GNSS_FLAGS_GLONASS_L1 | UBX_TX_CFG_GNSS_FLAGS_ENABLE;
		}

		// IMES always disabled
		_buf.payload_tx_cfg_gnss.block[6].gnssId = UBX_TX_CFG_GNSS_GNSSID_IMES;
		_buf.payload_tx_cfg_gnss.block[6].flags = 0;

		// send message
		if (!sendMessage(UBX_MSG_CFG_GNSS, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_gnss))) {
			UBX_DEBUG("UBX CFG-GNSS message send failed");
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_GNSS, UBX_CONFIG_TIMEOUT, true) < 0) {
			UBX_DEBUG("UBX CFG-GNSS message ACK failed");
			return -1;
		}
	}

	/* configure message rates */
	/* the last argument is divisor for measurement rate (set by CFG RATE), i.e. 1 means 5Hz */

	/* try to set rate for NAV-PVT */
	/* (implemented for ubx7+ modules only, use NAV-SOL, NAV-POSLLH, NAV-VELNED and NAV-TIMEUTC for ubx6) */
	if (!configureMessageRate(UBX_MSG_NAV_PVT, 1)) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, true) < 0) {
		_use_nav_pvt = false;

	} else {
		_use_nav_pvt = true;
	}

	UBX_DEBUG("%susing NAV-PVT", _use_nav_pvt ? "" : "not ");

	if (!_use_nav_pvt) {
		if (!configureMessageRateAndAck(UBX_MSG_NAV_TIMEUTC, 5, true)) {
			return -1;
		}

		if (!configureMessageRateAndAck(UBX_MSG_NAV_POSLLH, 1, true)) {
			return -1;
		}

		if (!configureMessageRateAndAck(UBX_MSG_NAV_SOL, 1, true)) {
			return -1;
		}

		if (!configureMessageRateAndAck(UBX_MSG_NAV_VELNED, 1, true)) {
			return -1;
		}
	}

	if (!configureMessageRateAndAck(UBX_MSG_NAV_STATUS, 1, true)) {
		return -1;
	}

	if (!configureMessageRateAndAck(UBX_MSG_NAV_DOP, 1, true)) {
		return -1;
	}

	if (!configureMessageRateAndAck(UBX_MSG_NAV_SVINFO, (_satellite_info != nullptr) ? 5 : 0, true)) {
		return -1;
	}

	if (!configureMessageRateAndAck(UBX_MSG_MON_HW, 1, true)) {
		return -1;
	}

	return 0;
#endif // configureDevicePreV27 body disabled
}

int GPSDriverUBX::configureDevice(const GPSConfig &config, const int32_t uart2_baudrate)
{
	// === DISABLED PERIPHERAL CONFIG WRITES (entire v27+ init) ===
	// Originally configures modern u-blox 9/10/F9P receivers via CFG-VALSET
	// over the v27 configuration database. The full pipeline:
	//   * Port protocols (CFG-*INPROT/OUTPROT) for UART1, USB, I2C - controls
	//     which protocols (UBX / RTCM3 / NMEA) the receiver accepts and emits
	//     on each physical port. Useful to silence NMEA and gate RTCM in/out.
	//   * NAVSPG-FIXMODE / NAVSPG-UTCSTANDARD / NAVSPG-DYNMODEL - 2D/3D mode
	//     selection, UTC standard (USNO/GPS/etc.), dynamic-platform model.
	//     Tunes the navigation engine for the expected vehicle dynamics.
	//   * ODO_* - odometer + low-pass filters on velocity/COG. Disabled for
	//     unfiltered raw outputs.
	//   * RATE_MEAS / RATE_NAV / RATE_TIMEREF - measurement period, nav rate,
	//     time reference (e.g. 100 ms -> 10 Hz fix; F9P up to 20 Hz).
	//   * NAVHPG_DGNSSMODE - select RTK fixed/float. Required to get fixed
	//     carrier-phase RTK fixes on F9P.
	//   * ITFM_ENABLE - jamming/interference monitor (drives jamming_indicator
	//     in MON-HW/MON-RF). Useful for interference diagnostics.
	//   * SIGNAL_*_ENA - per-band signal selectors (GPS L1/L2/L5, GAL E1/E5A/E5B,
	//     BDS B1/B2/B2A, GLO L1, QZSS, SBAS L1, NavIC L5). Controls which
	//     constellation/band combinations the receiver tracks.
	//   * MSGOUT_UBX_NAV_*_I2C - per-message stream rates on the
	//     I2C/UART/USB port (NAV-PVT, NAV-DOP, NAV-SAT, NAV-STATUS, NAV-HPPOSLLH,
	//     NAV-RELPOSNED, MON-RF, RXM-RTCM). cfgValsetPort() fans the same key
	//     out to UART1/USB or SPI. Tells the receiver which solution/status
	//     messages to emit and how often.
	//   * UART2 protocol/baud + RTCM type 1005/1074/1084/1094/1124/4072 outputs
	//     (moving-base or static-base modes). Lets the receiver feed
	//     correction data over UART2 to/from a paired base/rover.
	//   * UART1 protocol setup for RoverWithMovingBaseUART1 / MovingBaseUART1
	//     (single-cable heading configurations).
	// Disabled here: receiver is assumed to be fully provisioned externally
	// (u-center / pre-flashed config). PX4 just consumes whatever NAV-* /
	// MON-* / RXM-RTCM stream the receiver already emits.
	(void)config;
	(void)uart2_baudrate;
	return 0;
#if 0
	// There is no RTCM or USB interface on M10
	if (_board != Board::u_blox10) {

		int cfg_valset_msg_size = initCfgValset();

		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X, _output_mode == OutputMode::RTCM ? 0 : 1,
				   cfg_valset_msg_size);

		if (_output_mode != OutputMode::GPS) {
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 1, cfg_valset_msg_size);
		}

		// USB
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_RTCM3X, _output_mode == OutputMode::RTCM ? 0 : 1,
				   cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBINPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_UBX, 1, cfg_valset_msg_size);

		if (_output_mode != OutputMode::GPS) {
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_RTCM3X, 1, cfg_valset_msg_size);
		}

		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_USBOUTPROT_NMEA, 0, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}
	}

	/* set configuration parameters */
	int cfg_valset_msg_size = initCfgValset();
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_FIXMODE, 3 /* Auto 2d/3d */, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_UTCSTANDARD, 3 /* USNO (U.S. Naval Observatory derived from GPS) */,
			   cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVSPG_DYNMODEL, _dyn_model, cfg_valset_msg_size);

	// disable odometer & filtering
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_USE_ODO, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_USE_COG, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_OUTLPVEL, 0, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_ODO_OUTLPCOG, 0, cfg_valset_msg_size);

	// measurement rate
	// F9P L1L2 in firmware <1.50 the max update rate with 4 constellations is 9Hz without RTK and 7Hz with RTK
	// F9P L1L2 in firmware >=1.50 the max update rate with 4 constellations is 7Hz without RTK and 5Hz with RTK
	// F9P L1L5 the max update rate with 4 constellations is 8Hz without RTK and 7Hz with RTK
	// Receivers such as M9N can go higher than 10Hz, but the number of used satellites will be restricted to 16. (Not mentioned in datasheet)
	int rate_meas = 100; // 10Hz

	switch (_board) {
	case Board::u_blox9_F9P_L1L2:
		rate_meas = 40; // 20Hz - Out of spec but working.
		break;

	case Board::u_blox9_F9P_L1L5:
		rate_meas = 40; // 20Hz - Out of spec but working.
		break;

	default:
		break;
	}

	cfgValset<uint16_t>(UBX_CFG_KEY_RATE_MEAS, rate_meas, cfg_valset_msg_size);
	cfgValset<uint16_t>(UBX_CFG_KEY_RATE_NAV, 1, cfg_valset_msg_size);
	cfgValset<uint8_t>(UBX_CFG_KEY_RATE_TIMEREF, 0, cfg_valset_msg_size);

	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	// RTK (optional, as only RTK devices like F9P support it)
	cfg_valset_msg_size = initCfgValset();
	cfgValset<uint8_t>(UBX_CFG_KEY_NAVHPG_DGNSSMODE, 3 /* RTK Fixed */, cfg_valset_msg_size);

	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

	cfg_valset_msg_size = initCfgValset();

	// enable jamming monitor
	cfgValset<uint8_t>(UBX_CFG_KEY_ITFM_ENABLE, 1, cfg_valset_msg_size);

	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

	// configure active GNSS systems (leave signal bands as is)
	// Note: For M10 configuration if changing from default. As per the
	//       MAX-M10S integration guide UBX-20053088 - R03, see section
	//       2.1.1.3 GNSS signal configuration for details on some restrictions.
	//       Implementing these restrictions are a TODO item for M10.
	if (static_cast<int32_t>(config.gnss_systems) != 0) {
		cfg_valset_msg_size = initCfgValset();

		// GPS and QZSS should always be enabled and disabled together, according to uBlox
		if (config.gnss_systems & GNSSSystemsMask::ENABLE_GPS) {
			UBX_DEBUG("GNSS Systems: Use GPS + QZSS");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_ENA, 1, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_QZSS_ENA, 1, cfg_valset_msg_size);

			if (_board == Board::u_blox9_F9P_L1L2) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_L2C_ENA, 1, cfg_valset_msg_size);

			} else if (_board == Board::u_blox9_F9P_L1L5) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_L5_ENA, 1, cfg_valset_msg_size);
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_L5_HEALTH_OVERRIDE, 1, cfg_valset_msg_size);
			}

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_ENA, 0, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_QZSS_ENA, 0, cfg_valset_msg_size);

			if (_board == Board::u_blox9_F9P_L1L2) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_L2C_ENA, 0, cfg_valset_msg_size);

			} else if (_board == Board::u_blox9_F9P_L1L5) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GPS_L5_ENA, 0, cfg_valset_msg_size);
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_L5_HEALTH_OVERRIDE, 0, cfg_valset_msg_size);
			}
		}

		if (config.gnss_systems & GNSSSystemsMask::ENABLE_GALILEO) {
			UBX_DEBUG("GNSS Systems: Use Galileo");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_ENA, 1, cfg_valset_msg_size);

			if (_board == Board::u_blox9_F9P_L1L2) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_E5B_ENA, 1, cfg_valset_msg_size);

			} else if (_board == Board::u_blox9_F9P_L1L5) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_E5A_ENA, 1, cfg_valset_msg_size);
			}

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_ENA, 0, cfg_valset_msg_size);

			if (_board == Board::u_blox9_F9P_L1L2) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_E5B_ENA, 0, cfg_valset_msg_size);

			} else if (_board == Board::u_blox9_F9P_L1L5) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GAL_E5A_ENA, 0, cfg_valset_msg_size);
			}
		}

		if (config.gnss_systems & GNSSSystemsMask::ENABLE_BEIDOU) {
			UBX_DEBUG("GNSS Systems: Use BeiDou");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_ENA, 1, cfg_valset_msg_size);

			if (_board == Board::u_blox9_F9P_L1L2) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_B2_ENA, 1, cfg_valset_msg_size);

			} else if (_board == Board::u_blox9_F9P_L1L5) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_B2A_ENA, 1, cfg_valset_msg_size);
			}

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_ENA, 0, cfg_valset_msg_size);

			if (_board == Board::u_blox9_F9P_L1L2) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_B2_ENA, 0, cfg_valset_msg_size);

			} else if (_board == Board::u_blox9_F9P_L1L5) {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_BDS_B2A_ENA, 0, cfg_valset_msg_size);
			}
		}

		if (config.gnss_systems & GNSSSystemsMask::ENABLE_GLONASS) {
			UBX_DEBUG("GNSS Systems: Use GLONASS");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GLO_ENA, 1, cfg_valset_msg_size);

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_GLO_ENA, 0, cfg_valset_msg_size);
		}

		if (_board == Board::u_blox9_F9P_L1L5) {
			if (config.gnss_systems & GNSSSystemsMask::ENABLE_NAVIC) {
				UBX_DEBUG("GNSS Systems: Use NavIC");
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_NAVIC_ENA, 1, cfg_valset_msg_size);
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_NAVIC_L5_ENA, 1, cfg_valset_msg_size);

			} else {
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_NAVIC_ENA, 0, cfg_valset_msg_size);
				cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_NAVIC_L5_ENA, 0, cfg_valset_msg_size);
			}
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			UBX_DEBUG("UBX GNSS config send failed");
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		// send SBAS config separately, because it seems to be buggy (with u-center, too)
		cfg_valset_msg_size = initCfgValset();

		if (config.gnss_systems & GNSSSystemsMask::ENABLE_SBAS) {
			UBX_DEBUG("GNSS Systems: Use SBAS");
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_ENA, 1, cfg_valset_msg_size);
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_L1CA_ENA, 1, cfg_valset_msg_size);

		} else {
			cfgValset<uint8_t>(UBX_CFG_KEY_SIGNAL_SBAS_ENA, 0, cfg_valset_msg_size);
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true);
	}

	// Configure message rates
	// Send a new CFG-VALSET message to make sure it does not get too large
	cfg_valset_msg_size = initCfgValset();
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_PVT_I2C, 1, cfg_valset_msg_size);

	// There is no RTCM on M10 and M9* (except F9P)
	if (_board != Board::u_blox10 && _board != Board::u_blox9) {
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_HPPOSLLH_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_RELPOSNED_I2C,
			      _mode == UBXMode::RoverWithMovingBase || _mode == UBXMode::RoverWithMovingBaseUART1 ? 1 : 0,
			      cfg_valset_msg_size);
	}

	_use_nav_pvt = true;
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_DOP_I2C, 1, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SAT_I2C, (_satellite_info != nullptr) ? 10 : 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_STATUS_I2C, 1, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_MON_RF_I2C, 1, cfg_valset_msg_size);

	if ((_board == Board::u_blox9) || (_board == Board::u_blox9_F9P_L1L2) || (_board == Board::u_blox9_F9P_L1L5)) {
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_RXM_RTCM_I2C, 1, cfg_valset_msg_size);
	}

	if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	if (_interface == Interface::UART || _interface == Interface::SPI) {

		// Enable/Disable GPS protocols at I2C interface
		cfg_valset_msg_size = initCfgValset();

		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_UBX,
				   config.interface_protocols & InterfaceProtocolsMask::I2C_IN_PROT_UBX, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_NMEA,
				   config.interface_protocols & InterfaceProtocolsMask::I2C_IN_PROT_NMEA, cfg_valset_msg_size);

		// There is no RTCM on M10
		if (_board != Board::u_blox10) {
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2CINPROT_RTCM3X,
					   config.interface_protocols & InterfaceProtocolsMask::I2C_IN_PROT_RTCM3X, cfg_valset_msg_size);
		}

		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_UBX,
				   config.interface_protocols & InterfaceProtocolsMask::I2C_OUT_PROT_UBX, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_NMEA,
				   config.interface_protocols & InterfaceProtocolsMask::I2C_OUT_PROT_NMEA, cfg_valset_msg_size);

		if ((_board == Board::u_blox9_F9P_L1L2) || (_board == Board::u_blox9_F9P_L1L5)) {
			cfgValset<uint8_t>(UBX_CFG_KEY_CFG_I2COUTPROT_RTCM3X,
					   config.interface_protocols & InterfaceProtocolsMask::I2C_OUT_PROT_RTCM3X, cfg_valset_msg_size);
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}
	}

	if (_mode == UBXMode::RoverWithStaticBaseUart2 || _mode == UBXMode::RoverWithMovingBase) {
		UBX_DEBUG("Configuring UART2 for rover");
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 0, cfg_valset_msg_size);
		// enable RTCM input on uart2 + set baudrate
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_STOPBITS, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_DATABITS, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_PARITY, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X, 0, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART2_BAUDRATE, uart2_baudrate, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

	} else if (_mode == UBXMode::MovingBase) {
		UBX_DEBUG("Configuring UART2 for moving base");
		// enable RTCM output on uart2 + set baudrate
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_STOPBITS, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_DATABITS, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2_PARITY, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2INPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_UBX, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART2OUTPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_CFG_UART2_BAUDRATE, uart2_baudrate, cfg_valset_msg_size);

		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1074_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1084_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1094_UART2, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1124_UART2, 1, cfg_valset_msg_size);

		if (_board == Board::u_blox9_F9P_L1L2) {
			// F9P-15B doesn't support 4072
			cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_0_UART2, 1, cfg_valset_msg_size);
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

	} else if (_mode == UBXMode::RoverWithMovingBaseUART1) {
		UBX_DEBUG("Configuring UART1 for rover");
		// heading output period 1 second
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 0, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

	} else if (_mode == UBXMode::MovingBaseUART1) {
		UBX_DEBUG("Configuring UART1 for moving base");
		// enable RTCM output on uart1
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1INPROT_NMEA, 0, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_UBX, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_CFG_UART1OUTPROT_RTCM3X, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_UART1, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1074_UART1, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1084_UART1, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1094_UART1, 1, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1124_UART1, 1, cfg_valset_msg_size);

		if (_board == Board::u_blox9_F9P_L1L2) {
			// F9P-15B doesn't support 4072
			cfgValset<uint8_t>(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE4072_0_UART1, 1, cfg_valset_msg_size);
		}

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

	}

	return 0;
#endif // configureDevice body disabled
}

int GPSDriverUBX::initCfgValset()
{
	memset(&_buf.payload_tx_cfg_valset, 0, sizeof(_buf.payload_tx_cfg_valset));
	_buf.payload_tx_cfg_valset.layers = UBX_CFG_LAYER_RAM;
	return sizeof(_buf.payload_tx_cfg_valset) - sizeof(_buf.payload_tx_cfg_valset.cfgData);
}

uint8_t GPSDriverUBX::ubxCfgKeySize(uint32_t key)
{
	// storage-size id in bits 28-30 of the key ID (u-blox Gen9+ config key format)
	switch (UBX_CFG_KEY_SIZE_ID(key)) {
	case 0x01: return 1;   // L  (1 bit, stored/transferred as 1 byte)
	case 0x02: return 1;   // U1 / I1 / E1 / X1
	case 0x03: return 2;   // U2 / I2 / E2 / X2
	case 0x04: return 4;   // U4 / I4 / E4 / X4 / R4
	case 0x05: return 8;   // U8 / I8 / X8 / R8
	default:   return 0;   // unknown -> caller must treat as fatal for this pair
	}
}

template<typename Handler>
int GPSDriverUBX::walkCfgTlv(const uint8_t *body, uint16_t len, Handler handler)
{
	// body points at the first key byte (the 4-byte version/layer/position header is
	// already skipped). Layout: repeated [4-byte LE keyID][value bytes]. Identical
	// encoding for a CFG-VALGET response and a CFG-VALSET request body.
	uint16_t off = 0;
	int pairs = 0;

	while (off < len) {
		if (off + 4 > len) {
			UBX_WARN("VALGET/TLV: truncated key at offset %u", off);
			return -1;
		}

		uint32_t key;
		memcpy(&key, body + off, sizeof(key));   // little-endian on target
		const uint8_t vlen = ubxCfgKeySize(key);

		if (vlen == 0) {
			UBX_WARN("VALGET/TLV: unknown size for key 0x%08x at offset %u", (unsigned)key, off);
			return -1;
		}

		if (off + 4 + vlen > len) {
			UBX_WARN("VALGET/TLV: truncated value for key 0x%08x at offset %u", (unsigned)key, off);
			return -1;
		}

		handler(key, body + off + 4, vlen);
		off += 4 + vlen;
		++pairs;
	}

	return pairs;
}

template<typename T>
bool GPSDriverUBX::cfgValset(uint32_t key_id, T value, int &msg_size)
{
	if (msg_size + sizeof(key_id) + sizeof(value) > sizeof(_buf)) {
		// If this happens use several CFG-VALSET messages instead of one
		UBX_WARN("buf for CFG_VALSET too small");
		return false;
	}

	uint8_t *buffer = (uint8_t *)&_buf.payload_tx_cfg_valset;
	memcpy(buffer + msg_size, &key_id, sizeof(key_id));
	msg_size += sizeof(key_id);
	memcpy(buffer + msg_size, &value, sizeof(value));
	msg_size += sizeof(value);
	return true;
}

bool GPSDriverUBX::cfgValsetPort(uint32_t key_id, uint8_t value, int &msg_size)
{
	if (_interface == Interface::SPI) {
		if (!cfgValset<uint8_t>(key_id + 4, value, msg_size)) {
			return false;
		}

	} else {
		// enable on UART1 & USB (TODO: should we enable UART2 too? -> better would be to detect the port)
		if (!cfgValset<uint8_t>(key_id + 1, value, msg_size)) {
			return false;
		}

		// M10 has no USB
		if (_board != Board::u_blox10) {
			if (!cfgValset<uint8_t>(key_id + 3, value, msg_size)) {
				return false;
			}
		}
	}

	return true;
}

/* Decode two ASCII hex nibbles at s into a byte. Returns -1 if either char is not hex. */
static int ubx_hex_byte(const char *s)
{
	auto nib = [](char c) -> int {
		if (c >= '0' && c <= '9') { return c - '0'; }
		if (c >= 'a' && c <= 'f') { return c - 'a' + 10; }
		if (c >= 'A' && c <= 'F') { return c - 'A' + 10; }
		return -1;
	};
	const int hi = nib(s[0]);
	const int lo = nib(s[1]);
	if (hi < 0 || lo < 0) { return -1; }
	return (hi << 4) | lo;
}

int GPSDriverUBX::loadConfigFromFile(const char *path)
{
	const int fd = ::open(path, O_RDONLY);

	if (fd < 0) {
		// No config file present is the normal case for an externally-provisioned
		// receiver; warn and leave the device untouched (never fails bring-up).
		UBX_WARN("UBX cfg: %s not found/unreadable (errno %d), skipping provisioning", path, errno);
		return -1;
	}

	// Own buffers — sendMessage() streams the caller buffer directly, so the tiny _buf
	// union is not involved and frames may be larger than sizeof(_buf). The two large
	// buffers are heap-allocated to keep the GPS thread stack frame small.
	// All heap-allocated: u-center packs one CFG-VALGET per line with many keys, so a line's
	// ASCII hex and its decoded payload are large (KB-scale) and must not sit on the GPS stack.
	char    *line   = (char *)malloc(UBX_CFGFILE_LINE_MAX);             // one file line (ASCII)
	uint8_t *hexbuf = (uint8_t *)malloc(UBX_CFGFILE_HEX_MAX);           // decoded bytes of one line
	uint8_t *valset = (uint8_t *)malloc(4 + UBX_VALSET_MAX_KEYS * 12);  // header + up to 64 [key+value]

	if (!line || !hexbuf || !valset) {
		UBX_ERR("UBX cfg: out of memory");
		free(line);
		free(hexbuf);
		free(valset);
		::close(fd);
		return -1;
	}

	const size_t valset_cap = 4 + UBX_VALSET_MAX_KEYS * 12;
	unsigned lineno = 0;
	unsigned frames = 0;
	unsigned nak = 0;
	unsigned skipped = 0;

	// Manual line reader over the fd (no stdio: bounded memory, no NuttX buffering surprises).
	char   rd[256];
	int    rd_len = 0;
	int    rd_pos = 0;
	int    li = 0;
	bool   line_overflow = false;
	bool   eof = false;

	while (!eof) {
		if (rd_pos >= rd_len) {
			rd_len = ::read(fd, rd, sizeof(rd));
			rd_pos = 0;

			if (rd_len <= 0) {
				eof = true;

				if (li == 0) { break; }   // no partial line pending
			}
		}

		char c = 0;

		if (!eof) {
			c = rd[rd_pos++];
		}

		if (!eof && c != '\n') {
			if (li < (int)UBX_CFGFILE_LINE_MAX - 1) {
				line[li++] = c;

			} else {
				line_overflow = true;   // keep consuming until newline, then report
			}

			continue;
		}

		// End of a line (or EOF with a pending partial line).
		line[li] = '\0';
		++lineno;
		const int this_li = li;
		li = 0;

		if (line_overflow) {
			UBX_WARN("UBX cfg line %u: too long (>%u), skipping", lineno, (unsigned)UBX_CFGFILE_LINE_MAX);
			UBXCFG_LOG("provision: line %u too long (>%u chars), skipping", lineno, (unsigned)UBX_CFGFILE_LINE_MAX);
			line_overflow = false;
			continue;
		}

		// Trim leading whitespace.
		char *p = line;

		while (*p == ' ' || *p == '\t' || *p == '\r') { ++p; }

		// Skip blank and comment lines.
		if (*p == '\0' || *p == '#' || *p == ';' || *p == '/') {
			continue;
		}

		(void)this_li;

		// Skip the "<NAME> - " prefix and reach the hex field. The message NAME itself
		// contains '-' (e.g. "CFG-VALGET", "CFG-RATE-MEAS"), so we must split on the
		// " - " separator (space-dash-space) between name and hex, NOT the first '-'.
		char *sep = strstr(p, " - ");

		if (sep) {
			p = sep + 3;   // step past " - "

		} else {
			// No " - " separator: fall back to skipping a leading non-hex name token
			// (advance past the first whitespace-delimited word if it isn't hex).
			char *q = p;

			while (*q && *q != ' ' && *q != '\t') { ++q; }

			if (*q) { p = q + 1; }
		}

		// Tokenize hex bytes.
		int n = 0;
		bool bad = false;

		while (*p && n < (int)UBX_CFGFILE_HEX_MAX) {
			while (*p == ' ' || *p == '\t' || *p == '\r') { ++p; }

			if (*p == '\0') { break; }

			// need two hex chars
			if (p[1] == '\0') { bad = true; break; }

			const int b = ubx_hex_byte(p);

			if (b < 0) { bad = true; break; }

			hexbuf[n++] = (uint8_t)b;
			p += 2;
		}

		if (bad) {
			UBX_WARN("UBX cfg line %u: bad hex, skipping line", lineno);
			++skipped;
			continue;
		}

		// u-center writes the raw UBX frame bytes (minus sync + checksum):
		//   class(1) id(1) length(2, LE) | version(1) layer(1) position(2) | [key(4)+value]...
		// So the CFG-VALGET body header is at offset 4, and the key/value TLVs at offset 8.
		static const int kHdrBytes = 2 /*class+id*/ + 2 /*length*/ + 4 /*ver+layer+pos*/;

		if (n < kHdrBytes) {
			UBX_WARN("UBX cfg line %u: malformed (%d bytes), skipping", lineno, n);
			++skipped;
			continue;
		}

		const uint8_t cls = hexbuf[0];
		const uint8_t id  = hexbuf[1];

		// Keep only CFG-VALGET lines (0x06 0x8B) — u-center exports config as VALGET dumps.
		if (!(cls == UBX_CLASS_CFG && id == UBX_ID_CFG_VALGET)) {
			UBX_DEBUG("UBX cfg line %u: skipping non-VALGET (%02X %02X)", lineno, cls, id);
			++skipped;
			continue;
		}

		// Walk the key/value TLVs (after class+id+length+4-byte VALGET header) and re-emit
		// them as CFG-VALSET frames (RAM layer), splitting at 64 keys per frame.
		const uint8_t *tlv = &hexbuf[kHdrBytes];
		const uint16_t tlv_len = (uint16_t)(n - kHdrBytes);

		int  msg_size = 0;
		int  keys_in_frame = 0;

		auto flush_frame = [&]() -> bool {
			if (keys_in_frame == 0) { return true; }

			const bool ok = sendMessage(UBX_MSG_CFG_VALSET, valset, (uint16_t)msg_size);

			if (!ok) {
				UBX_ERR("UBX cfg line %u: UART write failed", lineno);
				return false;
			}

			// Longer ACK wait than the default RAM timeout: these VALSET frames also write the
			// F9P Flash layer (erase/program), which the receiver acknowledges more slowly.
			if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CFGFILE_ACK_TIMEOUT, true) < 0) {
				UBX_WARN("UBX cfg line %u: device NAK/timeout for VALSET", lineno);
				++nak;

			} else {
				++frames;
			}

			msg_size = 0;
			keys_in_frame = 0;
			return true;
		};

		auto start_frame = [&]() {
			// 4-byte VALSET header: version=0, layers, reserved[2]=0.
			// Apply to RAM (takes effect immediately for this session) AND Flash (persists
			// in the F9P's own NVM across power cycles, like u-center's "save to flash").
			valset[0] = 0;
			valset[1] = UBX_CFG_LAYER_RAM | UBX_CFG_LAYER_FLASH;
			valset[2] = 0;
			valset[3] = 0;
			msg_size = 4;
			keys_in_frame = 0;
		};

		start_frame();

		bool io_ok = true;

		const int pairs = walkCfgTlv(tlv, tlv_len,
					     [&](uint32_t key, const uint8_t *val, uint8_t vlen) {
			if (!io_ok) { return; }

			// Would this pair overflow the frame buffer or the 64-key limit? flush first.
			if (keys_in_frame >= UBX_VALSET_MAX_KEYS ||
			    (size_t)msg_size + 4 + vlen > valset_cap) {
				if (!flush_frame()) { io_ok = false; return; }
				start_frame();
			}

			memcpy(&valset[msg_size], &key, 4);
			msg_size += 4;
			memcpy(&valset[msg_size], val, vlen);
			msg_size += vlen;
			++keys_in_frame;
		});

		if (!io_ok) {
			// hard UART error — stop trying, report what we got
			break;
		}

		if (pairs < 0) {
			UBX_WARN("UBX cfg line %u: truncated VALGET body, skipping remainder", lineno);
			++skipped;
			continue;
		}

		if (!flush_frame()) {
			break;
		}
	}

	::close(fd);
	free(line);
	free(hexbuf);
	free(valset);

	if (frames == 0 && nak == 0 && skipped == 0) {
		UBX_WARN("UBX cfg: no CFG-VALGET lines found in %s", path);
	}

	UBX_INFO("UBX cfg: %s frames=%u nak=%u skipped=%u", path, frames, nak, skipped);
	return (int)frames;
}

// Config groups present on the ZED-F9P (group byte = bits 16-23 of a key ID), derived from the
// UBX_CFG_KEY_* constants. A wildcard CFG-VALGET over each group returns that group's RAM values.
static const uint8_t kUbxCfgGroups[] = {
	0x03, 0x11, 0x14, 0x21, 0x22, 0x31, 0x32, 0x41, 0x51, 0x52, 0x53, 0x64,
	0x65, 0x71, 0x72, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0x91,
};

int GPSDriverUBX::pollAllConfig(int fd, uint32_t &crc)
{
	// Poll every known config group with a wildcard CFG-VALGET on the RAM layer, paginating
	// until a page returns fewer than 64 pairs. The VALGET response handler (payloadRxDone)
	// walks each page's TLV body, filters non-zero values, and appends KEY= lines to fd.
	// Uses the same UART as NAV output, so this must run only when gated (RBF inserted).
	static const int kMaxPagesPerGroup = 32;

	_valget_dump_fd = fd;
	_valget_dump_crc = 0;
	_valget_dump_count = 0;

	for (unsigned g = 0; g < sizeof(kUbxCfgGroups) / sizeof(kUbxCfgGroups[0]); ++g) {
		const uint8_t group = kUbxCfgGroups[g];

		for (int page = 0; page < kMaxPagesPerGroup; ++page) {
			ubx_payload_tx_cfg_valget_t req{};
			req.version  = 0;
			req.layer    = UBX_CFG_VALGET_LAYER_RAM;
			req.position = (uint16_t)(page * 64);
			// wildcard: all items in this group (item bits 0xFFFF, size nibble 0)
			req.keys     = ((uint32_t)group << 16) | 0x0000FFFFu;

			const uint32_t count_before = _valget_dump_count;

			_valget_len = 0;
			_valget_capturing = true;

			const bool ok = sendMessage(UBX_MSG_CFG_VALGET, (const uint8_t *)&req, sizeof(req));

			if (!ok) {
				_valget_capturing = false;
				UBX_ERR("VALGET group %02X: UART write failed", group);
				_valget_dump_fd = -1;
				crc = _valget_dump_crc;
				return -1;
			}

			// The response (CFG-VALGET, 0x06 0x8B) is decoded in payloadRxDone which clears
			// _valget_capturing; a NAK/timeout means the group/page has no more data.
			const bool acked = (waitForAck(UBX_MSG_CFG_VALGET, UBX_CONFIG_TIMEOUT, false) == 0);
			_valget_capturing = false;

			const uint32_t got = _valget_dump_count - count_before;

			if (!acked && got == 0) {
				break;   // empty group or done
			}

			if (got < 64) {
				break;   // last page of this group
			}

			if (page == kMaxPagesPerGroup - 1) {
				UBX_WARN("VALGET group %02X: pagination cap hit", group);
			}
		}
	}

	crc = _valget_dump_crc;
	const int total = (int)_valget_dump_count;
	_valget_dump_fd = -1;
	return total;
}

int GPSDriverUBX::dumpConfigToFile(const char *tmp_path, const char *final_path)
{
	// Ensure the target directory exists (ignore EEXIST).
	if (::mkdir(UBX_CFG_DIR, 0777) < 0 && errno != EEXIST) {
		UBX_ERR("UBX dump: mkdir %s failed (errno %d)", UBX_CFG_DIR, errno);
		return -1;
	}

	const int fd = ::open(tmp_path, O_WRONLY | O_CREAT | O_TRUNC, 0644);

	if (fd < 0) {
		UBX_ERR("UBX dump: open %s failed (errno %d)", tmp_path, errno);
		return -1;
	}

	char hdr[128];
	int hn = snprintf(hdr, sizeof(hdr),
			  "# UBLOX F9P RAM CONFIG DUMP\n# time_us=%llu layer=RAM\n",
			  (unsigned long long)gps_absolute_time());
	::write(fd, hdr, hn);

	uint32_t crc = 0;
	const int count = pollAllConfig(fd, crc);

	if (count < 0) {
		::close(fd);
		::unlink(tmp_path);
		return -1;
	}

	char trailer[64];
	int tn = snprintf(trailer, sizeof(trailer), "# COUNT=%d\n# CRC32=0x%08x\n", count, (unsigned)crc);
	::write(fd, trailer, tn);

	::fsync(fd);
	::close(fd);

	if (::rename(tmp_path, final_path) < 0) {
		UBX_ERR("UBX dump: rename %s -> %s failed (errno %d)", tmp_path, final_path, errno);
		::unlink(tmp_path);
		return -1;
	}

	return count;
}

int GPSDriverUBX::restartSurveyInPreV27()
{
	// === DISABLED PERIPHERAL CONFIG WRITES (pre-v27 base-station setup) ===
	// Originally drives the receiver into Time Mode 3 (base station) on legacy
	// modules:
	//   * CFG-MSG rate=0 for RTCM3 types 1005/1077/1087/1230/1097/1127 - turn
	//     off any pre-existing RTCM3 output streams before reconfiguring.
	//   * CFG-TMODE3 flags=0 - disable any active Time Mode (stop survey-in or
	//     fixed-position mode cleanly).
	//   * CFG-TMODE3 flags=1 + svinMinDur + svinAccLimit - start a survey-in:
	//     receiver averages its own position until duration/accuracy thresholds
	//     are met. Used to auto-determine a base-station coordinate.
	//   * CFG-MSG NAV-SVIN at 5 Hz - enable survey-in status output so the host
	//     can monitor progress.
	//   * CFG-TMODE3 flags=2|lat/lon mode + ecef[XYZ]Lat/Lon/Alt + HP parts +
	//     fixedPosAcc - configure fixed-base mode at a known surveyed position.
	//     Used when the base coordinate is already known precisely.
	// Useful for setting up an RTCM correction source. Disabled - we do not
	// operate this driver as a base station; survey-in / fixed-base must be
	// pre-provisioned in the receiver if needed.
	return 0;
#if 0
	//disable RTCM output
	configureMessageRate(UBX_MSG_RTCM3_1005, 0);
	configureMessageRate(UBX_MSG_RTCM3_1077, 0);
	configureMessageRate(UBX_MSG_RTCM3_1087, 0);
	configureMessageRate(UBX_MSG_RTCM3_1230, 0);
	configureMessageRate(UBX_MSG_RTCM3_1097, 0);
	configureMessageRate(UBX_MSG_RTCM3_1127, 0);

	//stop it first
	//FIXME: stopping the survey-in process does not seem to work
	memset(&_buf.payload_tx_cfg_tmode3, 0, sizeof(_buf.payload_tx_cfg_tmode3));
	_buf.payload_tx_cfg_tmode3.flags        = 0; /* disable time mode */

	if (!sendMessage(UBX_MSG_CFG_TMODE3, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_tmode3))) {
		UBX_WARN("TMODE3 failed. Device w/o base station support?");
		return -1;
	}

	if (waitForAck(UBX_MSG_CFG_TMODE3, UBX_CONFIG_TIMEOUT, true) < 0) {
		return -1;
	}

	if (_base_settings.type == BaseSettingsType::survey_in) {
		UBX_DEBUG("Starting Survey-in");

		memset(&_buf.payload_tx_cfg_tmode3, 0, sizeof(_buf.payload_tx_cfg_tmode3));
		_buf.payload_tx_cfg_tmode3.flags        = 1; /* start survey-in */
		_buf.payload_tx_cfg_tmode3.svinMinDur   = _base_settings.settings.survey_in.min_dur;
		_buf.payload_tx_cfg_tmode3.svinAccLimit = _base_settings.settings.survey_in.acc_limit;

		if (!sendMessage(UBX_MSG_CFG_TMODE3, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_tmode3))) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_TMODE3, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		/* enable status output of survey-in */
		if (!configureMessageRateAndAck(UBX_MSG_NAV_SVIN, 5, true)) {
			return -1;
		}

	} else {
		UBX_DEBUG("Setting fixed base position");

		const FixedPositionSettings &settings = _base_settings.settings.fixed_position;

		memset(&_buf.payload_tx_cfg_tmode3, 0, sizeof(_buf.payload_tx_cfg_tmode3));
		_buf.payload_tx_cfg_tmode3.flags = 2 /* fixed mode */ | (1 << 8) /* lat/lon mode */;
		int64_t lat64 = (int64_t)(settings.latitude * 1e9);
		_buf.payload_tx_cfg_tmode3.ecefXOrLat = (int32_t)(lat64 / 100);
		_buf.payload_tx_cfg_tmode3.ecefXOrLatHP = lat64 % 100; // range [-99, 99]
		int64_t lon64 = (int64_t)(settings.longitude * 1e9);
		_buf.payload_tx_cfg_tmode3.ecefYOrLon = (int32_t)(lon64 / 100);
		_buf.payload_tx_cfg_tmode3.ecefYOrLonHP = lon64 % 100;
		int64_t alt64 = (int64_t)((double)settings.altitude * 1e4);
		_buf.payload_tx_cfg_tmode3.ecefZOrAlt = (int32_t)(alt64 / 100); // cm
		_buf.payload_tx_cfg_tmode3.ecefZOrAltHP = alt64 % 100; // 0.1mm

		_buf.payload_tx_cfg_tmode3.fixedPosAcc = (uint32_t)(settings.position_accuracy * 10.f);

		if (!sendMessage(UBX_MSG_CFG_TMODE3, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_tmode3))) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_TMODE3, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		// directly enable RTCM3 output
		return activateRTCMOutput(true);
	}

	return 0;
#endif // restartSurveyInPreV27 body disabled
}

int GPSDriverUBX::restartSurveyIn()
{
	// === DISABLED PERIPHERAL CONFIG WRITES (v27+ base-station setup) ===
	// Same intent as restartSurveyInPreV27 but on modern receivers via
	// CFG-VALSET / config DB:
	//   * MSGOUT_RTCM_3X_TYPE{1005,1077,1087,1230,1097,1127}_* = 0 - silence
	//     any existing RTCM3 base-station message outputs before reconfig.
	//   * TMODE_MODE = 1 (Survey-in) + TMODE_SVIN_MIN_DUR/ACC_LIMIT - start a
	//     self-survey: the receiver averages its position until threshold met.
	//   * MSGOUT_UBX_NAV_SVIN = 5 - stream survey-in status to host.
	//   * TMODE_MODE = 2 (Fixed) + TMODE_POS_TYPE=1 (lat/lon/height) +
	//     LAT/LON/HEIGHT (with HP fractional parts) + FIXED_POS_ACC -
	//     program a precisely surveyed base coordinate.
	// Same purpose: configure the receiver as an RTK base station emitting
	// RTCM corrections. Disabled - this driver runs as rover; do not push
	// any base-station config to the peripheral.
	return 0;
#if 0
	if (_output_mode != OutputMode::RTCM) {
		return -1;
	}

	if (!_proto_ver_27_or_higher) {
		return restartSurveyInPreV27();
	}

	//disable RTCM output
	int cfg_valset_msg_size = initCfgValset();
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C, 0, cfg_valset_msg_size);
	cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C, 0, cfg_valset_msg_size);
	sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
	waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false);

	if (_base_settings.type == BaseSettingsType::survey_in) {
		UBX_DEBUG("Starting Survey-in");

		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_MODE, 1 /* Survey-in */, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_MIN_DUR, _base_settings.settings.survey_in.min_dur, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_SVIN_ACC_LIMIT, _base_settings.settings.survey_in.acc_limit, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C, 5, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

	} else {
		UBX_DEBUG("Setting fixed base position");

		const FixedPositionSettings &settings = _base_settings.settings.fixed_position;
		cfg_valset_msg_size = initCfgValset();
		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_MODE, 2 /* Fixed Mode */, cfg_valset_msg_size);
		cfgValset<uint8_t>(UBX_CFG_KEY_TMODE_POS_TYPE, 1 /* Lat/Lon/Height */, cfg_valset_msg_size);
		int64_t lat64 = (int64_t)(settings.latitude * 1e9);
		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_LAT, (int32_t)(lat64 / 100), cfg_valset_msg_size);
		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_LAT_HP, lat64 % 100 /* range [-99, 99] */, cfg_valset_msg_size);
		int64_t lon64 = (int64_t)(settings.longitude * 1e9);
		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_LON, (int32_t)(lon64 / 100), cfg_valset_msg_size);
		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_LON_HP, lon64 % 100 /* range [-99, 99] */, cfg_valset_msg_size);
		int64_t alt64 = (int64_t)((double)settings.altitude * 1e4);
		cfgValset<int32_t>(UBX_CFG_KEY_TMODE_HEIGHT, (int32_t)(alt64 / 100) /* cm */, cfg_valset_msg_size);
		cfgValset<int8_t>(UBX_CFG_KEY_TMODE_HEIGHT_HP, alt64 % 100 /* 0.1mm */, cfg_valset_msg_size);
		cfgValset<uint32_t>(UBX_CFG_KEY_TMODE_FIXED_POS_ACC, (uint32_t)(settings.position_accuracy * 10.f),
				    cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, true) < 0) {
			return -1;
		}

		// directly enable RTCM3 output
		return activateRTCMOutput(true);

	}

	return 0;
#endif // restartSurveyIn body disabled
}

int	// -1 = NAK, error or timeout, 0 = ACK
GPSDriverUBX::waitForAck(const uint16_t msg, const unsigned timeout, const bool report)
{
	int ret = -1;

	_ack_state = UBX_ACK_WAITING;
	_ack_waiting_msg = msg;	// memorize sent msg class&ID for ACK check

	gps_abstime time_started = gps_absolute_time();

	while ((_ack_state == UBX_ACK_WAITING) && (gps_absolute_time() < time_started + timeout * 1000)) {
		receive(timeout);
	}

	if (_ack_state == UBX_ACK_GOT_ACK) {
		ret = 0;	// ACK received ok

	} else if (report) {
		if (_ack_state == UBX_ACK_GOT_NAK) {
			UBX_DEBUG("ubx msg 0x%04x NAK", SWAP16((unsigned)msg));

		} else {
			UBX_DEBUG("ubx msg 0x%04x ACK timeout", SWAP16((unsigned)msg));
		}
	}

	_ack_state = UBX_ACK_IDLE;
	return ret;
}

int	// -1 = error, 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverUBX::receive(unsigned timeout)
{
	uint8_t buf[GPS_READ_BUFFER_SIZE];

	/* timeout additional to poll */
	gps_abstime time_started = gps_absolute_time();

	int handled = 0;

	while (true) {
		bool ready_to_return = _configured ? (_got_posllh && _got_velned) : handled;

		/* return success if ready */
		if (ready_to_return) {
			_got_posllh = false;
			_got_velned = false;
			return handled;
		}

		/* Wait for only UBX_PACKET_TIMEOUT if something already received. */
		int ret = read(buf, sizeof(buf), (_got_posllh || _got_velned) ? UBX_PACKET_TIMEOUT : timeout);

		if (ret < 0) {
			/* something went wrong when polling or reading */
			UBX_WARN("ubx poll_or_read err");
			return -1;

		} else if (ret > 0) {
			//UBX_DEBUG("read %d bytes", ret);

			/* pass received bytes to the packet decoder */
			for (int i = 0; i < ret; i++) {
				handled |= parseChar(buf[i]);
				//UBX_DEBUG("parsed %d: 0x%x", i, buf[i]);
			}

			if (_interface == Interface::SPI) {
				if (buf[ret - 1] == 0xff) {
					if (ready_to_return) {
						_got_posllh = false;
						_got_velned = false;
						return handled;
					}
				}
			}
		}

		/* abort after timeout if no useful packets received */
		if (time_started + timeout * 1000 < gps_absolute_time()) {
			UBX_DEBUG("timed out, returning");
			return -1;
		}
	}
}

int	// 0 = decoding, 1 = message handled, 2 = sat info message handled
GPSDriverUBX::parseChar(const uint8_t b)
{
	int ret = 0;

	if (_rtcm_parsing) {
		if (_rtcm_parsing->addByte(b)) {
			gotRTCMMessage(_rtcm_parsing->message(), _rtcm_parsing->messageLength());
			decodeInit();
			_rtcm_parsing->reset();
			return ret;
		}
	}

	switch (_decode_state) {

	/* Expecting Sync1 */
	case UBX_DECODE_SYNC1:
		if (b == UBX_SYNC1) {	// Sync1 found --> expecting Sync2
			UBX_TRACE_PARSER("A");
			_decode_state = UBX_DECODE_SYNC2;
		}

		break;

	/* Expecting Sync2 */
	case UBX_DECODE_SYNC2:
		if (b == UBX_SYNC2) {	// Sync2 found --> expecting Class
			UBX_TRACE_PARSER("B");
			_decode_state = UBX_DECODE_CLASS;

		} else {		// Sync1 not followed by Sync2: reset parser
			decodeInit();
		}

		break;

	/* Expecting Class */
	case UBX_DECODE_CLASS:
		UBX_TRACE_PARSER("C");
		addByteToChecksum(b);  // checksum is calculated for everything except Sync and Checksum bytes
		_rx_msg = b;
		_decode_state = UBX_DECODE_ID;
		break;

	/* Expecting ID */
	case UBX_DECODE_ID:
		UBX_TRACE_PARSER("D");
		addByteToChecksum(b);
		_rx_msg |= b << 8;
		_decode_state = UBX_DECODE_LENGTH1;
		break;

	/* Expecting first length byte */
	case UBX_DECODE_LENGTH1:
		UBX_TRACE_PARSER("E");
		addByteToChecksum(b);
		_rx_payload_length = b;
		_decode_state = UBX_DECODE_LENGTH2;
		break;

	/* Expecting second length byte */
	case UBX_DECODE_LENGTH2:
		UBX_TRACE_PARSER("F");
		addByteToChecksum(b);
		_rx_payload_length |= b << 8;	// calculate payload size

		if (payloadRxInit() != 0) {	// start payload reception
			PX4_WARN("UBX_DECODE_LENGTH2: payload discarded");
			// payload will not be handled, discard message
			decodeInit();

		} else {
			_decode_state = (_rx_payload_length > 0) ? UBX_DECODE_PAYLOAD : UBX_DECODE_CHKSUM1;
		}

		break;

	/* Expecting payload */
	case UBX_DECODE_PAYLOAD:
		UBX_TRACE_PARSER(".");
		addByteToChecksum(b);

		switch (_rx_msg) {
		case UBX_MSG_NAV_SAT:
			ret = payloadRxAddNavSat(b);	// add a NAV-SAT payload byte
			break;

		case UBX_MSG_NAV_SVINFO:
			ret = payloadRxAddNavSvinfo(b);	// add a NAV-SVINFO payload byte
			break;

		case UBX_MSG_MON_VER:
			ret = payloadRxAddMonVer(b);	// add a MON-VER payload byte
			break;

		case UBX_MSG_CFG_VALGET:
			ret = payloadRxAddCfgValget(b);	// add a CFG-VALGET response byte (RAM dump)
			break;

		default:
			ret = payloadRxAdd(b);		// add a payload byte
			break;
		}

		if (ret < 0) {
			PX4_WARN("UBX_DECODE_PAYLOAD: payload discarded");
			// payload not handled, discard message
			decodeInit();

		} else if (ret > 0) {
			// payload complete, expecting checksum
			_decode_state = UBX_DECODE_CHKSUM1;

		} else {
			// expecting more payload, stay in state UBX_DECODE_PAYLOAD
		}

		ret = 0;
		break;

	/* Expecting first checksum byte */
	case UBX_DECODE_CHKSUM1:
		if (_rx_ck_a != b) {
			UBX_DEBUG("ubx checksum err");
			decodeInit();

		} else {
			_decode_state = UBX_DECODE_CHKSUM2;
		}

		break;

	/* Expecting second checksum byte */
	case UBX_DECODE_CHKSUM2:
		if (_rx_ck_b != b) {
			UBX_DEBUG("ubx checksum err");

		} else {
			ret = payloadRxDone();	// finish payload processing

			if (_rtcm_parsing) {
				_rtcm_parsing->reset();
			}
		}

		decodeInit();
		break;

	default:
		break;
	}

	return ret;
}

/**
 * Start payload rx
 */
int	// -1 = abort, 0 = continue
GPSDriverUBX::payloadRxInit()
{
	int ret = 0;

	_rx_state = UBX_RXMSG_HANDLE;	// handle by default

	switch (_rx_msg) {
	case UBX_MSG_NAV_PVT:
		if ((_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX7)		/* u-blox 7 msg format */
		    && (_rx_payload_length != UBX_PAYLOAD_RX_NAV_PVT_SIZE_UBX8)) {	/* u-blox 8+ msg format */
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (!_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if not using NAV-PVT
		}

		break;

	case UBX_MSG_INF_DEBUG:
	case UBX_MSG_INF_ERROR:
	case UBX_MSG_INF_NOTICE:
	case UBX_MSG_INF_WARNING:
		if (_rx_payload_length >= sizeof(ubx_buf_t)) {
			_rx_payload_length = sizeof(ubx_buf_t) - 1; //avoid buffer overflow
		}

		break;

	case UBX_MSG_NAV_POSLLH:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_posllh_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_SOL:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_sol_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_STATUS:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_status_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_DOP:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_dop_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_RELPOSNED:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_relposned_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_HPPOSLLH:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_hpposllh_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_TIMEUTC:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_timeutc_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_NAV_SAT:
	case UBX_MSG_NAV_SVINFO:
		if (_satellite_info == nullptr) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if sat info not requested

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else {
			memset(_satellite_info, 0, sizeof(*_satellite_info));        // initialize sat info
		}

		break;

	case UBX_MSG_NAV_SVIN:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_svin_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		}

		break;

	case UBX_MSG_NAV_VELNED:
		if (_rx_payload_length != sizeof(ubx_payload_rx_nav_velned_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured

		} else if (_use_nav_pvt) {
			_rx_state = UBX_RXMSG_DISABLE;        // disable if using NAV-PVT instead
		}

		break;

	case UBX_MSG_MON_VER:
		break;		// unconditionally handle this message

	case UBX_MSG_MON_HW:
		if ((_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx6_t))	/* u-blox 6 msg format */
		    && (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_ubx7_t))	/* u-blox 7+ msg format */
		    && (_rx_payload_length != sizeof(ubx_payload_rx_mon_hw_deprecated_t))) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured
		}

		break;

	case UBX_MSG_MON_RF:
		if (_rx_payload_length < sizeof(ubx_payload_rx_mon_rf_t) ||
		    (_rx_payload_length - 4) % sizeof(ubx_payload_rx_mon_rf_t::ubx_payload_rx_mon_rf_block_t) != 0) {

			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured
		}

		break;

	case UBX_MSG_RXM_RTCM:
		if (_rx_payload_length != sizeof(ubx_payload_rx_rxm_rtcm_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (!_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if not _configured
		}

		break;

	case UBX_MSG_ACK_ACK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_ack_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if _configured
		}

		break;

	case UBX_MSG_ACK_NAK:
		if (_rx_payload_length != sizeof(ubx_payload_rx_ack_nak_t)) {
			_rx_state = UBX_RXMSG_ERROR_LENGTH;

		} else if (_configured) {
			_rx_state = UBX_RXMSG_IGNORE;        // ignore if _configured
		}

		break;

	case UBX_MSG_CFG_VALGET:
		// RAM read-back response. Only accept while a pollAllConfig() request is in flight
		// and it fits our dedicated capture buffer (must NOT go into the small _buf union).
		if (_valget_capturing && _rx_payload_length <= UBX_VALGET_RX_MAX) {
			_rx_state = UBX_RXMSG_HANDLE;
			_valget_len = 0;

		} else {
			_rx_state = UBX_RXMSG_DISABLE;
		}

		break;

	default:
		_rx_state = UBX_RXMSG_DISABLE;	// disable all other messages
		break;
	}

	switch (_rx_state) {
	case UBX_RXMSG_HANDLE:	// handle message
	case UBX_RXMSG_IGNORE:	// ignore message but don't report error
		ret = 0;
		break;

	case UBX_RXMSG_DISABLE:	// disable unexpected messages
		UBX_DEBUG("ubx msg 0x%04x len %u unexpected", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);

		// === DISABLED PERIPHERAL CONFIG WRITE (auto-silence path) ===
		// Originally: when the driver decodes a UBX message it didn't ask for
		// (RXM-RAWX, RXM-SFRBX, NAV-TIMEGPS on v27+, or any unhandled msg on
		// pre-v27), throttle-then-send CFG-VALSET MSGOUT=0 (v27+) or CFG-MSG
		// rate=0 (pre-v27) to tell the receiver to stop emitting it.
		// Useful to clean up bandwidth on links shared with other tools or
		// when the receiver's previous config left extra streams enabled.
		// Disabled - never push silence commands; just discard the unwanted
		// message and continue.
		// if (_proto_ver_27_or_higher) {
		// 	uint32_t key_id = 0;
		//
		// 	switch (_rx_msg) { // we cannot infer the config Key ID from _rx_msg for protocol version 27+
		// 	case UBX_MSG_RXM_RAWX:
		// 		key_id = UBX_CFG_KEY_MSGOUT_UBX_RXM_RAWX_I2C;
		// 		break;
		//
		// 	case UBX_MSG_RXM_SFRBX:
		// 		key_id = UBX_CFG_KEY_MSGOUT_UBX_RXM_SFRBX_I2C;
		// 		break;
		//
		// 	case UBX_MSG_NAV_TIMEGPS:
		// 		key_id = UBX_CFG_KEY_MSGOUT_UBX_NAV_TIMEGPS_I2C;
		// 		break;
		// 	}
		//
		// 	if (key_id != 0) {
		// 		gps_abstime t = gps_absolute_time();
		//
		// 		if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL && _configured) {
		// 			/* don't attempt for every message to disable, some might not be disabled */
		// 			_disable_cmd_last = t;
		// 			UBX_DEBUG("ubx disabling msg 0x%04x (0x%04x)", SWAP16((unsigned)_rx_msg), (uint16_t)key_id);
		//
		// 			// this will overwrite _buf, which is fine, as we'll return -1 and abort further parsing
		// 			int cfg_valset_msg_size = initCfgValset();
		// 			cfgValsetPort(key_id, 0, cfg_valset_msg_size);
		// 			sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size);
		// 		}
		// 	}
		//
		// } else {
		// 	gps_abstime t = gps_absolute_time();
		//
		// 	if (t > _disable_cmd_last + DISABLE_MSG_INTERVAL) {
		// 		/* don't attempt for every message to disable, some might not be disabled */
		// 		_disable_cmd_last = t;
		// 		UBX_DEBUG("ubx disabling msg 0x%04x", SWAP16((unsigned)_rx_msg));
		//
		// 		configureMessageRate(_rx_msg, 0);
		// 	}
		// }

		ret = -1;	// return error, abort handling this message
		break;

	case UBX_RXMSG_ERROR_LENGTH:	// error: invalid length
		UBX_WARN("ubx msg 0x%04x invalid len %u", SWAP16((unsigned)_rx_msg), (unsigned)_rx_payload_length);
		ret = -1;	// return error, abort handling this message
		break;

	default:	// invalid message state
		UBX_WARN("ubx internal err1");
		ret = -1;	// return error, abort handling this message
		break;
	}

	return ret;
}

/**
 * Add payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAdd(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	p_buf[_rx_payload_index] = b;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAddCfgValget(const uint8_t b)
{
	// Accumulate the CFG-VALGET response into our dedicated buffer (never _buf: the response
	// can exceed sizeof(_buf)). payloadRxInit already gated _rx_payload_length <= UBX_VALGET_RX_MAX.
	if (_valget_len < UBX_VALGET_RX_MAX) {
		_valget_storage[_valget_len++] = b;
	}

	int ret = 0;

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAddNavSat(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_nav_sat_part1_t)) {
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_nav_sat_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			_satellite_info->count = MIN(_buf.payload_rx_nav_sat_part1.numSvs, satellite_info_s::SAT_INFO_MAX_SATELLITES);
			UBX_TRACE_SVINFO("SAT len %u  numCh %u", (unsigned)_rx_payload_length,
					 (unsigned)_buf.payload_rx_nav_sat_part1.numSvs);
		}

		if (_rx_payload_index < sizeof(ubx_payload_rx_nav_sat_part1_t) + _satellite_info->count * sizeof(
			    ubx_payload_rx_nav_sat_part2_t)) {
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_sat_part1_t)) % sizeof(
						     ubx_payload_rx_nav_sat_part2_t);
			p_buf[buf_index] = b;

			if (buf_index == sizeof(ubx_payload_rx_nav_sat_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_sat_part1_t)) /
						     sizeof(ubx_payload_rx_nav_sat_part2_t);

				// convert gnssId:svId to a 8 bit number (use svId numbering from NAV-SVINFO)
				uint8_t ubx_sat_gnssId = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.gnssId);
				uint8_t ubx_sat_svId = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.svId);

				uint8_t svinfo_svid = 255;

				switch (ubx_sat_gnssId) {
				case 0:  // GPS: G1-G23 -> 1-32
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 32) {
						svinfo_svid = ubx_sat_svId;
					}

					break;

				case 1:  // SBAS: S120-S158 -> 120-158
					if (ubx_sat_svId >= 120 && ubx_sat_svId <= 158) {
						svinfo_svid = ubx_sat_svId;
					}

					break;

				case 2:  // Galileo: E1-E36 -> 211-246
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 36) {
						svinfo_svid = ubx_sat_svId + 210;
					}

					break;

				case 3:  // BeiDou: B1-B37 -> 159-163,33-64
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 4) {
						svinfo_svid = ubx_sat_svId + 158;

					} else if (ubx_sat_svId >= 5 && ubx_sat_svId <= 37) {
						svinfo_svid = ubx_sat_svId + 28;
					}

					break;

				case 4:  // IMES: I1-I10 -> 173-182
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 10) {
						svinfo_svid = ubx_sat_svId + 172;
					}

					break;

				case 5:  // QZSS: Q1-A10 -> 193-202
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 10) {
						svinfo_svid = ubx_sat_svId + 192;
					}

					break;

				case 6:  // GLONASS: R1-R32 -> 65-96, R? -> 255
					if (ubx_sat_svId >= 1 && ubx_sat_svId <= 32) {
						svinfo_svid = ubx_sat_svId + 64;
					}

					break;
				}

				_satellite_info->svid[sat_index]	  = svinfo_svid;
				_satellite_info->used[sat_index]	  = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.flags & 0x01);
				_satellite_info->elevation[sat_index] = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.elev);
				_satellite_info->azimuth[sat_index]	  = static_cast<uint8_t>(static_cast<float>(_buf.payload_rx_nav_sat_part2.azim) *
						255.0f / 360.0f);
				_satellite_info->snr[sat_index]		  = static_cast<uint8_t>(_buf.payload_rx_nav_sat_part2.cno);
				_satellite_info->prn[sat_index]		  = svinfo_svid;
				UBX_TRACE_SVINFO("SAT #%02u  svid %3u  used %u  elevation %3u  azimuth %3u  snr %3u  prn %3u",
						 static_cast<unsigned>(sat_index + 1),
						 static_cast<unsigned>(_satellite_info->svid[sat_index]),
						 static_cast<unsigned>(_satellite_info->used[sat_index]),
						 static_cast<unsigned>(_satellite_info->elevation[sat_index]),
						 static_cast<unsigned>(_satellite_info->azimuth[sat_index]),
						 static_cast<unsigned>(_satellite_info->snr[sat_index]),
						 static_cast<unsigned>(_satellite_info->prn[sat_index])
						);
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Add NAV-SVINFO payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAddNavSvinfo(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_nav_svinfo_part1_t)) {
			// Part 1 complete: decode Part 1 buffer
			_satellite_info->count = MIN(_buf.payload_rx_nav_svinfo_part1.numCh, satellite_info_s::SAT_INFO_MAX_SATELLITES);
			UBX_TRACE_SVINFO("SVINFO len %u  numCh %u", (unsigned)_rx_payload_length,
					 (unsigned)_buf.payload_rx_nav_svinfo_part1.numCh);
		}

		if (_rx_payload_index < sizeof(ubx_payload_rx_nav_svinfo_part1_t) + _satellite_info->count * sizeof(
			    ubx_payload_rx_nav_svinfo_part2_t)) {
			// Still room in _satellite_info: fill Part 2 buffer
			unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) % sizeof(
						     ubx_payload_rx_nav_svinfo_part2_t);
			p_buf[buf_index] = b;

			if (buf_index == sizeof(ubx_payload_rx_nav_svinfo_part2_t) - 1) {
				// Part 2 complete: decode Part 2 buffer
				unsigned sat_index = (_rx_payload_index - sizeof(ubx_payload_rx_nav_svinfo_part1_t)) /
						     sizeof(ubx_payload_rx_nav_svinfo_part2_t);
				_satellite_info->svid[sat_index]      = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.svid);
				_satellite_info->used[sat_index]      = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.flags >> 3 & 0x01);
				_satellite_info->elevation[sat_index] = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.elev);
				_satellite_info->azimuth[sat_index]   = static_cast<uint8_t>(static_cast<float>(_buf.payload_rx_nav_svinfo_part2.azim) *
									255.0f / 360.0f);
				_satellite_info->snr[sat_index]       = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.cno);
				_satellite_info->prn[sat_index]       = static_cast<uint8_t>(_buf.payload_rx_nav_svinfo_part2.svid);

				UBX_TRACE_SVINFO("SVINFO #%02u  svid %3u  used %u  elevation %3u  azimuth %3u  snr %3u  prn %3u",
						 static_cast<unsigned>(sat_index + 1),
						 static_cast<unsigned>(_satellite_info->svid[sat_index]),
						 static_cast<unsigned>(_satellite_info->used[sat_index]),
						 static_cast<unsigned>(_satellite_info->elevation[sat_index]),
						 static_cast<unsigned>(_satellite_info->azimuth[sat_index]),
						 static_cast<unsigned>(_satellite_info->snr[sat_index]),
						 static_cast<unsigned>(_satellite_info->prn[sat_index])
						);
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Add MON-VER payload rx byte
 */
int	// -1 = error, 0 = ok, 1 = payload completed
GPSDriverUBX::payloadRxAddMonVer(const uint8_t b)
{
	int ret = 0;
	uint8_t *p_buf = (uint8_t *)&_buf;

	if (_rx_payload_index < sizeof(ubx_payload_rx_mon_ver_part1_t)) {
		// Fill Part 1 buffer
		p_buf[_rx_payload_index] = b;

	} else {
		if (_rx_payload_index == sizeof(ubx_payload_rx_mon_ver_part1_t)) {
			// Part 1 complete: decode Part 1 buffer and calculate hash for SW&HW version strings
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.swVersion, FNV1_32_INIT);
			_ubx_version = fnv1_32_str(_buf.payload_rx_mon_ver_part1.hwVersion, _ubx_version);
			UBX_DEBUG("VER hash 0x%08x", (uint16_t)_ubx_version);
			UBX_DEBUG("VER hw  \"%10s\"", _buf.payload_rx_mon_ver_part1.hwVersion);
			UBX_DEBUG("VER sw  \"%30s\"", _buf.payload_rx_mon_ver_part1.swVersion);

			// Device detection (See https://forum.u-blox.com/index.php/9432/need-help-decoding-ubx-mon-ver-hardware-string)
			if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00040005",
				    sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox5;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00040007",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox6;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00070000",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox7;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00080000",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox8;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "00190000",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox9;

			} else if (strncmp((const char *)_buf.payload_rx_mon_ver_part1.hwVersion, "000A0000",
					   sizeof(_buf.payload_rx_mon_ver_part1.hwVersion)) == 0) {
				_board = Board::u_blox10;

			} else {
				UBX_WARN("unknown board hw: %s", _buf.payload_rx_mon_ver_part1.hwVersion);
			}

			UBX_DEBUG("detected board: %i", static_cast<int>(_board));
		}

		// fill Part 2 buffer
		unsigned buf_index = (_rx_payload_index - sizeof(ubx_payload_rx_mon_ver_part1_t)) % sizeof(
					     ubx_payload_rx_mon_ver_part2_t);
		p_buf[buf_index] = b;

		if (buf_index == sizeof(ubx_payload_rx_mon_ver_part2_t) - 1) {
			// Part 2 complete: decode Part 2 buffer
			UBX_DEBUG("VER ext \" %30s\"", _buf.payload_rx_mon_ver_part2.extension);

			// "FWVER=" Firmware of product category and version
			const char *fwver_str = strstr((const char *)_buf.payload_rx_mon_ver_part2.extension, "FWVER=");

			if (fwver_str != nullptr) {
				GPS_INFO("u-blox firmware version: %s", fwver_str + strlen("FWVER="));

				// Check if its a ZED-F9P-15B
				if ((_board == Board::u_blox9) && strstr(fwver_str, "HPGL1L5")) {
					_board = Board::u_blox9_F9P_L1L5;
					UBX_DEBUG("F9P-15B detected");
				}
			}

			// "PROTVER=" Supported protocol version.
			const char *protver_str = strstr((const char *)_buf.payload_rx_mon_ver_part2.extension, "PROTVER=");

			if (protver_str != nullptr) {
				GPS_INFO("u-blox protocol version: %s", protver_str + strlen("PROTVER="));
			}

			// "MOD=" Module identification. Set in production.
			const char *mod_str = strstr((const char *)_buf.payload_rx_mon_ver_part2.extension, "MOD=");

			if (mod_str != nullptr) {
				// in case of u-blox9 family, check if it's an F9P
				if (_board == Board::u_blox9) {
					if (strstr(mod_str, "F9P")) {
						_board = Board::u_blox9_F9P_L1L2;
						UBX_DEBUG("F9P detected");
					}
				}

				GPS_INFO("u-blox module: %s", mod_str + strlen("MOD="));
			}
		}
	}

	if (++_rx_payload_index >= _rx_payload_length) {
		ret = 1;	// payload received completely
	}

	return ret;
}

/**
 * Finish payload rx
 */
int	// 0 = no message handled, 1 = message handled, 2 = sat info message handled
GPSDriverUBX::payloadRxDone()
{
	int ret = 0;

	// return if no message handled
	if (_rx_state != UBX_RXMSG_HANDLE) {
		return ret;
	}

	// handle message
	switch (_rx_msg) {

	case UBX_MSG_NAV_PVT:
		UBX_TRACE_RXMSG("Rx NAV-PVT");

		/* CSV log of UBX-NAV-PVT per u-blox F9 HPG 1.32 Interface Description (UBX-22008968) p.146-148.
		 * Split across two PX4_INFO_RAW calls due to per-call print buffer length limit.
		 *
		 * Line 1 (prefix + meta + time/date/validity + fix):
		 *   prefix, now_us, iTOW [ms], year, month, day, hour, min, sec,
		 *   valid (bitfield), tAcc [ns], nano [ns],
		 *   fixType, flags (bitfield), flags2 (bitfield), numSV
		 *
		 * Line 2 (continuation prefix + position/velocity/accuracy/heading):
		 *   prefix, now_us, lon [1e-7 deg], lat [1e-7 deg], height [mm], hMSL [mm],
		 *   hAcc [mm], vAcc [mm], velN [mm/s], velE [mm/s], velD [mm/s],
		 *   gSpeed [mm/s], headMot [1e-5 deg], sAcc [mm/s], headAcc [1e-5 deg],
		 *   pDOP [0.01], flags3 (bitfield), reserved0 (5 bytes, printed as u32 low word),
		 *   headVeh [1e-5 deg]
		 *
		 * Note: magDec/magAcc fields from F9 spec are not present in local
		 *       ubx_payload_rx_nav_pvt_t and are therefore not logged. */
		{
		hrt_abstime now = hrt_absolute_time();
		static hrt_abstime now_prev = 0;

		// Sample the gap once: hrt_elapsed_time() advances between calls, so reading it
		// separately for the interval and the rate reported two different measurements.
		// Derive it from `now` so the printed gap matches the timestamp in the CSV line below.
		const hrt_abstime pvt_gap_us = (now_prev != 0) ? (now - now_prev) : 0;

		if (pvt_gap_us > 0 && _nav_pvt_warn_period_us != 0 && pvt_gap_us > _nav_pvt_warn_period_us) {
			PX4_WARN("UBX NAV-PVT time jump detected - %.1fms (%.1fHz)",
				 (double)pvt_gap_us * 1e-3, 1e6 / (double)pvt_gap_us);
		}

		now_prev = now;

		PRIME_LOG("%s,%llu,%u,%u,%u,%d,%u,%u,%u,%u\r\n",
			     UBX_NAV_PVT_PREFIX,
			     (unsigned long long)now,
			     (unsigned)_buf.payload_rx_nav_pvt.iTOW,
			     (unsigned)_buf.payload_rx_nav_pvt.min,
			     (unsigned)_buf.payload_rx_nav_pvt.sec,
			     (int)_buf.payload_rx_nav_pvt.nano,
			     (unsigned)_buf.payload_rx_nav_pvt.fixType,
			     (unsigned)_buf.payload_rx_nav_pvt.flags,
			     (unsigned)_buf.payload_rx_nav_pvt.flags2,
			     (unsigned)_buf.payload_rx_nav_pvt.numSV);
		PRIME_LOG("%s,%llu,%d,%d,%d,%d,%u,%u,%d,%d,%d,%d,%d,%u,%u,%u\r\n",
			     UBX_NAV_PVT_PREFIX,
			     (unsigned long long)now,
			     (int)_buf.payload_rx_nav_pvt.lon,
			     (int)_buf.payload_rx_nav_pvt.lat,
			     (int)_buf.payload_rx_nav_pvt.height,
			     (int)_buf.payload_rx_nav_pvt.hMSL,
			     (unsigned)_buf.payload_rx_nav_pvt.hAcc,
			     (unsigned)_buf.payload_rx_nav_pvt.vAcc,
			     (int)_buf.payload_rx_nav_pvt.velN,
			     (int)_buf.payload_rx_nav_pvt.velE,
			     (int)_buf.payload_rx_nav_pvt.velD,
			     (int)_buf.payload_rx_nav_pvt.gSpeed,
			     (int)_buf.payload_rx_nav_pvt.headMot,
			     (unsigned)_buf.payload_rx_nav_pvt.sAcc,
			     (unsigned)_buf.payload_rx_nav_pvt.pDOP,
			     (unsigned)_buf.payload_rx_nav_pvt.flags3);

		//Check if position fix flag is good
		if ((_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_GNSSFIXOK) == 1) {
			_gps_position->fix_type		 = _buf.payload_rx_nav_pvt.fixType;

			if (_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN) {
				_gps_position->fix_type = 4; //DGPS
			}

			uint8_t carr_soln = _buf.payload_rx_nav_pvt.flags >> 6;

			if (carr_soln == 1) {
				_gps_position->fix_type = 5; //Float RTK

			} else if (carr_soln == 2) {
				_gps_position->fix_type = 6; //Fixed RTK
			}

			_gps_position->vel_ned_valid = true;

		} else {
			_gps_position->fix_type		 = 0;
			_gps_position->vel_ned_valid = false;
		}

		_gps_position->satellites_used	= _buf.payload_rx_nav_pvt.numSV;
		_gps_position->last_correction_age = (_buf.payload_rx_nav_pvt.flags3 & UBX_RX_NAV_PVT_FLAGS_LAST_CORR_AGE_MASK) >> 1;

		// ---- RTK-state diagnostics (throttled) --------------------------------------
		// carrSoln (0=none, 1=float, 2=fixed), diffSoln, numSV and the receiver's own
		// correction-age code tell us at a glance whether corrections reach the engine.
		{
			const uint8_t carr_soln_dbg = (_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_CARRSOLN) >> 6;
			const bool    diff_soln_dbg = (_buf.payload_rx_nav_pvt.flags & UBX_RX_NAV_PVT_FLAGS_DIFFSOLN) != 0;
			const hrt_abstime now_pvt   = hrt_absolute_time();
			static hrt_abstime rtk_state_last_summary = 0;

			if (now_pvt - rtk_state_last_summary > UBX_RTCM_LOG_SUMMARY_INTERVAL) {
				rtk_state_last_summary = now_pvt;
				const char *soln = (carr_soln_dbg == 2) ? "RTK-FIXED" :
						   (carr_soln_dbg == 1) ? "RTK-FLOAT" : "none";
				PRIME_LOG("[INFO] RTK state: carrSoln %u (%s), diffSoln %u, fixType %u, numSV %u, corrAgeCode %u\r\n",
					  carr_soln_dbg, soln, (unsigned)diff_soln_dbg,
					  (unsigned)_buf.payload_rx_nav_pvt.fixType,
					  (unsigned)_buf.payload_rx_nav_pvt.numSV,
					  (unsigned)_gps_position->last_correction_age);
			}

			// Watchdog: if corrections are clearly flowing (float/fixed or diffSoln)
			// yet UBX-RXM-RTCM never arrived, it is not enabled on the F9P active port.
			if (_rxm_rtcm_last_seen == 0 && (carr_soln_dbg != 0 || diff_soln_dbg)
			    && now_pvt - _rxm_rtcm_last_absent_warn > UBX_RTCM_LOG_ABSENT_INTERVAL) {
				_rxm_rtcm_last_absent_warn = now_pvt;
				PRIME_LOG("[WARN] RTK: corrections active but UBX-RXM-RTCM never seen - enable it on F9P port\r\n");
			}
		}

		if (_gps_position->fix_type < 99) { // Continue receiving non HPPOS even in RTK mode.
			// When RTK is active and solid (fix=6), these values will be filled by HPPOSLLH:
			_gps_position->latitude_deg		= _buf.payload_rx_nav_pvt.lat * 1e-7;
			_gps_position->longitude_deg		= _buf.payload_rx_nav_pvt.lon * 1e-7;
			_gps_position->altitude_msl_m		= _buf.payload_rx_nav_pvt.hMSL * 1e-3;
			_gps_position->altitude_ellipsoid_m	= _buf.payload_rx_nav_pvt.height * 1e-3;

			_gps_position->eph		= static_cast<float>(_buf.payload_rx_nav_pvt.hAcc) * 1e-3f;
			_gps_position->epv		= static_cast<float>(_buf.payload_rx_nav_pvt.vAcc) * 1e-3f;

			_rate_count_lat_lon++;
			_got_posllh = true;
		}

		_gps_position->s_variance_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.sAcc) * 1e-3f;

		_gps_position->vel_m_s		= static_cast<float>(_buf.payload_rx_nav_pvt.gSpeed) * 1e-3f;

		_gps_position->vel_n_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.velN) * 1e-3f;
		_gps_position->vel_e_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.velE) * 1e-3f;
		_gps_position->vel_d_m_s	= static_cast<float>(_buf.payload_rx_nav_pvt.velD) * 1e-3f;

		_gps_position->cog_rad		= static_cast<float>(_buf.payload_rx_nav_pvt.headMot) * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad	= static_cast<float>(_buf.payload_rx_nav_pvt.headAcc) * M_DEG_TO_RAD_F * 1e-5f;

		//Check if time and date fix flags are good
		if ((_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDDATE)
		    && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_VALIDTIME)
		    && (_buf.payload_rx_nav_pvt.valid & UBX_RX_NAV_PVT_VALID_FULLYRESOLVED)) {
#ifndef NO_MKTIME
			/* convert to unix timestamp */
			tm timeinfo{};
			timeinfo.tm_year	= _buf.payload_rx_nav_pvt.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_pvt.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_pvt.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_pvt.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_pvt.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_pvt.sec;


			time_t epoch = mktime(&timeinfo);

			if (epoch > GPS_EPOCH_SECS) {
				// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
				// and control its drift. Since we rely on the HRT for our monotonic
				// clock, updating it from time to time is safe.

				timespec ts{};
				ts.tv_sec = epoch;
				ts.tv_nsec = _buf.payload_rx_nav_pvt.nano;

				setClock(ts);

				_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
				_gps_position->time_utc_usec += _buf.payload_rx_nav_pvt.nano / 1000;

			} else {
				_gps_position->time_utc_usec = 0;
			}

#else
			_gps_position->time_utc_usec = 0;
#endif
		}

		_gps_position->timestamp = gps_absolute_time();
		_last_timestamp_time = _gps_position->timestamp;

		_rate_count_vel++;
		_got_velned = true;

		ret = 1;
		}
		break;

	case UBX_MSG_INF_DEBUG:
	case UBX_MSG_INF_NOTICE: {
			uint8_t *p_buf = (uint8_t *)&_buf;
			p_buf[_rx_payload_length] = 0;
			UBX_DEBUG("ubx msg: %s", p_buf);
		}
		break;

	case UBX_MSG_INF_ERROR:
	case UBX_MSG_INF_WARNING: {
			uint8_t *p_buf = (uint8_t *)&_buf;
			p_buf[_rx_payload_length] = 0;
			UBX_WARN("ubx msg: %s", p_buf);
		}
		break;

	case UBX_MSG_NAV_POSLLH:
		UBX_TRACE_RXMSG("Rx NAV-POSLLH");

		_gps_position->latitude_deg	= _buf.payload_rx_nav_posllh.lat * 1e-7;
		_gps_position->longitude_deg	= _buf.payload_rx_nav_posllh.lon * 1e-7;
		_gps_position->altitude_msl_m	= _buf.payload_rx_nav_posllh.hMSL * 1e-3;
		_gps_position->altitude_ellipsoid_m = _buf.payload_rx_nav_posllh.height * 1e-3;
		_gps_position->eph	= static_cast<float>(_buf.payload_rx_nav_posllh.hAcc) * 1e-3f; // from mm to m
		_gps_position->epv	= static_cast<float>(_buf.payload_rx_nav_posllh.vAcc) * 1e-3f; // from mm to m

		_gps_position->timestamp = gps_absolute_time();

		_rate_count_lat_lon++;
		_got_posllh = true;

		ret = 1;
		break;

	case UBX_MSG_NAV_HPPOSLLH:
		UBX_TRACE_RXMSG("Rx NAV-HPPOSLLH");

		if (_buf.payload_rx_nav_hpposllh.flags == 0 && _gps_position->fix_type == 6) {
			_gps_position->latitude_deg	= _buf.payload_rx_nav_hpposllh.lat * 1e-7 + _buf.payload_rx_nav_hpposllh.latHp *
							  1e-9;  // regular precision lat/lon (1e7), plus high precision (1e9)
			_gps_position->longitude_deg	= _buf.payload_rx_nav_hpposllh.lon * 1e-7 + _buf.payload_rx_nav_hpposllh.lonHp * 1e-9;
			_gps_position->altitude_msl_m = _buf.payload_rx_nav_hpposllh.hMSL * 1e-3 + _buf.payload_rx_nav_hpposllh.hMSLHp *
							1e-4;	// regular precision altitude, mm, plus high precision components of altitude, 0.1 mm
			_gps_position->altitude_ellipsoid_m = _buf.payload_rx_nav_hpposllh.height * 1e-3 + _buf.payload_rx_nav_hpposllh.heightHp
							      * 1e-4;
			_gps_position->eph	= static_cast<float>(_buf.payload_rx_nav_hpposllh.hAcc) *
						  1e-4f; // Accuracy estimates, convert from 0.1 mm to m
			_gps_position->epv	= static_cast<float>(_buf.payload_rx_nav_hpposllh.vAcc) * 1e-4f;

			_gps_position->timestamp = gps_absolute_time();

			_rate_count_lat_lon++;
			_got_posllh = true;

			ret = 1;
		}

		break;

	case UBX_MSG_NAV_SOL:
		UBX_TRACE_RXMSG("Rx NAV-SOL");

		_gps_position->fix_type		= _buf.payload_rx_nav_sol.gpsFix;
		_gps_position->s_variance_m_s	= static_cast<float>(_buf.payload_rx_nav_sol.sAcc) * 1e-2f;	// from cm to m
		_gps_position->satellites_used	= _buf.payload_rx_nav_sol.numSV;

		ret = 1;
		break;

	case UBX_MSG_NAV_STATUS:
		UBX_TRACE_RXMSG("Rx NAV-STATUS");

		_gps_position->spoofing_state = (_buf.payload_rx_nav_status.flags2 & UBX_RX_NAV_STATUS_SPOOFDETSTATE_MASK) >>
						UBX_RX_NAV_STATUS_SPOOFDETSTATE_SHIFT;

		ret = 1;
		break;

	case UBX_MSG_NAV_DOP:
		UBX_TRACE_RXMSG("Rx NAV-DOP");

		_gps_position->hdop		= _buf.payload_rx_nav_dop.hDOP * 0.01f;	// from cm to m
		_gps_position->vdop		= _buf.payload_rx_nav_dop.vDOP * 0.01f;	// from cm to m
		_gps_position->pdop		= _buf.payload_rx_nav_dop.pDOP * 0.01f;	// from cm to m //TODO (dekel): test behavior

		/* CSV log: prefix, now_us, iTOW and DOPs (as floats) */
		// PX4_INFO_RAW("%s,%llu,%u,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
		// 	UBX_NAV_DOP_PREFIX,
		// 	(unsigned long long)hrt_absolute_time(),
		// 	(unsigned int)_buf.payload_rx_nav_dop.iTOW,
		// 	(double)(_buf.payload_rx_nav_dop.gDOP * 0.01f),
		// 	(double)(_buf.payload_rx_nav_dop.pDOP * 0.01f),
		// 	(double)(_buf.payload_rx_nav_dop.tDOP * 0.01f),
		// 	(double)(_buf.payload_rx_nav_dop.vDOP * 0.01f),
		// 	(double)(_buf.payload_rx_nav_dop.hDOP * 0.01f),
		// 	(double)(_buf.payload_rx_nav_dop.nDOP * 0.01f),
		// 	(double)(_buf.payload_rx_nav_dop.eDOP * 0.01f));

		ret = 1;
		break;

	case UBX_MSG_NAV_TIMEUTC:
		UBX_TRACE_RXMSG("Rx NAV-TIMEUTC");

		if (_buf.payload_rx_nav_timeutc.valid & UBX_RX_NAV_TIMEUTC_VALID_VALIDUTC) {
#ifndef NO_MKTIME
			// convert to unix timestamp
			tm timeinfo {};
			timeinfo.tm_year	= _buf.payload_rx_nav_timeutc.year - 1900;
			timeinfo.tm_mon		= _buf.payload_rx_nav_timeutc.month - 1;
			timeinfo.tm_mday	= _buf.payload_rx_nav_timeutc.day;
			timeinfo.tm_hour	= _buf.payload_rx_nav_timeutc.hour;
			timeinfo.tm_min		= _buf.payload_rx_nav_timeutc.min;
			timeinfo.tm_sec		= _buf.payload_rx_nav_timeutc.sec;
			timeinfo.tm_isdst	= 0;

			time_t epoch = mktime(&timeinfo);

			// only set the time if it makes sense

			if (epoch > GPS_EPOCH_SECS) {
				// FMUv2+ boards have a hardware RTC, but GPS helps us to configure it
				// and control its drift. Since we rely on the HRT for our monotonic
				// clock, updating it from time to time is safe.

				timespec ts{};
				ts.tv_sec = epoch;
				ts.tv_nsec = _buf.payload_rx_nav_timeutc.nano;

				setClock(ts);

				_gps_position->time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
				_gps_position->time_utc_usec += _buf.payload_rx_nav_timeutc.nano / 1000;

			} else {
				_gps_position->time_utc_usec = 0;
			}

#else
			_gps_position->time_utc_usec = 0;
#endif
		}

		_last_timestamp_time = gps_absolute_time();

		ret = 1;
		break;

	case UBX_MSG_NAV_SAT:
	case UBX_MSG_NAV_SVINFO:
		UBX_TRACE_RXMSG("Rx NAV-SVINFO");

		// _satellite_info already populated by payload_rx_add_svinfo(), just add a timestamp
		_satellite_info->timestamp = gps_absolute_time();

		ret = 2;
		break;

	case UBX_MSG_NAV_SVIN:
		UBX_TRACE_RXMSG("Rx NAV-SVIN");
		{
			ubx_payload_rx_nav_svin_t &svin = _buf.payload_rx_nav_svin;

			UBX_DEBUG("Survey-in status: %lus cur accuracy: %lumm nr obs: %lu valid: %i active: %i",
				  svin.dur, svin.meanAcc / 10, svin.obs, static_cast<int>(svin.valid), static_cast<int>(svin.active));

			SurveyInStatus status{};
			double ecef_x = (static_cast<double>(svin.meanX) + static_cast<double>(svin.meanXHP) * 0.01) * 0.01;
			double ecef_y = (static_cast<double>(svin.meanY) + static_cast<double>(svin.meanYHP) * 0.01) * 0.01;
			double ecef_z = (static_cast<double>(svin.meanZ) + static_cast<double>(svin.meanZHP) * 0.01) * 0.01;
			ECEF2lla(ecef_x, ecef_y, ecef_z, status.latitude, status.longitude, status.altitude);
			status.duration = svin.dur;
			status.mean_accuracy = svin.meanAcc / 10;
			status.flags = (svin.valid & 1) | ((svin.active & 1) << 1);
			surveyInStatus(status);

			if (svin.valid == 1 && svin.active == 0) {
				if (activateRTCMOutput(true) != 0) {
					return 0;
				}
			}
		}

		ret = 1;
		break;

	case UBX_MSG_NAV_VELNED:
		UBX_TRACE_RXMSG("Rx NAV-VELNED");

		_gps_position->vel_m_s        = static_cast<float>(_buf.payload_rx_nav_velned.gSpeed) * 1e-2f;
		_gps_position->vel_n_m_s      = static_cast<float>(_buf.payload_rx_nav_velned.velN)  * 1e-2f; // NED NORTH velocity
		_gps_position->vel_e_m_s      = static_cast<float>(_buf.payload_rx_nav_velned.velE)  * 1e-2f; // NED EAST velocity
		_gps_position->vel_d_m_s      = static_cast<float>(_buf.payload_rx_nav_velned.velD)  * 1e-2f; // NED DOWN velocity
		_gps_position->cog_rad        = static_cast<float>(_buf.payload_rx_nav_velned.heading) * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->c_variance_rad = static_cast<float>(_buf.payload_rx_nav_velned.cAcc)    * M_DEG_TO_RAD_F * 1e-5f;
		_gps_position->vel_ned_valid  = true;

		_rate_count_vel++;
		_got_velned = true;

		ret = 1;
		break;

	case UBX_MSG_NAV_RELPOSNED:
		UBX_TRACE_RXMSG("Rx NAV-RELPOSNED");

		if ((_mode == UBXMode::RoverWithMovingBase) || (_mode == UBXMode::RoverWithMovingBaseUART1)) {
			float heading = _buf.payload_rx_nav_relposned.relPosHeading * 1e-5f;
			float heading_acc = _buf.payload_rx_nav_relposned.accHeading * 1e-5f;
			float rel_length = _buf.payload_rx_nav_relposned.relPosLength + _buf.payload_rx_nav_relposned.relPosHPLength * 1e-2f;
			float rel_length_acc = _buf.payload_rx_nav_relposned.accLength * 1e-2f;
			bool heading_valid = _buf.payload_rx_nav_relposned.flags & (1 << 8);
			bool rel_pos_valid = _buf.payload_rx_nav_relposned.flags & (1 << 2);
			bool carrier_solution_fixed = _buf.payload_rx_nav_relposned.flags & (1 << 4);
			(void)rel_length_acc;

			if (heading_valid && rel_pos_valid && rel_length < 1000.f && carrier_solution_fixed) { // validity & sanity checks
				heading *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]
				heading -= _heading_offset; // range: [-pi, 3pi]

				if (heading > M_PI_F) {
					heading -= 2.f * M_PI_F; // final range is [-pi, pi]
				}

				_gps_position->heading = heading;

				heading_acc *= M_PI_F / 180.0f; // deg to rad, now in range [0, 2pi]

				_gps_position->heading_accuracy = heading_acc;

				UBX_DEBUG("Heading: %.3f rad, acc: %.1f deg, relLen: %.1f cm, relAcc: %.1f cm, valid: %i %i", (double)heading,
					  (double)heading_acc, (double)rel_length, (double)rel_length_acc, heading_valid, rel_pos_valid);
			}

			ret = 1;
		}

		{
			sensor_gnss_relative_s gps_rel{};

			gps_rel.timestamp_sample = gps_absolute_time(); // TODO: adjust with delay estimate

			gps_rel.time_utc_usec = _buf.payload_rx_nav_relposned.iTOW * 1000; // TODO: convert iTOW ms GPS time of week
			gps_rel.reference_station_id = _buf.payload_rx_nav_relposned.refStationId;

			// relPosN + (relPosHPN * 1e-2), relPosHPN is 0.1 mm
			gps_rel.position[0] = (_buf.payload_rx_nav_relposned.relPosN + _buf.payload_rx_nav_relposned.relPosHPN * 1e-2f) * 1e-2f;
			gps_rel.position[1] = (_buf.payload_rx_nav_relposned.relPosE + _buf.payload_rx_nav_relposned.relPosHPE * 1e-2f) * 1e-2f;
			gps_rel.position[2] = (_buf.payload_rx_nav_relposned.relPosD + _buf.payload_rx_nav_relposned.relPosHPD * 1e-2f) * 1e-2f;

			// full length of the relative position vector, in units of cm, is given by relPosLength + (relPosHPLength * 1e-2)
			gps_rel.position_length = (_buf.payload_rx_nav_relposned.relPosLength
						   + _buf.payload_rx_nav_relposned.relPosHPLength * 1e-2f) * 1e-2f;

			gps_rel.heading = _buf.payload_rx_nav_relposned.relPosHeading * 1e-5f * (M_PI_F / 180.f);  // 1e-5 deg -> radians
			gps_rel.heading_accuracy = _buf.payload_rx_nav_relposned.accHeading * 1e-5f * (M_PI_F / 180.f); // 1e-5 deg -> radians

			// Accuracy of relative position in 0.1 mm
			gps_rel.position_accuracy[0] = _buf.payload_rx_nav_relposned.accN * 1e-4f; // 0.1mm -> m
			gps_rel.position_accuracy[1] = _buf.payload_rx_nav_relposned.accE * 1e-4f; // 0.1mm -> m
			gps_rel.position_accuracy[2] = _buf.payload_rx_nav_relposned.accD * 1e-4f; // 0.1mm -> m

			gps_rel.accuracy_length = _buf.payload_rx_nav_relposned.accLength * 1e-4f; // 0.1mm -> m

			gps_rel.gnss_fix_ok                  = _buf.payload_rx_nav_relposned.flags & (1 << 0);
			gps_rel.differential_solution        = _buf.payload_rx_nav_relposned.flags & (1 << 1);
			gps_rel.relative_position_valid      = _buf.payload_rx_nav_relposned.flags & (1 << 2);
			gps_rel.carrier_solution_floating    = _buf.payload_rx_nav_relposned.flags & (1 << 3);
			gps_rel.carrier_solution_fixed       = _buf.payload_rx_nav_relposned.flags & (1 << 4);
			gps_rel.moving_base_mode             = _buf.payload_rx_nav_relposned.flags & (1 << 5);
			gps_rel.reference_position_miss      = _buf.payload_rx_nav_relposned.flags & (1 << 6);
			gps_rel.reference_observations_miss  = _buf.payload_rx_nav_relposned.flags & (1 << 7);
			gps_rel.heading_valid                = _buf.payload_rx_nav_relposned.flags & (1 << 8);
			gps_rel.relative_position_normalized = _buf.payload_rx_nav_relposned.flags & (1 << 9);

			gotRelativePositionMessage(gps_rel);
		}

		break;

	case UBX_MSG_MON_VER:
		UBX_TRACE_RXMSG("Rx MON-VER");

		// This is polled only on startup, and the startup code waits for an ack
		if (_ack_state == UBX_ACK_WAITING && _ack_waiting_msg == UBX_MSG_MON_VER) {
			_ack_state = UBX_ACK_GOT_ACK;
		}

		ret = 1;
		break;

	case UBX_MSG_MON_HW:
		UBX_TRACE_RXMSG("Rx MON-HW");

		switch (_rx_payload_length) {

		case sizeof(ubx_payload_rx_mon_hw_ubx6_t):	/* u-blox 6 msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx6.noisePerMS;
			_gps_position->automatic_gain_control   = _buf.payload_rx_mon_hw_ubx6.agcCnt;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx6.jamInd;

			ret = 1;
			break;

		case sizeof(ubx_payload_rx_mon_hw_ubx7_t):	/* u-blox 7+ msg format */
			_gps_position->noise_per_ms		= _buf.payload_rx_mon_hw_ubx7.noisePerMS;
			_gps_position->automatic_gain_control   = _buf.payload_rx_mon_hw_ubx7.agcCnt;
			_gps_position->jamming_indicator	= _buf.payload_rx_mon_hw_ubx7.jamInd;

			ret = 1;
			break;

		case sizeof(ubx_payload_rx_mon_hw_deprecated_t):	/* u-blox 27+ deprecated, ignore */
			ret = 0;
			break;

		default:		// unexpected payload size:
			ret = 0;	// don't handle message
			break;
		}

		break;

	case UBX_MSG_MON_RF:
		UBX_TRACE_RXMSG("Rx MON-RF");

		_gps_position->noise_per_ms		= _buf.payload_rx_mon_rf.block[0].noisePerMS;
		_gps_position->jamming_indicator	= _buf.payload_rx_mon_rf.block[0].jamInd;
		_gps_position->jamming_state		= _buf.payload_rx_mon_rf.block[0].flags;

		ret = 1;
		break;

	case UBX_MSG_RXM_RTCM: {
		UBX_TRACE_RXMSG("Rx RXM-RTCM");

		const bool    crc_failed = (_buf.payload_rx_rxm_rtcm.flags & UBX_RX_RXM_RTCM_CRCFAILED_MASK) != 0;
		const uint8_t msg_used   = (_buf.payload_rx_rxm_rtcm.flags & UBX_RX_RXM_RTCM_MSGUSED_MASK) >>
					   UBX_RX_RXM_RTCM_MSGUSED_SHIFT;

		_gps_position->rtcm_crc_failed = crc_failed;
		_gps_position->rtcm_msg_used   = msg_used;

		// This is the single most valuable RTK diagnostic: per RTCM type, did the
		// F9P actually USE the correction (msgUsed==2) or just receive it? A stream
		// of msgUsed==1 (not used) or crcFailed==1 means corrections arrive but do
		// not feed the RTK engine - exactly the "receiving but never FIXED" symptom.
		const hrt_abstime now = hrt_absolute_time();
		_rxm_rtcm_last_seen = now;
		_rxm_rtcm_count++;

		if (crc_failed) {
			_rxm_rtcm_crc_failed++;

		} else if (msg_used == 2) {
			_rxm_rtcm_used++;

		} else {
			// msg_used == 1 (not used) or 0 (unknown)
			_rxm_rtcm_not_used++;
		}

		// Periodic aggregate: used/not-used/crc-fail ratio at a glance. The ratio is
		// the smoking gun (corrections received but not applied), so one throttled
		// line carries it without a print per message at the RTCM rate.
		if (now - _rxm_rtcm_last_summary > UBX_RTCM_LOG_SUMMARY_INTERVAL) {
			PRIME_LOG("[INFO] %s: %u msgs, %u used, %u not-used, %u crc-fail (last type %u)\r\n",
				  UBX_RXM_RTCM_PREFIX, (unsigned)_rxm_rtcm_count, (unsigned)_rxm_rtcm_used,
				  (unsigned)_rxm_rtcm_not_used, (unsigned)_rxm_rtcm_crc_failed,
				  (unsigned)_buf.payload_rx_rxm_rtcm.msgType);

			_rxm_rtcm_last_summary = now;
			_rxm_rtcm_count = 0;
			_rxm_rtcm_used = 0;
			_rxm_rtcm_not_used = 0;
			_rxm_rtcm_crc_failed = 0;
		}

		ret = 1;
		break;
	}

	case UBX_MSG_CFG_VALGET: {
			UBX_TRACE_RXMSG("Rx CFG-VALGET");

			// Body after the 4-byte header (version, layer, position[2]) is [key][value] TLVs,
			// identical encoding to a VALSET body. Walk it, keep non-zero values, append to the
			// open dump file. Accumulated into _valget_storage (never _buf).
			if (_valget_dump_fd >= 0 && _valget_len > 4) {
				const uint8_t *tlv = &_valget_storage[4];
				const uint16_t tlv_len = (uint16_t)(_valget_len - 4);

				walkCfgTlv(tlv, tlv_len, [&](uint32_t key, const uint8_t *val, uint8_t vlen) {
					// non-zero filter
					bool nonzero = false;

					for (uint8_t i = 0; i < vlen; ++i) {
						if (val[i] != 0) { nonzero = true; break; }
					}

					if (!nonzero) { return; }

					char lb[80];
					int  ln = snprintf(lb, sizeof(lb), "KEY=0x%08x SIZE=%u VALUE=", (unsigned)key, vlen);

					for (uint8_t i = 0; i < vlen; ++i) {
						ln += snprintf(lb + ln, sizeof(lb) - ln, "%02x%s", val[i], (i + 1 < vlen) ? " " : "");
					}

					ln += snprintf(lb + ln, sizeof(lb) - ln, "\n");

					::write(_valget_dump_fd, lb, ln);
					_valget_dump_crc = calculateCRC32((uint32_t)ln, (uint8_t *)lb, _valget_dump_crc);
					++_valget_dump_count;
				});
			}

			_valget_capturing = false;
			ret = 1;
			break;
		}

	case UBX_MSG_ACK_ACK:
		UBX_TRACE_RXMSG("Rx ACK-ACK");

		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_ACK;
		}

		ret = 1;
		break;

	case UBX_MSG_ACK_NAK:
		UBX_TRACE_RXMSG("Rx ACK-NAK");

		if ((_ack_state == UBX_ACK_WAITING) && (_buf.payload_rx_ack_ack.msg == _ack_waiting_msg)) {
			_ack_state = UBX_ACK_GOT_NAK;
		}

		ret = 1;
		break;

	default:
		break;
	}

	if (ret > 0) {
		_gps_position->timestamp_time_relative = (int32_t)(_last_timestamp_time - _gps_position->timestamp);
	}

	return ret;
}

int
GPSDriverUBX::activateRTCMOutput(bool reduce_update_rate)
{
	// === DISABLED PERIPHERAL CONFIG WRITES (RTCM3 output activation) ===
	// Originally enables a base-station's RTCM3 correction stream:
	//   * RATE_MEAS (or CFG-RATE on pre-v27) = 1000 ms - drop receiver to 1 Hz
	//     fix rate when reduce_update_rate is set, since base RTCM streams
	//     only need 1 Hz. (Survey-in keeps higher rate to speed convergence.)
	//   * MSGOUT_RTCM_3X_TYPE1005=5, TYPE1077/1087/1097/1127/1230=1 -
	//     enable the receiver's RTCM3 output on each port (or via CFG-MSG on
	//     pre-v27):
	//       1005: stationary RTK reference-station ARP (every 5 epochs).
	//       1077: GPS MSM7 observables.
	//       1087: GLONASS MSM7 observables.
	//       1097: Galileo MSM7 observables.
	//       1127: BeiDou MSM7 observables.
	//       1230: GLONASS code-phase biases.
	//   * MSGOUT_UBX_NAV_SVIN = 0 - stop survey-in status messages now that
	//     survey is complete and RTCM output has taken over.
	// Useful only when this device is the RTK base broadcasting corrections.
	// Disabled - rover does not generate RTCM3 output.
	(void)reduce_update_rate;
	return 0;
#if 0
	/* For base stations we switch to 1 Hz update rate, which is enough for RTCM output.
	 * For the survey-in, we still want 5/10 Hz, because this speeds up the process */

	if (_proto_ver_27_or_higher) {
		int cfg_valset_msg_size = initCfgValset();

		if (reduce_update_rate) {
			cfgValset<uint16_t>(UBX_CFG_KEY_RATE_MEAS, 1000, cfg_valset_msg_size);
		}

		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1005_I2C, 5, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1077_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1087_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1230_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1097_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_RTCM_3X_TYPE1127_I2C, 1, cfg_valset_msg_size);
		cfgValsetPort(UBX_CFG_KEY_MSGOUT_UBX_NAV_SVIN_I2C, 0, cfg_valset_msg_size);

		if (!sendMessage(UBX_MSG_CFG_VALSET, (uint8_t *)&_buf, cfg_valset_msg_size)) {
			return -1;
		}

		if (waitForAck(UBX_MSG_CFG_VALSET, UBX_CONFIG_TIMEOUT, false) < 0) {
			return -1;
		}

	} else {

		if (reduce_update_rate) {
			memset(&_buf.payload_tx_cfg_rate, 0, sizeof(_buf.payload_tx_cfg_rate));
			_buf.payload_tx_cfg_rate.measRate	= 1000;
			_buf.payload_tx_cfg_rate.navRate	= UBX_TX_CFG_RATE_NAVRATE;
			_buf.payload_tx_cfg_rate.timeRef	= UBX_TX_CFG_RATE_TIMEREF;

			if (!sendMessage(UBX_MSG_CFG_RATE, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rate))) { return -1; }

			// according to the spec we should receive an (N)ACK here, but we don't
		}

		configureMessageRate(UBX_MSG_NAV_SVIN, 0);

		// stationary RTK reference station ARP (can be sent at lower rate)
		if (!configureMessageRate(UBX_MSG_RTCM3_1005, 5)) { return -1; }

		// GPS
		if (!configureMessageRate(UBX_MSG_RTCM3_1077, 1)) { return -1; }

		// GLONASS
		if (!configureMessageRate(UBX_MSG_RTCM3_1087, 1)) { return -1; }

		// GLONASS code-phase biases
		if (!configureMessageRate(UBX_MSG_RTCM3_1230, 1)) { return -1; }

		// Galileo
		if (!configureMessageRate(UBX_MSG_RTCM3_1097, 1)) { return -1; }

		// BeiDou
		if (!configureMessageRate(UBX_MSG_RTCM3_1127, 1)) { return -1; }
	}

	return 0;
#endif // activateRTCMOutput body disabled
}

void
GPSDriverUBX::decodeInit()
{
	_decode_state = UBX_DECODE_SYNC1;
	_rx_ck_a = 0;
	_rx_ck_b = 0;
	_rx_payload_length = 0;
	_rx_payload_index = 0;
}

void
GPSDriverUBX::addByteToChecksum(const uint8_t b)
{
	_rx_ck_a = _rx_ck_a + b;
	_rx_ck_b = _rx_ck_b + _rx_ck_a;
}

void
GPSDriverUBX::calcChecksum(const uint8_t *buffer, const uint16_t length, ubx_checksum_t *checksum)
{
	for (uint16_t i = 0; i < length; i++) {
		checksum->ck_a = checksum->ck_a + buffer[i];
		checksum->ck_b = checksum->ck_b + checksum->ck_a;
	}
}

bool
GPSDriverUBX::configureMessageRate(const uint16_t msg, const uint8_t rate)
{
	// === DISABLED PERIPHERAL CONFIG WRITE (legacy CFG-MSG) ===
	// Originally: build a UBX-CFG-MSG packet selecting one message class/ID
	// and its output rate (0 = off, N = once every N nav epochs), then send
	// it to the receiver. Used by the pre-v27 init path to subscribe the
	// host to NAV-*, MON-*, RTCM3-* streams, and by the auto-disable branch
	// in payloadRxInit() to silence unexpected messages. Useful to control
	// which streams the receiver emits on legacy hardware. Disabled - do
	// not push any CFG-MSG rate changes; return true so callers that check
	// the boolean don't treat absence as failure.
	(void)msg; (void)rate;
	return true;
#if 0
	if (_proto_ver_27_or_higher) {
		// configureMessageRate() should not be called if _proto_ver_27_or_higher is true.
		// If you see this message the calling code needs to be fixed.
		UBX_WARN("FIXME: use of deprecated msg CFG_MSG (%i %i)", msg, rate);
	}

	ubx_payload_tx_cfg_msg_t cfg_msg;	// don't use _buf (allow interleaved operation)
	memset(&cfg_msg, 0, sizeof(cfg_msg));

	cfg_msg.msg	= msg;
	cfg_msg.rate	= rate;

	return sendMessage(UBX_MSG_CFG_MSG, (uint8_t *)&cfg_msg, sizeof(cfg_msg));
#endif
}

bool
GPSDriverUBX::configureMessageRateAndAck(uint16_t msg, uint8_t rate, bool report_ack_error)
{
	if (!configureMessageRate(msg, rate)) {
		return false;
	}

	return waitForAck(UBX_MSG_CFG_MSG, UBX_CONFIG_TIMEOUT, report_ack_error) >= 0;
}

bool
GPSDriverUBX::sendMessage(const uint16_t msg, const uint8_t *payload, const uint16_t length)
{
	ubx_header_t   header = {UBX_SYNC1, UBX_SYNC2, 0, 0};
	ubx_checksum_t checksum = {0, 0};

	// Populate header
	header.msg	= msg;
	header.length	= length;

	// Calculate checksum
	calcChecksum(((uint8_t *)&header) + 2, sizeof(header) - 2, &checksum); // skip 2 sync bytes

	if (payload != nullptr) {
		calcChecksum(payload, length, &checksum);
	}

	// Send message
	if (write((void *)&header, sizeof(header)) != sizeof(header)) {
		return false;
	}

	if (payload && write((void *)payload, length) != length) {
		return false;
	}

	if (write((void *)&checksum, sizeof(checksum)) != sizeof(checksum)) {
		return false;
	}

	return true;
}

uint32_t
GPSDriverUBX::fnv1_32_str(uint8_t *str, uint32_t hval)
{
	uint8_t *s = str;

	/*
	 * FNV-1 hash each octet in the buffer
	 */
	while (*s) {

		/* multiply by the 32 bit FNV magic prime mod 2^32 */
#if defined(NO_FNV_GCC_OPTIMIZATION)
		hval *= FNV1_32_PRIME;
#else
		hval += (hval << 1) + (hval << 4) + (hval << 7) + (hval << 8) + (hval << 24);
#endif

		/* xor the bottom with the current octet */
		hval ^= (uint32_t) * s++;
	}

	/* return our new hash value */
	return hval;
}

int
GPSDriverUBX::reset(GPSRestartType restart_type)
{
	// === DISABLED PERIPHERAL CONFIG WRITE (CFG-RST) ===
	// Originally: send UBX-CFG-RST with resetMode = software reset and
	// navBbrMask = HOT/WARM/COLD start to ask the receiver to drop / partially
	// drop / fully drop its battery-backed-RAM (ephemeris, almanac, last fix,
	// clock drift, etc.) and reboot the navigation engine. Useful when
	// commanding a GPS reset from PX4 (e.g. to recover from a stuck fix).
	// Disabled - do not command the receiver to reset; report success without
	// touching the device.
	(void)restart_type;
	return 0;
#if 0
	memset(&_buf.payload_tx_cfg_rst, 0, sizeof(_buf.payload_tx_cfg_rst));
	_buf.payload_tx_cfg_rst.resetMode = UBX_TX_CFG_RST_MODE_SOFTWARE;

	switch (restart_type) {
	case GPSRestartType::Hot:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_HOT_START;
		break;

	case GPSRestartType::Warm:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_WARM_START;
		break;

	case GPSRestartType::Cold:
		_buf.payload_tx_cfg_rst.navBbrMask = UBX_TX_CFG_RST_BBR_MODE_COLD_START;
		break;

	default:
		return -2;
	}

	if (sendMessage(UBX_MSG_CFG_RST, (uint8_t *)&_buf, sizeof(_buf.payload_tx_cfg_rst))) {
		return 0;
	}

	return -2;
#endif
}
