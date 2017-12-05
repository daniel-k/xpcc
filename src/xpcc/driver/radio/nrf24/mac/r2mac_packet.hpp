#ifndef XPCC__NRF24_MAC_R2MAC_PACKET_HPP
#define XPCC__NRF24_MAC_R2MAC_PACKET_HPP

#include <stdint.h>
#include <xpcc/architecture.hpp>

namespace xpcc {

template<typename Config>
class R2MACPacket
{
public:

	class xpcc_packed Packet {

	public:
		enum class
		Type : uint8_t
		{
			Beacon = 0x01,
			AssociationRequest = 0x02,
			Data = 0x03,
			None = 0xff
		};

		static const char*
		toStr(Type type) {
			switch(type) {
			case Type::Beacon:				return "Beacon";
			case Type::AssociationRequest:	return "AssociationRequest";
			case Type::Data:				return "Data";
			case Type::None:				return "None";
			default:						return "Invalid"; }
		}


	private:
		struct xpcc_packed Header
		{
			Type type;
		};

	/// Memory layout is such that an instance of this class can be piped into
	/// the PHY without further alterations.
	private:
		typename Config::L2Header headerBelow;
		Header	header;
	public:
		/// User will put data here
		uint8_t	payload[typename Config::L2Frame::getPayloadLength() - sizeof(Header)];


	public:

		static constexpr uint8_t
		getFrameOverhead()
		{ return sizeof(typename Config::L2Header) + sizeof(Header); }

		xpcc_always_inline void
		setDestination(typename Config::L2Address dest)
		{ headerBelow.setDestination(dest); }

		xpcc_always_inline typename Config::L2Address
		getDestination()
		{ return headerBelow.getDestination(); }

		xpcc_always_inline typename Config::L2Address
		getSource()
		{ return headerBelow.getSource(); }

		xpcc_always_inline void
		setType(Type type)
		{ header.type = type; }

		xpcc_always_inline Type
		getType()
		{ return header.type; }

	};
};

} // namespace xpcc

#endif // XPCC__NRF24_MAC_R2MAC_PACKET_HPP
