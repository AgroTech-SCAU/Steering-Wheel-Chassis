#ifdef NDEBUG
#undef NDEBUG
#endif
#include "mcu_comm_bridge/binary_frame.hpp"
#include <cassert>
#include <iostream>
using namespace mcu_comm_bridge;
int main() {
    BinaryFrameParser parser;
    auto short_frame = pack_frame(0x28u, 9u, 0u, std::vector<uint8_t>(6u));
    auto frames = parser.feed(short_frame.data(), short_frame.size());
    assert(frames.empty() && "result frames with an invalid payload length must be rejected");
    assert(parser.stats().known_msg_bad_length == 1u);
    std::vector<uint8_t> payload(7u);
    write_u16_le(payload, 0, 65535u);
    payload[2] = 1u;
    write_i32_le(payload, 3, 7);
    auto bytes = pack_frame(0x28u, 10u, 0u, payload);
    frames = parser.feed(bytes.data(), bytes.size());
    assert(frames.size() == 1u);
    assert(read_u16_le(frames[0].payload, 0) == 65535u);
    assert(frames[0].payload[2] == 1u);
    assert(read_i32_le(frames[0].payload, 3) == 7);
    std::cout << "PASS arm command result frame validation and wire round trip\n";
}
