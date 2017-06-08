#ifndef MANIFOLD_UARCH_MEMMSG_H
#define MANIFOLD_UARCH_MEMMSG_H

namespace manifold {
namespace uarch {


//! This class defines messages sent between last level cache and memory control
struct Mem_msg {
public:
    Mem_msg(uint64_t address, bool isRead) : addr(address), read(isRead) {}

    uint64_t addr;
    bool read;
    uint64_t get_addr() { return addr; }
    bool is_read() { return read; }
    int type;   // 0 MEM REQ, 1 MEM RPLY
};


} // namespace uarch
} //namespace manifold

#endif // MANIFOLD_UARCH_MEMMSG_H
