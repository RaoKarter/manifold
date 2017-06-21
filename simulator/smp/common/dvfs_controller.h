#ifndef DVFS_CONTROLLER_H
#define DVFS_CONTROLLER_H

#include "kernel/component-decl.h"
#include "kernel/clock.h"
#include "proc_builder.h"
#include "mc_builder.h"
#include "spx/core.h"
#include "hmc_xbar.h"
#include "serdes_link.h"
//#include "kitfox_builder.h"
//#include "kitfox_proxy.h"

#include <map>
#include <vector>
#include <iterator>
#include <string>
#include <iostream>
#include <numeric>


class HMC_builder;
class Spx_builder;
//class KitFoxBuilder;

namespace manifold {
namespace dvfs_controller {

struct dvfs_settings {
//    dvfs_settings(ProcBuilder* pb, MemControllerBuilder* mb, KitFoxBuilder* kb) : proc_builder(pb), mem_builder(mb), kitfox_builder(kb)
    dvfs_settings(ProcBuilder* pb, MemControllerBuilder* mb) : proc_builder(pb), mem_builder(mb)
    {}
    ProcBuilder* proc_builder;
    MemControllerBuilder* mem_builder;
//    KitFoxBuilder* kitfox_builder;
};

class Controller : public manifold::kernel::Component {
public:
    enum {OUT_PORT=0};
    Controller(int, dvfs_settings&, manifold::kernel::Clock&);
    ~Controller() { }

    int get_dvfs_controller_id() { return controller_id; }
    void tick();

    void compute_next_dvfs();

    void print_stats(ostream&);
    void print_config(ostream&);

protected:

private:
    int controller_id;
    manifold::kernel::Clock& controller_clk;
    Spx_builder* p_spx;
    HMC_builder* p_hmc;
//    KitFoxBuilder* p_kfb;
    std::vector<int> proc_cid;
    std::vector<int> serdes_cid;
    std::vector<manifold::spx::spx_core_t*> core;
    std::vector<manifold::hmc_serdes::HMC_SerDes*> serdes;

};

} // namespace dvfs_controller
} // namespace manifold
#endif // DVFS_CONTROLLER_H
