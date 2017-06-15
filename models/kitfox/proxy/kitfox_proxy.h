#ifndef __KITFOX_PROXY_H__
#define __KITFOX_PROXY_H__
#ifdef USE_KITFOX

#include <assert.h>
#include <string>
#include "kitfox.h"
#include "kernel/component.h"
#include "kernel/clock.h"
#include "uarch/kitfoxCounter.h"

namespace manifold {
namespace kitfox_proxy {

template <typename T>
class kitfox_proxy_request_t
{
public:
    kitfox_proxy_request_t() {}
    kitfox_proxy_request_t(int NodeID, manifold::uarch::KitFoxType s, manifold::kernel::Time_t t) : node_id(NodeID), type(s), time(t) {  counter.clear(); }
    kitfox_proxy_request_t(int NodeID, manifold::uarch::KitFoxType s, manifold::kernel::Time_t t, int ranks)
                            : node_id(NodeID), type(s), time(t)
    {
        counter.set_num_dyn_comp(ranks);
    }
    ~kitfox_proxy_request_t() {}
    int get_id() const { return node_id; }
    manifold::uarch::KitFoxType get_type()  const { return type; }
    manifold::kernel::Time_t get_time() const { return time; }
    void set_counter (const T c) { counter = c; }
    T get_counter (void) { return counter;}
    void set_serdes_power(double power)
    {
        cerr << "set_serdes_power= " << power << endl;
        counter.serdes_power = power;
    }
    void set_xbar_power(double power)
    {
        cerr << "set_xbar_power= " << power << endl;
        counter.xbar_power = power;
    }
    void set_dram_power (std::vector<double> power)
    {
        assert(power.size() == counter.ranks);
        cerr << "Vault power";
        for(std::vector<double>::iterator it = counter.vault.begin(); it != counter.vault.end();++it)
            cerr << "\t" << *it;
        cerr << endl;
        counter.vault = power;
    }
    // kitfox_proxy_request_t operator=(const kitfox_proxy_request_t &_)
    // {
    //     core_id = _.core_id;
    //     time = _.time;
    //     counter = _.counter;
    //     name = _.name;

    //     return *this;
    // }

private:
    int node_id;
    manifold::kernel::Time_t time;
    manifold::uarch::KitFoxType type;
    T counter;
};

class kitfox_proxy_t : public manifold::kernel::Component
{
public:
    kitfox_proxy_t(const char *ConfigFile, uint64_t SamplingFreq, int vault_ranks);
    kitfox_proxy_t(const char *ConfigFile, uint64_t SamplingFreq);
    ~kitfox_proxy_t();

    void tick();
    void tock();

    /* Add Manifold components to calculate power. */
    void add_manifold_node(manifold::kernel::CompId_t, manifold::uarch::KitFoxType);
#if 0
    void pair_counter_to_power_component(manifold::kernel::CompId_t);
    /* Add KitFox components to invoke calculations. */
    void add_kitfox_power_component(libKitFox::Comp_ID);
    void add_kitfox_thermal_component(libKitFox::Comp_ID);
    void add_kitfox_reliability_component(libKitFox::Comp_ID);
#endif

    void calculate_power(manifold::uarch::pipeline_counter_t c, manifold::kernel::Time_t t, const string prefix);
    void calculate_power(manifold::uarch::cache_counter_t c, manifold::kernel::Time_t t, const string prefix);
    void calculate_power(manifold::uarch::hmcxbar_counter_t c, manifold::kernel::Time_t t, const string prefix);
    void calculate_power(manifold::uarch::hmcserdes_counter_t c, manifold::kernel::Time_t t, const string prefix);
    void calculate_power(manifold::uarch::dram_power_t c, manifold::kernel::Time_t t, const string vault_id);
    template <typename T> void handle_kitfox_proxy_response(int temp, kitfox_proxy_request_t<T> *Req);
    int synchronize_vault_power(manifold::kernel::Time_t time);
    int synchronize_logic_layer_power(manifold::kernel::Time_t time);

private:
    libKitFox::kitfox_t *kitfox;
    size_t comp_num;
    int ranks;

    std::vector<std::pair<manifold::kernel::CompId_t, manifold::uarch::KitFoxType>> manifold_node;
#if 0
    std::vector<std::pair<libKitFox::Comp_ID, libKitFox::counter_t*> > kitfox_power_component;
    std::vector<libKitFox::Comp_ID> kitfox_thermal_component;
    std::vector<libKitFox::Comp_ID> kitfox_reliability_component;
#endif
};


template <typename T>
void kitfox_proxy_t::handle_kitfox_proxy_response(int temp, kitfox_proxy_request_t<T> *Req)
{
    assert(Req != NULL);
    T c = Req->get_counter();
    string prefix;
    manifold::kernel::Time_t t_mem_layer;
    manifold::kernel::Time_t t_logic_layer;

    if (Req->get_type() == manifold::uarch::KitFoxType::core_type)
    {
        prefix = "package.core_die.core" + std::to_string(Req->get_id());
        calculate_power(c, Req->get_time(), prefix);
    }
    else if (Req->get_type() == manifold::uarch::KitFoxType::l1cache_type)
    {
        prefix = "package.core_die.l1cache" + std::to_string(Req->get_id());
        calculate_power(c, Req->get_time(), prefix);
    }
    else if (Req->get_type() == manifold::uarch::KitFoxType::l2cache_type)
    {
        prefix = "package.llc_die.cache" + std::to_string(Req->get_id());
        calculate_power(c, Req->get_time(), prefix);
    }
    else if (Req->get_type() == manifold::uarch::KitFoxType::hmcxbar_type)
    {
        prefix = "package.logic_die.interconnect" + std::to_string(Req->get_id());
        calculate_power(c, Req->get_time(), prefix);
        t_logic_layer = Req->get_time();
    }
    else if (Req->get_type() == manifold::uarch::KitFoxType::hmcserdes_type)
    {
        prefix = "package.logic_die.serdes" + std::to_string(Req->get_id());
        calculate_power(c, Req->get_time(), prefix);
    }
    else if (Req->get_type() == manifold::uarch::KitFoxType::dram_type)
    {
        prefix = std::to_string(Req->get_id()); // reusing variable name
        calculate_power(c, Req->get_time(), prefix);
        t_mem_layer = Req->get_time();
    }
    else
    {
        cerr << "unknown counter type!" << endl;
        exit(1);
    }

    comp_num += 1;
    if (comp_num == manifold_node.size()) {

//        assert(synchronize_vault_power(t_mem_layer) == 0);
        if(ranks != 0)
            assert(synchronize_logic_layer_power(t_logic_layer) == 0);
        libKitFox::Comp_ID package_id = kitfox->get_component_id("package");
        assert(package_id != INVALID_COMP_ID);
//        kitfox->calculate_temperature(package_id, Req->get_time(), m_clk->period);

        for(unsigned int i = 0, core_id = 0, l1_id = 0, l2_id = 0,
            hmcvault_id = 0, hmcxbar_id =0, hmcserdes_id = 0; i < manifold_node.size(); i++)
        {
            libKitFox::Kelvin t;
            string partition;
            libKitFox::Comp_ID par_id;

            switch (manifold_node[i].second)
            {
            case manifold::uarch::KitFoxType::core_type:
            {
                partition = "package.core_die.core" + std::to_string(core_id++);
                par_id = kitfox->get_component_id(partition);
//                assert(kitfox->pull_data(par_id, Req->get_time(), m_clk->period, libKitFox::KITFOX_DATA_TEMPERATURE, &t) == libKitFox::KITFOX_QUEUE_ERROR_NONE);

//                cerr << partition + ".temperature = " << t << "Kelvin @" << Req->get_time() << endl;
                break;
            }
            case manifold::uarch::KitFoxType::l1cache_type:
            {
                partition = "package.core_die.l1cache" + std::to_string(l1_id++);
                par_id = kitfox->get_component_id(partition);
//                assert(kitfox->pull_data(par_id, Req->get_time(), m_clk->period, libKitFox::KITFOX_DATA_TEMPERATURE, &t) == libKitFox::KITFOX_QUEUE_ERROR_NONE);

//                cerr << partition + ".temperature = " << t << "Kelvin @" << Req->get_time() << endl;
                break;
            }
            case manifold::uarch::KitFoxType::l2cache_type:
            {
                partition = "package.llc_die.cache" + std::to_string(l2_id++);
                par_id = kitfox->get_component_id(partition);
//                assert(kitfox->pull_data(par_id, Req->get_time(), m_clk->period, libKitFox::KITFOX_DATA_TEMPERATURE, &t) == libKitFox::KITFOX_QUEUE_ERROR_NONE);

//                cerr << partition + ".temperature = " << t << "Kelvin @" << Req->get_time() << endl;
                break;
            }
            case manifold::uarch::KitFoxType::hmcxbar_type:
            {
                partition = "package.logic_die.interconnect0";
                par_id = kitfox->get_component_id(partition);
//                assert(kitfox->pull_data(par_id, Req->get_time(), m_clk->period, libKitFox::KITFOX_DATA_TEMPERATURE, &t) == libKitFox::KITFOX_QUEUE_ERROR_NONE);

//                cerr << partition + ".temperature = " << t << "Kelvin @" << Req->get_time() << endl;
                hmcxbar_id++;
                break;
            }
            case manifold::uarch::KitFoxType::hmcserdes_type:
            {
                partition = "package.logic_die.serdes" + std::to_string(hmcserdes_id++);
                par_id = kitfox->get_component_id(partition);
//                assert(kitfox->pull_data(par_id, Req->get_time(), m_clk->period, libKitFox::KITFOX_DATA_TEMPERATURE, &t) == libKitFox::KITFOX_QUEUE_ERROR_NONE);

//                cerr << partition + ".temperature = " << t << "Kelvin @" << Req->get_time() << endl;
                break;
            }
            case manifold::uarch::KitFoxType::dram_type:
            {
                for(int k = 0; k < ranks;k++)
                {
                    partition = "package.memory_die" + std::to_string(k) + ".vault" + std::to_string(hmcvault_id);
                    par_id = kitfox->get_component_id(partition);
//                    assert(kitfox->pull_data(par_id, Req->get_time(), m_clk->period, libKitFox::KITFOX_DATA_TEMPERATURE, &t) == libKitFox::KITFOX_QUEUE_ERROR_NONE);

//                    cerr << partition + ".temperature = " << t << "Kelvin @" << Req->get_time() << endl;
                }
                hmcvault_id++;
                break;
            }
            default:
                cerr << "error manifold_node type!" << endl;
                exit(1);
            }


        }

        // Print thermal grid
#if 0
        libKitFox::grid_t<libKitFox::Kelvin> thermal_grid;
        assert(kitfox->pull_data(package_id, Req->get_time(), m_clk->period, libKitFox::KITFOX_DATA_THERMAL_GRID, &thermal_grid) == libKitFox::KITFOX_QUEUE_ERROR_NONE);

        for(unsigned z = 0; z < thermal_grid.dies(); z++) {
            printf("die %d:\n", z);
            for(unsigned x = 0; x < thermal_grid.rows(); x++) {
                for(unsigned y = 0; y < thermal_grid.cols(); y++) {
                    printf("%3.2lf ", thermal_grid(x, y, z));
                }
                printf("\n");
            }
            printf("\n");
        }
        printf("\n");
#endif

        // reset kitfox component counter
        comp_num = 0;
    }

    delete Req;
}


} // namespace kitfox_proxy
} // namespace manifold

#endif // USE_KITFOX
#endif
