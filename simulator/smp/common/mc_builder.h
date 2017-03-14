#ifndef MC_BUILDER_H
#define MC_BUILDER_H

#include <map>
#include <libconfig.h++>
#include "CaffDRAM/Controller.h"
#include "DRAMSim2/dram_sim.h"
#include "uarch/DestMap.h"
#include "hmc_xbar.h"
#ifdef HUTDEBUG
#include "pkt_generator.h"
#endif


#ifdef HUTDEBUG
class SysBuilder_llp_HUT;
#else
class SysBuilder_llp;
class NetworkBuilder;
#endif

class MemControllerBuilder {
public:
    enum {CAFFDRAM, DRAMSIM, HMC}; //mc types

#ifdef HUTDEBUG
    MemControllerBuilder(SysBuilder_llp_HUT* b) : m_sysBuilder(b) {}
#else
    MemControllerBuilder(SysBuilder_llp* b) : m_sysBuilder(b) {}
#endif

    virtual ~MemControllerBuilder() {}

    virtual int get_type() = 0;

    virtual void read_config(libconfig::Config&) = 0;
    virtual void create_mcs(std::map<int, int>& id_lp) = 0;
#ifdef HUTDEBUG
    virtual void connect_mc_network() = 0;
#else
    virtual void connect_mc_network(NetworkBuilder*) = 0;
#endif
    virtual void print_config(std::ostream&);
    virtual void print_stats(std::ostream&) = 0;
    virtual void set_mc_map_obj(manifold::uarch::DestMap *mc_map) = 0;

protected:
#ifdef HUTDEBUG
    SysBuilder_llp_HUT* m_sysBuilder;
#else
    SysBuilder_llp* m_sysBuilder;
#endif

    unsigned m_NUM_MC;
    std::vector<double> m_CLOCK_FREQ;
    std::vector<manifold::kernel::Clock*> m_clocks;
    bool m_use_default_clock;
    std::map<int, int> m_mc_id_cid_map; //node id to cid map

};


#ifndef HUTDEBUG
//builder for CaffDRAM
class CaffDRAM_builder : public MemControllerBuilder {
public:
    CaffDRAM_builder(SysBuilder_llp* b) : MemControllerBuilder(b) {}

    int get_type() { return CAFFDRAM; }

    void read_config(libconfig::Config&);
    void create_mcs(std::map<int, int>& id_lp);
    void connect_mc_network(NetworkBuilder*);
    void set_mc_map_obj(manifold::uarch::DestMap *mc_map);

    manifold::caffdram::Dsettings& get_settings() { return m_dram_settings; }

    void print_config(std::ostream&);
    void print_stats(std::ostream&);

private:
    manifold::caffdram::Dsettings m_dram_settings;
    int m_MC_DOWNSTREAM_CREDITS; //credits for sending down to network
    int m_MEM_MSG_TYPE;
    int m_CREDIT_MSG_TYPE;
};



//builder for DramSim
class DramSim_builder : public MemControllerBuilder {
public:
    DramSim_builder(SysBuilder_llp* b) : MemControllerBuilder(b) {}

    int get_type() { return DRAMSIM; }

    void read_config(libconfig::Config&);
    void create_mcs(std::map<int, int>& id_lp);
    void connect_mc_network(NetworkBuilder*);
    void set_mc_map_obj(manifold::uarch::DestMap *mc_map);

    void print_config(std::ostream&);
    void print_stats(std::ostream&);

private:
    int m_MC_DOWNSTREAM_CREDITS; //credits for sending down to network
    int m_MEM_MSG_TYPE;
    int m_CREDIT_MSG_TYPE;
    std::string m_DEV_FILE; //device file name
    std::string m_SYS_FILE; //system file name
    unsigned m_MEM_SIZE; //mem size;
};

#endif // HUTDEBUG

//builder for HMC
class HMC_builder : public MemControllerBuilder
{
public:
#ifdef HUTDEBUG
	HMC_builder(SysBuilder_llp_HUT* b) : MemControllerBuilder(b) {}
#else
	HMC_builder(SysBuilder_llp* b) : MemControllerBuilder(b) {}
#endif

	int get_type() { return HMC; }

    void read_config(libconfig::Config&);
    void create_mcs(std::map<int, int>& id_lp);
#ifdef HUTDEBUG
    void connect_mc_network();
    static int HMCLog2(unsigned num)
	{
		assert(num > 0);

		int bits = 0;
		while(((unsigned)0x1 << bits) < num)
		{
			bits++;
		}
		return bits;
	}
#else
    void connect_mc_network(NetworkBuilder*);
#endif
    void set_mc_map_obj(manifold::uarch::DestMap *hmc_map);

    void print_config(std::ostream&);
    void print_stats(std::ostream&);

private:
    int m_HMC_DOWNSTREAM_CREDITS; //credits for sending up to network
    int m_HMC_UPSTREAM_CREDITS; //credits for sending down to vault
    int m_MC_DOWNSTREAM_CREDITS; //credits for sending up to HMC xbar
    int m_MEM_MSG_TYPE;
    int m_CREDIT_MSG_TYPE;
    std::string m_DEV_FILE; //device file name
    std::string m_SYS_FILE; //system file name
    unsigned m_VAULT_SIZE; //size of each vault
    unsigned m_MEM_SIZE; //mem size = m_NUM_VAULTS x m_VAULT_SIZE
    unsigned m_NUM_VAULTS; //number of vaults in each HMC
    unsigned m_NUM_SERDES; //number of SerDes links to each HMC
    unsigned m_NUM_HMCs; // number of HMCs
    std::vector<int> vault_node_idx_vec;
    std::set<int> vault_node_idx_set; //set is used to ensure each index is unique

    void create_drams(int lp, int* hmc_cid, manifold::uarch::PageBasedMap *mc_map);
    //function to create drams from within create_mcs();

protected:
    bool hmc_use_default_clock;
    std::vector<double> hmc_CLOCK_FREQ;
    std::vector<manifold::kernel::Clock*> hmc_clocks; //clock for the HMC xbar
    std::map<int, int> m_hmc_id_cid_map; //hmc id to cid map
    std::map<int,int>* m_vault_id_cid_map; //vault id to cid map
#ifdef HUTDEBUG
    std::map<int,int> m_pkt_gen_id_cid_map;
#endif


};

#endif // #ifndef MC_BUILDER_H
