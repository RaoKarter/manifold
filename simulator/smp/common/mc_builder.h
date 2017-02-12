#ifndef MC_BUILDER_H
#define MC_BUILDER_H

#include <map>
#include <libconfig.h++>
#include "CaffDRAM/Controller.h"
#include "DRAMSim2/dram_sim.h"
#include "uarch/DestMap.h"
#include "hmc_xbar.h"



class SysBuilder_llp;
class NetworkBuilder;

class MemControllerBuilder {
public:
    enum {CAFFDRAM, DRAMSIM, HMC}; //mc types

    MemControllerBuilder(SysBuilder_llp* b) : m_sysBuilder(b) {}

    virtual ~MemControllerBuilder() {}

    virtual int get_type() = 0;

    virtual void read_config(libconfig::Config&) = 0;
    virtual void create_mcs(std::map<int, int>& id_lp) = 0;
    virtual void connect_mc_network(NetworkBuilder*) = 0;
/*  RAO Additions ------------------------*/
/*    virtual void create_vault_mcs(std::map<int, int>& id_lp) = 0;
    virtual void create_xbar(std::map<int, int>& id_lp) = 0;
    virtual void connect_xbar_network(NetworkBuilder*) = 0;
    virtual void connect_mc_xbar(NetworkBuilder*) = 0; // TODO: See how llp cache works and use the same here
*/
/*----------------------------------------*/
    virtual void print_config(std::ostream&);
    virtual void print_stats(std::ostream&) = 0;
    virtual void set_mc_map_obj(manifold::uarch::DestMap *mc_map) = 0;

protected:
    SysBuilder_llp* m_sysBuilder;

    unsigned m_NUM_MC;
    std::vector<double> m_CLOCK_FREQ;
    std::vector<manifold::kernel::Clock*> m_clocks;
    bool m_use_default_clock;
    std::map<int, int> m_mc_id_cid_map; //node id to cid map
};



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


//builder for HMC
class HMC_builder : public MemControllerBuilder
{
public:
	HMC_builder(SysBuilder_llp* b) : MemControllerBuilder(b) {}

	int get_type() { return HMC; }

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
    unsigned m_VAULT_SIZE; //size of each vault
    unsigned m_MEM_SIZE; //mem size = m_NUM_VAULTS x m_VAULT_SIZE
    unsigned m_NUM_VAULTS; //number of vaults in HMC
    unsigned m_NUM_SERDES; //number of SerDes links to HMC
    unsigned m_NUM_XBAR_MC_PORTS; //number of ports for each instance of xbar

    int create_xbar(int,int); //function to create xbars from within create_mcs();
    void connect_xbar_mc(); //function to connect xbar instances with DRAMSim2 instances

protected:
    bool xbar_use_default_clock;
    std::vector<double> xbar_CLOCK_FREQ;
    std::vector<manifold::kernel::Clock*> xbar_clocks; //clock for the xbar switches
    std::map<int, int> m_xbar_id_cid_map; //node id to cid map

};




#endif // #ifndef NETWORK_BUILDER_H
